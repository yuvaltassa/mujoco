// Reproducible, contact-free translational shake experiment.
#include <mujoco/mujoco.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int Id(const mjModel* m, int type, const std::string& name) {
  int id = mj_name2id(m, type, name.c_str());
  if (id < 0) throw std::runtime_error("missing " + name);
  return id;
}

double Distance(const mjtNum* a, const mjtNum* b) {
  double sum = 0;
  for (int k = 0; k < 3; ++k) sum += (a[k]-b[k])*(a[k]-b[k]);
  return std::sqrt(sum);
}

void Array(const mjtNum* x, int n) {
  std::cout << '[';
  for (int i = 0; i < n; ++i) std::cout << (i ? "," : "") << x[i];
  std::cout << ']';
}

void Audit(mjModel* m, mjData* d) {
  mj_forward(m, d);
  std::vector<mjtNum> mass(m->nv*m->nv);
  mj_fullM(m, d, mass.data());
  std::cout << std::setprecision(17) << "{\"version\":\"" << mj_versionString()
            << "\",\"num_bytes\":" << sizeof(mjtNum) << ",\"nv\":" << m->nv
            << ",\"ne\":" << d->ne << ",\"mass_matrix\":";
  Array(mass.data(), m->nv*m->nv);
  std::cout << ",\"equality_jacobian\":";
  Array(d->efc_J, d->ne*m->nv);
  std::cout << ",\"equality_residual\":";
  Array(d->efc_pos, d->ne);
  std::cout << ",\"diagonal\":";
  Array(d->efc_diagA, d->ne);
  std::cout << ",\"regularizer\":";
  Array(d->efc_R, d->ne);
  std::cout << ",\"equality_force\":";
  Array(d->efc_force, d->ne);
  std::cout << ",\"joints\":[";
  for (int j = 0; j < m->njnt; ++j) {
    std::cout << (j ? "," : "") << "\"" << mj_id2name(m, mjOBJ_JOINT, j) << "\"";
  }
  std::cout << "],\"bodies\":[";
  for (int b = 0; b < m->nbody; ++b) {
    std::cout << (b ? "," : "") << "{\"name\":\"" << mj_id2name(m, mjOBJ_BODY, b)
              << "\",\"mass\":" << m->body_mass[b] << ",\"com\":";
    Array(d->xipos+3*b, 3);
    std::cout << ",\"inertia\":";
    Array(m->body_inertia+3*b, 3);
    std::cout << ",\"local_com\":";
    Array(m->body_ipos+3*b, 3);
    std::cout << ",\"local_iquat\":";
    Array(m->body_iquat+4*b, 4);
    std::cout << ",\"rotation\":";
    Array(d->ximat+9*b, 9);
    std::cout << '}';
  }
  std::cout << "],\"colliding_geoms\":";
  int collision = 0;
  for (int g = 0; g < m->ngeom; ++g) collision += (m->geom_contype[g] || m->geom_conaffinity[g]);
  std::cout << collision << "}\n";
}

double Smooth(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x*x*x*(10+x*(-15+6*x));
}

int main(int argc, char** argv) {
  try {
    if (argc != 3 && argc != 12) {
      throw std::runtime_error("runner model --audit | runner model csv axis peak freq dt ctrl exact integrator shake_seconds contacts");
    }
    char error[2048] = {};
    mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) throw std::runtime_error(error);
    mjData* d = mj_makeData(m);
    if (argc == 3) {
      if (std::string(argv[2]) == "--diagnose") {
        m->opt.enableflags |= mjENBL_DIAGEXACT;
        d->ctrl[0] = 128;
      } else if (std::string(argv[2]) != "--audit") {
        throw std::runtime_error("expected --audit or --diagnose");
      }
      Audit(m, d);
      mj_deleteData(d);
      mj_deleteModel(m);
      return 0;
    }
    int axis = std::stoi(argv[3]);
    double peak = std::stod(argv[4]), freq = std::stod(argv[5]);
    m->opt.timestep = std::stod(argv[6]);
    double ctrl = std::stod(argv[7]);
    bool exact = std::stoi(argv[8]);
    std::string integrator = argv[9];
    double duration = std::stod(argv[10]);
    bool contacts = std::stoi(argv[11]);
    if (axis < 0 || axis > 2 || m->opt.timestep <= 0 || duration < 2 || freq <= 0 ||
        peak < 0 || ctrl < 0 || ctrl > 255) throw std::runtime_error("invalid run parameters");
    if (exact) m->opt.enableflags |= mjENBL_DIAGEXACT;
    else m->opt.enableflags &= ~mjENBL_DIAGEXACT;
    if (contacts) m->opt.disableflags &= ~mjDSBL_CONTACT;
    else m->opt.disableflags |= mjDSBL_CONTACT;
    if (integrator == "discrete") m->opt.integrator = mjINT_DISCRETE;
    else if (integrator == "implicitfast") m->opt.integrator = mjINT_IMPLICITFAST;
    else throw std::runtime_error("supported integrators: implicitfast, discrete");
    d->ctrl[Id(m, mjOBJ_ACTUATOR, "fingers_actuator")] = ctrl;
    int cf[2], cc[2], pad[2], wc[2], ws[2];
    for (int s = 0; s < 2; ++s) {
      std::string side = s ? "left" : "right";
      cf[s] = Id(m, mjOBJ_SITE, side+"_closure_f");
      cc[s] = Id(m, mjOBJ_SITE, side+"_closure_c");
      pad[s] = Id(m, mjOBJ_SITE, side+"_pad_center");
      wc[s] = mj_name2id(m, mjOBJ_SITE, (side+"_weld_c").c_str());
      ws[s] = mj_name2id(m, mjOBJ_SITE, (side+"_weld_s").c_str());
    }
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("cannot open CSV");
    out << std::setprecision(12)
        << "time,acceleration,closure_m,weld_m,weld_rad,aperture_m,eq_power_W,kinetic_J,relative_potential_J,actuator_force,ncon,iterations";
    for (int s = 0; s < 2; ++s) for (char c : std::string("xyz")) out << ',' << (s ? "left_" : "right_") << c;
    for (int j = 0; j < m->njnt; ++j) out << ',' << mj_id2name(m, mjOBJ_JOINT, j);
    out << '\n';
    constexpr double settle = 2;
    int steps = std::lround((settle+duration+1)/m->opt.timestep);
    double step_seconds = 0;
    bool failed = false;
    int samples = 0;
    for (int k = 0; k <= steps; ++k) {
      double t = k*static_cast<double>(m->opt.timestep), u = t-settle;
      double acceleration = peak*Smooth(u)*Smooth(duration-u)*std::sin(2*mjPI*freq*u);
      m->opt.gravity[0] = m->opt.gravity[1] = 0;
      m->opt.gravity[2] = -9.81;
      m->opt.gravity[axis] -= acceleration;
      mj_forward(m, d);
      double closure = 0, weld = 0, angle = 0, power = 0;
      for (int s = 0; s < 2; ++s) {
        closure = std::max(closure, Distance(d->site_xpos+3*cf[s], d->site_xpos+3*cc[s]));
        if (wc[s] >= 0) {
          weld = std::max(weld, Distance(d->site_xpos+3*wc[s], d->site_xpos+3*ws[s]));
          mjtNum qa[4], qb[4], delta[3];
          mju_mat2Quat(qa, d->site_xmat+9*wc[s]);
          mju_mat2Quat(qb, d->site_xmat+9*ws[s]);
          mju_subQuat(delta, qa, qb);
          angle = std::max(angle, static_cast<double>(mju_norm3(delta)));
        }
      }
      for (int e = 0; e < d->ne; ++e) power += d->efc_force[e]*d->efc_vel[e];
      int iterations = 0;
      for (int s = 0; s < d->nisland; ++s) iterations += d->solver_niter[s];
      out << t << ',' << acceleration << ',' << closure << ',' << weld << ',' << angle
          << ',' << Distance(d->site_xpos+3*pad[0], d->site_xpos+3*pad[1]) << ',' << power
          << ',' << d->energy[1] << ',' << d->energy[0] << ',' << d->actuator_force[0]
          << ',' << d->ncon << ',' << iterations;
      for (int s = 0; s < 2; ++s) for (int a = 0; a < 3; ++a) out << ',' << d->site_xpos[3*pad[s]+a];
      for (int j = 0; j < m->nq; ++j) out << ',' << d->qpos[j];
      out << '\n';
      ++samples;
      for (int w = 0; w < mjNWARNING; ++w) failed |= d->warning[w].number != 0;
      for (int j = 0; j < m->nv; ++j) failed |= !std::isfinite(d->qvel[j]);
      failed |= !std::isfinite(closure) || closure > .1;
      if (failed || k == steps) break;
      double before = d->time;
      auto start = std::chrono::steady_clock::now();
      mj_step(m, d);
      step_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      if (d->time <= before) { failed = true; break; }
    }
    std::cout << std::setprecision(12) << "{\"failed\":" << (failed ? "true" : "false")
              << ",\"samples\":" << samples << ",\"step_seconds\":" << step_seconds
              << ",\"num_bytes\":" << sizeof(mjtNum) << ",\"nv\":" << m->nv << "}\n";
    mj_deleteData(d);
    mj_deleteModel(m);
    return failed ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

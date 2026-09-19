// Force-transfer benchmark; planar rows are a lab-only projection and compaction.
#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "planar_rows.h"
#include "regularization.h"

inline int row_mode;
void Rows(const mjModel* m, mjData* d) {
  if (row_mode == 1)
    PlanarRows(m, d);
  else if (row_mode == 2)
    LabRegularize(m, d);
}
int Id(const mjModel* m, int type, const char* name) {
  int id = mj_name2id(m, type, name);
  if (id < 0) throw std::runtime_error(std::string("missing ") + name);
  return id;
}
void Array(const mjtNum* x, int n) {
  std::cout << '[';
  for (int i = 0; i < n; ++i) std::cout << (i ? "," : "") << x[i];
  std::cout << ']';
}
double Distance(const mjtNum* a, const mjtNum* b) {
  double sum = 0;
  for (int k = 0; k < 3; ++k) sum += (a[k] - b[k]) * (a[k] - b[k]);
  return std::sqrt(sum);
}
void Geometry(double theta, double length, double& phi, double& ratio, double& psi_derivative) {
  double bx = .1 * std::cos(theta), by = .1 * std::sin(theta), vx = .3 - bx, vy = -by;
  double distance = std::hypot(vx, vy),
         along = (length * length - .04 + distance * distance) / (2 * distance);
  double height = std::sqrt(std::max(0.0, length * length - along * along));
  double cx = bx + along * vx / distance - height * vy / distance;
  double cy = by + along * vy / distance + height * vx / distance;
  phi = std::atan2(cy, cx - .3);
  double rx = cx - bx, ry = cy - by, dx = -.1 * std::sin(theta), dy = .1 * std::cos(theta);
  double tx = -.2 * std::sin(phi), ty = .2 * std::cos(phi);
  ratio = (rx * dx + ry * dy) / (rx * tx + ry * ty);
  psi_derivative = (rx * (ty * ratio - dy) - ry * (tx * ratio - dx)) / (length * length);
}
int main(int argc, char** argv) {
  try {
    if (argc != 5 && argc != 10 && argc != 11)
      throw std::runtime_error(
          "fourbar_runner model --audit rows exact | model csv mode rows exact dt load duration "
          "frequency [drive_kv]");
    char error[2048] = {};
    mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) throw std::runtime_error(error);
    bool audit = argc == 5;
    row_mode = std::stoi(argv[audit ? 3 : 4]);
    plane_enabled = row_mode == 1;
    bool exact = std::stoi(argv[audit ? 4 : 5]);
    if (exact)
      m->opt.enableflags |= mjENBL_DIAGEXACT;
    else
      m->opt.enableflags &= ~mjENBL_DIAGEXACT;
    if (row_mode < 0 || row_mode > 2 || m->opt.jacobian != mjJAC_DENSE ||
        m->opt.integrator != mjINT_IMPLICITFAST || m->opt.solver != mjSOL_NEWTON ||
        m->opt.noslip_iterations || !(m->opt.disableflags & mjDSBL_ISLAND))
      throw std::runtime_error("unsupported benchmark options");
    lab_regularization = "floor";
    lab_floor = 1e-6;
    plane_frame = Id(m, mjOBJ_BODY, "frame");
    mjcb_control = Rows;
    mjData* d = mj_makeData(m);
    if (audit) {
      mj_forward(m, d);
      std::vector<mjtNum> mass(m->nv * m->nv);
      mj_fullM(m, d, mass.data());
      std::cout << std::setprecision(17) << "{\"nv\":" << m->nv << ",\"ne\":" << d->ne
                << ",\"num_bytes\":" << sizeof(mjtNum) << ",\"J\":";
      Array(d->efc_J, d->ne * m->nv);
      std::cout << ",\"M\":";
      Array(mass.data(), m->nv * m->nv);
      std::cout << ",\"residual\":";
      Array(d->efc_pos, d->ne);
      std::cout << ",\"joints\":[";
      for (int i = 0; i < m->njnt; ++i)
        std::cout << (i ? "," : "") << '"' << mj_id2name(m, mjOBJ_JOINT, i) << '"';
      std::cout << "]}\n";
      mj_deleteData(d);
      mj_deleteModel(m);
      return 0;
    }
    std::string mode = argv[3];
    m->opt.timestep = std::stod(argv[6]);
    if (argc == 11) m->actuator_biasprm[2] = -std::stod(argv[10]);
    double load = std::stod(argv[7]), duration = std::stod(argv[8]), frequency = std::stod(argv[9]);
    if ((mode != "static" && mode != "calibrate" && mode != "dynamic" && mode != "passive") ||
        m->opt.timestep <= 0 || duration <= 0 || frequency <= 0)
      throw std::runtime_error("invalid experiment parameters");
    const mjtNum* reference = m->numeric_data + m->numeric_adr[Id(m, mjOBJ_NUMERIC, "reference")];
    double theta0 = reference[0], phi0 = reference[1], length = reference[4];
    int in = Id(m, mjOBJ_JOINT, "input"), out = Id(m, mjOBJ_JOINT, "output");
    int qi = m->jnt_qposadr[in], qo = m->jnt_qposadr[out], vi = m->jnt_dofadr[in],
        vo = m->jnt_dofadr[out];
    int tip1 = Id(m, mjOBJ_SITE, "input_tip"), tip2 = Id(m, mjOBJ_SITE, "output_tip");
    int coupler = mj_name2id(m, mjOBJ_SITE, "coupler_tip");
    if (mode == "passive") {
      m->actuator_gainprm[0] = 0;
      m->actuator_biasprm[1] = m->actuator_biasprm[2] = 0;
      double phi, ratio, psid;
      Geometry(theta0, length, phi, ratio, psid);
      d->qvel[vi] = 1;
      d->qvel[vo] = ratio;
      int joint = mj_name2id(m, mjOBJ_JOINT, "coupler");
      if (joint >= 0) d->qvel[m->jnt_dofadr[joint]] = psid - 1;
      joint = mj_name2id(m, mjOBJ_JOINT, "shadow");
      if (joint >= 0) d->qvel[m->jnt_dofadr[joint]] = psid - ratio;
    }
    std::ofstream file(argv[2]);
    if (!file) throw std::runtime_error("cannot open output");
    file << std::setprecision(12)
         << "time,input_angle,output_angle,input_velocity,output_velocity,input_acceleration,input_"
            "torque,output_torque,static_reference_torque,deflection_m,span_error_m,closure_m,"
            "kinetic_J,input_power_W,output_power_W,equality_power_W,rows";
    for (int j = 0; j < m->nq; ++j) file << ",q" << j;
    file << '\n';
    bool failed = false;
    double step_seconds = 0;
    int samples = 0;
    int steps = std::lround(duration / m->opt.timestep);
    for (int k = 0; k <= steps; ++k) {
      double t = k * static_cast<double>(m->opt.timestep);
      double ramp = std::min(1.0, t);
      ramp = ramp * ramp * ramp * (10 + ramp * (-15 + 6 * ramp));
      d->ctrl[0] =
          mode == "dynamic" ? (.17453292519943295 * ramp * std::sin(2 * mjPI * frequency * t)) : 0;
      double torque = 0;
      if (mode == "static") torque = -.2 * load * std::min(1.0, t / .5);
      if (mode == "calibrate") torque = t >= .25 && t < 1.25 ? -.2 * load : 0;
      if (mode == "dynamic") torque = -load * d->qvel[vo];
      d->qfrc_applied[vo] = torque;
      mj_forward(m, d);
      double theta = theta0 + d->qpos[qi], actual_phi = phi0 + d->qpos[qo], phi, ratio, psid;
      Geometry(theta, length, phi, ratio, psid);
      double deflection = .2 * std::atan2(std::sin(actual_phi - phi), std::cos(actual_phi - phi));
      double span = Distance(d->site_xpos + 3 * tip1, d->site_xpos + 3 * tip2) - length;
      double closure = coupler >= 0 ? Distance(d->site_xpos + 3 * coupler, d->site_xpos + 3 * tip2)
                                    : std::abs(span);
      double power = mju_dot(d->qfrc_constraint, d->qvel, m->nv), input = d->actuator_force[0];
      file << t << ',' << theta << ',' << actual_phi << ',' << d->qvel[vi] << ',' << d->qvel[vo]
           << ',' << d->qacc[vi] << ',' << input << ',' << torque << ',' << -torque * ratio << ','
           << deflection << ',' << span << ',' << closure << ',' << d->energy[1] << ','
           << input * d->qvel[vi] << ',' << torque * d->qvel[vo] << ',' << power << ',' << d->ne;
      for (int j = 0; j < m->nq; ++j) file << ',' << d->qpos[j];
      file << '\n';
      ++samples;
      for (int j = 0; j < m->nv; ++j) failed |= !std::isfinite(d->qvel[j]);
      for (int w = 0; w < mjNWARNING; ++w) failed |= d->warning[w].number > 0;
      failed |= !std::isfinite(closure) || closure > .1 || std::abs(d->qpos[qi]) > 6;
      if (failed || k == steps) break;
      double before = d->time;
      auto start = std::chrono::steady_clock::now();
      mj_step(m, d);
      step_seconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (d->time <= before) {
        failed = true;
        break;
      }
    }
    std::cout << std::setprecision(12) << "{\"failed\":" << (failed ? "true" : "false")
              << ",\"samples\":" << samples << ",\"num_bytes\":" << sizeof(mjtNum)
              << ",\"step_seconds\":" << step_seconds << "}\n";
    mj_deleteData(d);
    mj_deleteModel(m);
    return failed ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

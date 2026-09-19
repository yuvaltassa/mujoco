// Close, release a temporary fixture, and shake a box in translating-base coordinates.
#include <mujoco/mujoco.h>
#include "regularization.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int Id(const mjModel* m, int type, const char* name) {
  int id = mj_name2id(m, type, name);
  if (id < 0) throw std::runtime_error(std::string("missing ")+name);
  return id;
}

double Smooth(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x*x*x*(10+x*(-15+6*x));
}

double Distance(const mjtNum* a, const mjtNum* b) {
  double sum = 0;
  for (int k = 0; k < 3; ++k) sum += (a[k]-b[k])*(a[k]-b[k]);
  return std::sqrt(sum);
}

int main(int argc, char** argv) {
  try {
    if (argc != 10) throw std::runtime_error("grasp_runner model csv exact dt peak axis control frequency duration");
    char error[2048] = {};
    mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) throw std::runtime_error(error);
    if (std::stoi(argv[3])) m->opt.enableflags |= mjENBL_DIAGEXACT;
    else m->opt.enableflags &= ~mjENBL_DIAGEXACT;
    m->opt.disableflags &= ~mjDSBL_CONTACT;
    m->opt.timestep = std::stod(argv[4]);
    double peak = std::stod(argv[5]);
    int axis = std::stoi(argv[6]);
    double control = std::stod(argv[7]), frequency = std::stod(argv[8]), duration = std::stod(argv[9]);
    if (axis < 0 || axis > 2 || m->opt.timestep <= 0 || duration < 2 || frequency <= 0 ||
        peak < 0 || control < 0 || control > 255) throw std::runtime_error("invalid parameters");
    LabConfigure(m);
    mjData* d = mj_makeData(m);
    int object = Id(m, mjOBJ_BODY, "object");
    int geom = Id(m, mjOBJ_GEOM, "object_geom");
    int fixture = Id(m, mjOBJ_EQUALITY, "fixture");
    int actuator = Id(m, mjOBJ_ACTUATOR, "fingers_actuator");
    int pads[4] = {Id(m, mjOBJ_GEOM, "right_pad1"), Id(m, mjOBJ_GEOM, "right_pad2"),
                   Id(m, mjOBJ_GEOM, "left_pad1"), Id(m, mjOBJ_GEOM, "left_pad2")};
    int cf[2] = {Id(m, mjOBJ_SITE, "right_closure_f"), Id(m, mjOBJ_SITE, "left_closure_f")};
    int cc[2] = {Id(m, mjOBJ_SITE, "right_closure_c"), Id(m, mjOBJ_SITE, "left_closure_c")};
    mjtNum target[3];
    mju_copy3(target, m->body_pos+3*object);
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("cannot write trace");
    out << std::setprecision(12)
        << "time,acceleration,fixture,object_x,object_y,object_z,drop_m,displacement_m,angle_rad,closure_m,right_normal_N,left_normal_N,tangent_N,penetration_m,pad_contacts,other_object_contacts,self_contacts,actuator_force,eq_power_W";
    for (int j = 0; j < m->nq; ++j) out << ",q" << j;
    out << '\n';
    int steps = std::lround((3+duration)/m->opt.timestep);
    bool failed = false, dropped = false;
    int samples = 0;
    for (int k = 0; k <= steps; ++k) {
      double t = k*static_cast<double>(m->opt.timestep), u = t-2;
      double acceleration = peak*Smooth(u)*Smooth(duration-u)*std::sin(2*mjPI*frequency*u);
      d->ctrl[actuator] = control*Smooth(t/.8);
      d->eq_active[fixture] = t < 1.2;
      m->opt.gravity[0] = m->opt.gravity[1] = 0;
      m->opt.gravity[2] = -9.81;
      m->opt.gravity[axis] -= acceleration;
      mj_forward(m, d);
      double closure = 0, normal[2] = {}, tangent = 0, penetration = 0, power = 0;
      int pad_contacts = 0, other_contacts = 0, self_contacts = 0;
      for (int s = 0; s < 2; ++s) closure = std::max(closure, Distance(d->site_xpos+3*cf[s], d->site_xpos+3*cc[s]));
      for (int c = 0; c < d->ncon; ++c) {
        const mjContact& con = d->contact[c];
        int other = con.geom[0] == geom ? con.geom[1] : con.geom[1] == geom ? con.geom[0] : -1;
        if (other < 0) { ++self_contacts; continue; }
        penetration = std::max(penetration, -static_cast<double>(con.dist));
        int side = -1;
        for (int j = 0; j < 4; ++j) if (other == pads[j]) side = j/2;
        if (side < 0) { ++other_contacts; continue; }
        ++pad_contacts;
        mjtNum force[6];
        mj_contactForce(m, d, c, force);
        normal[side] += force[0];
        tangent += std::hypot(force[1], force[2]);
      }
      for (int e = 0; e < d->ne; ++e) power += d->efc_force[e]*d->efc_vel[e];
      double displacement = Distance(d->xpos+3*object, target);
      double drop = target[2]-d->xpos[3*object+2];
      double angle = 2*std::acos(std::min(1.0, std::abs(static_cast<double>(d->xquat[4*object]))));
      out << t << ',' << acceleration << ',' << static_cast<int>(d->eq_active[fixture]);
      for (int j = 0; j < 3; ++j) out << ',' << d->xpos[3*object+j];
      out << ',' << drop << ',' << displacement << ',' << angle << ',' << closure << ','
          << normal[0] << ',' << normal[1] << ',' << tangent << ',' << penetration << ','
          << pad_contacts << ',' << other_contacts << ',' << self_contacts << ','
          << d->actuator_force[actuator] << ',' << power;
      for (int j = 0; j < m->nq; ++j) out << ',' << d->qpos[j];
      out << '\n';
      ++samples;
      for (int w = 0; w < mjNWARNING; ++w) failed |= d->warning[w].number != 0;
      for (int j = 0; j < m->nv; ++j) failed |= !std::isfinite(d->qvel[j]);
      failed |= !std::isfinite(displacement) || closure > .1;
      dropped |= t >= 1.2 && (drop > .05 || displacement > .1);
      if (failed || dropped || k == steps) break;
      double before = d->time;
      mj_step(m, d);
      if (d->time <= before) { failed = true; break; }
    }
    std::cout << "{\"failed\":" << (failed ? "true" : "false")
              << ",\"dropped\":" << (dropped ? "true" : "false")
              << ",\"samples\":" << samples << ",\"num_bytes\":" << sizeof(mjtNum) << "}\n";
    mj_deleteData(d);
    mj_deleteModel(m);
    return failed ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

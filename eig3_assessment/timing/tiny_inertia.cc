// small-scale fullinertia: what does each library compile?
#include <cstdio>
#include <string>
#include <mujoco/mujoco.h>
static void run(const char* label, const char* full, const char* mass) {
  std::string xml = std::string("<mujoco><worldbody><body><joint/><inertial pos='0 0 0' mass='") + mass +
                    "' fullinertia='" + full + "'/></body></worldbody></mujoco>";
  char err[500] = "";
  mjSpec* s = mj_parseXMLString(xml.c_str(), nullptr, err, sizeof(err));
  mjModel* m = s ? mj_compile(s, nullptr) : nullptr;
  if (!m) { std::printf("%-34s -> ERROR: %s\n", label, s ? mjs_getError(s) : err); if (s) mj_deleteSpec(s); return; }
  // reconstruct tensor
  mjtNum R[9]; mju_quat2Mat(R, m->body_iquat + 4);
  double T[9] = {0};
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) T[3*i+j] += (double)R[3*i+k]*(double)m->body_inertia[3+k]*(double)R[3*j+k];
  std::printf("%-34s -> Ixx Iyy Izz = %.6g %.6g %.6g | Ixy Ixz Iyz = %.6g %.6g %.6g\n", label, T[0], T[4], T[8], T[1], T[2], T[5]);
  mj_deleteModel(m); mj_deleteSpec(s);
}
int main() {
  run("unit scale   (2 3 4 .1 .2 .3)", "2 3 4 .1 .2 .3", "2");
  run("1e-7 scale   (finger-tip sized)", "2e-7 3e-7 4e-7 1e-8 2e-8 3e-8", "0.01");
  run("1e-10 scale  (mm-sized part)", "2e-10 3e-10 4e-10 1e-11 2e-11 3e-11", "1e-4");
  run("1e-13 scale  (insect, SI units)", "2e-13 3e-13 4e-13 1e-14 2e-14 3e-14", "1e-6");
  run("1e-13 scale, NOT positive-definite", "1e-13 1e-13 1e-13 2e-13 0 0", "1e-6");
}

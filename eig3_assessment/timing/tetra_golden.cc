// print mesh_vert of the four FlippedFaceAllowed* test models at full precision
#include <cstdio>
#include <cstring>
#include <string>
#include <mujoco/mujoco.h>
static const char* kBodies[4] = {
  "<worldbody><geom type=\"mesh\" mesh=\"m\"/></worldbody>",
  "<worldbody><body><geom type=\"mesh\" mesh=\"m\" mass=\"0\"/></body></worldbody>",
  "<worldbody><body><inertial pos=\"0 0 0\" mass=\"1\"/><geom type=\"mesh\" mesh=\"m\"/></body></worldbody>",
  "<worldbody><body><geom type=\"mesh\" mesh=\"m\"/></body></worldbody>"};
int main() {
  for (int k = 0; k < 4; k++) {
    std::string mesh = k < 3
      ? "<mesh name=\"m\" vertex=\"0 0 0  1 0 0  0 2 0  0 0 3\" face=\"2 0 3  0 1 3  1 2 3  0 1 2\"/>"
      : "<mesh name=\"m\" vertex=\"0 0 0  1 0 0  0 2 0  0 0 3  0 0 3\" face=\"2 0 3  0 1 3  1 2 3  0 2 1  0 3 4\"/>";
    std::string xml = "<mujoco><asset>" + mesh + "</asset>" + kBodies[k] + "</mujoco>";
    mjSpec* s = mj_parseXMLString(xml.c_str(), nullptr, nullptr, 0);
    mjModel* m = s ? mj_compile(s, nullptr) : nullptr;
    if (!m) { std::printf("model %d failed\n", k); return 1; }
    std::printf("model %d:", k);
    for (int i = 0; i < 12; i++) std::printf(" %.17g", (double)m->mesh_vert[i]);
    std::printf("\n");
    mj_deleteModel(m); mj_deleteSpec(s);
  }
}

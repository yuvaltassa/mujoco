#include <cstdio>
#include <mujoco/mujoco.h>
int main(int argc, char** argv) {
  char err[1000];
  mjModel* m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) { std::printf("%s\n", err); return 1; }
  mj_saveModel(m, argv[2], nullptr, 0);
  mj_deleteModel(m);
}

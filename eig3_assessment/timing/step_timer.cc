// A/B step timing: ns per mj_step and per broadphase, best of R repeats of N steps
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <mujoco/mujoco.h>
int main(int argc, char** argv) {
  const char* path = argv[1];
  int nstep = std::atoi(argv[2]), nrep = std::atoi(argv[3]);
  char err[1000];
  mjModel* m = mj_loadXML(path, nullptr, err, sizeof(err));
  if (!m) { std::printf("%s\n", err); return 1; }
  mjData* d = mj_makeData(m);
  std::vector<double> per_step;
  for (int r = 0; r < nrep; r++) {
    mj_resetData(m, d);
    if (m->nkey) mj_resetDataKeyframe(m, d, 0);
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < nstep; i++) mj_step(m, d);
    auto t1 = std::chrono::steady_clock::now();
    per_step.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / nstep);
  }
  std::sort(per_step.begin(), per_step.end());
  std::printf("%.1f %.1f %d %d\n", per_step.front(), per_step[per_step.size() / 2], (int)m->nbody, (int)d->ncon);
  mj_deleteData(d); mj_deleteModel(m);
}

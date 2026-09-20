// Iteration counts of candidate stopping rules on the covariances that the
// broadphase passes to mju_eig3, recorded along a simulated trajectory.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <mujoco/mujoco.h>
#include "../eig3_template.h"

int main(int argc, char** argv) {
  const char* path = argv[1];
  int nstep = std::atoi(argv[2]);
  char err[1000];
  mjModel* m = mj_loadXML(path, nullptr, err, sizeof(err));
  if (!m) { std::printf("%s\n", err); return 1; }
  mjData* d = mj_makeData(m);
  if (m->nkey) mj_resetDataKeyframe(m, d, 0);
  std::vector<mjtNum> covs;
  for (int s = 0; s < nstep; s++) {
    mj_step(m, d);
    mjtNum cen[3] = {0, 0, 0}, cov[9] = {0};
    int cnt = 0;
    for (int i = 0; i < m->ngeom; i++) if (m->geom_bodyid[i]) { mju_addTo3(cen, d->geom_xpos + 3*i); cnt++; }
    if (!cnt) continue;
    mju_scl3(cen, cen, 1.0 / cnt);
    for (int i = 0; i < m->ngeom; i++) if (m->geom_bodyid[i]) {
      const mjtNum* p = d->geom_xpos + 3*i;
      mjtNum x[3] = {p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};
      for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) cov[3*a + b] += x[a] * x[b];
    }
    mju_scl(cov, cov, 1.0 / cnt, 9);
    covs.insert(covs.end(), cov, cov + 9);
  }
  const double eps = std::numeric_limits<mjtNum>::epsilon();
  struct { const char* name; eig3::Rule rule; } rules[] = {
    {"current", {"", true, 1e-12, 0, 0, false, false}},
    {"tight(16eps)", {"", false, 0, 16, 0, false, true}},
    {"loose(1e-6)", {"", false, 0, 1e-6 / eps, 0, false, true}},
    {"loose(1e-4)", {"", false, 0, 1e-4 / eps, 0, false, true}},
    {"loose(1e-3)", {"", false, 0, 1e-3 / eps, 0, false, true}},
    {"loose(1e-2)", {"", false, 0, 1e-2 / eps, 0, false, true}}};
  int n = covs.size() / 9;
  std::printf("%-18s", strrchr(path, '/') ? strrchr(path, '/') + 1 : path);
  for (auto& r : rules) {
    long sum = 0; int mx = 0;
    for (int i = 0; i < n; i++) {
      mjtNum w[3], v[9], q[4];
      int it = eig3::Eig3<mjtNum>(w, v, q, covs.data() + 9*i, r.rule);
      sum += it; mx = std::max(mx, it);
    }
    std::printf(" | %5.2f /%2d", (double)sum / std::max(1, n), mx);
  }
  std::printf("\n");
}

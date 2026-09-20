// Cost of mju_eig3 on the matrices the broadphase actually passes to it.
// Simulates a model, rebuilds the broadphase covariance at every step (as in
// mj_broadphase), then times mju_eig3 alone on the recorded matrices.
// Also prints a bitwise hash of the final state, to compare across libraries.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <mujoco/mujoco.h>

#ifdef USE_TOL
// internal variant used by the broadphase (exported for testing)
extern "C" int mju_eig3Tol(mjtNum eigval[3], mjtNum eigvec[9], mjtNum quat[4], const mjtNum mat[9],
                           mjtNum reltol);
#define EIG3(w, v, q, m) mju_eig3Tol(w, v, q, m, 1e-3)
#else
#define EIG3(w, v, q, m) mju_eig3(w, v, q, m)
#endif

int main(int argc, char** argv) {
  const char* path = argv[1];
  int nstep = std::atoi(argv[2]);
  char err[1000];
  size_t len = std::strlen(path);
  mjModel* m = (len > 4 && !std::strcmp(path + len - 4, ".mjb")) ? mj_loadModel(path, nullptr)
                                                                : mj_loadXML(path, nullptr, err, sizeof(err));
  if (!m) { std::printf("%s\n", err); return 1; }
  mjData* d = mj_makeData(m);
  if (m->nkey) mj_resetDataKeyframe(m, d, 0);

  std::vector<mjtNum> covs;
  auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < nstep; s++) {
    mj_step(m, d);
    // covariance of non-world geom positions and flex vertices, as in mj_broadphase
    mjtNum cen[3] = {0, 0, 0}, cov[9] = {0};
    int cnt = 0;
    for (int i = 0; i < m->ngeom; i++) if (m->geom_bodyid[i]) { mju_addTo3(cen, d->geom_xpos + 3*i); cnt++; }
    for (int i = 0; i < m->nflexvert; i++) if (m->flex_vertbodyid[i]) { mju_addTo3(cen, d->flexvert_xpos + 3*i); cnt++; }
    if (!cnt) continue;
    mju_scl3(cen, cen, 1.0 / cnt);
    auto add = [&](const mjtNum* p) {
      mjtNum x[3] = {p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]};
      for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) cov[3*a + b] += x[a] * x[b];
    };
    for (int i = 0; i < m->ngeom; i++) if (m->geom_bodyid[i]) add(d->geom_xpos + 3*i);
    for (int i = 0; i < m->nflexvert; i++) if (m->flex_vertbodyid[i]) add(d->flexvert_xpos + 3*i);
    mju_scl(cov, cov, 1.0 / cnt, 9);
    covs.insert(covs.end(), cov, cov + 9);
  }
  auto t1 = std::chrono::steady_clock::now();
  double step_ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / nstep;

  // bitwise hash of the final state (FNV-1a over qpos, qvel)
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](const mjtNum* p, int n) {
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n * sizeof(mjtNum); i++) { h ^= b[i]; h *= 1099511628211ull; }
  };
  mix(d->qpos, m->nq); mix(d->qvel, m->nv);

  // time mju_eig3 alone: best of 25 passes
  int n = covs.size() / 9, iter_max = 0;
  long iter_sum = 0;
  double best = 1e30, sink = 0;
  for (int rep = 0; rep < 25; rep++) {
    auto a = std::chrono::steady_clock::now();
    for (int i = 0; i < n; i++) {
      mjtNum w[3], v[9], q[4];
      int it = EIG3(w, v, q, covs.data() + 9*i);
      sink += w[0];
      if (!rep) { iter_sum += it; iter_max = std::max(iter_max, it); }
    }
    auto b = std::chrono::steady_clock::now();
    best = std::min(best, std::chrono::duration<double, std::nano>(b - a).count() / std::max(1, n));
  }
  std::printf("%.1f %.2f %d %.0f %016llx %d\n", best, (double)iter_sum / std::max(1, n), iter_max,
              step_ns, (unsigned long long)h, (int)(sink != sink));
  mj_deleteData(d); mj_deleteModel(m);
}

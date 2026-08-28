// Local utility (not for upstream): benchmark the elliptic cone-fold solver
// optimization against the stock rank-1-update path, replaying identical states
// so iteration counts match and the comparison is pure cost.
//
// For each simulated step we restore the pre-step state and time mj_forward
// twice -- once with folding disabled (MJ_CONE_FOLD huge) and once enabled --
// then assert the resulting qacc match. Toggling uses the MJ_CONE_FOLD env var
// that the (locally patched) solver reads.
//
// Usage: cone_bench model.xml [nsteps] [threshold] [reps] [dump.csv] [flags]
//   -tilt T        gravity = (-T, -T, -10): more tilt, more clumping
//   -condim N      set all geom_condim to N (3, 4, or 6)
//   -noisland      disable constraint islands
//   -nowarmstart   disable solver warmstart
//   -settle N      advance N steps before benching (reach the clumped regime)

#include <algorithm>
#include <chrono>  // NOLINT(build/c++11)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace {

double WallUs() {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// mjcb_time callback so d->timer[] is populated (microseconds).
mjtNum TimeCb() { return WallUs(); }

void ClearTimers(mjData* d) {
  for (int i = 0; i < mjNTIMER; i++) {
    d->timer[i].duration = 0;
    d->timer[i].number = 0;
  }
}

void SetFold(int thr) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%d", thr);
  setenv("MJ_CONE_FORCE", buf, 1);
}

// global and max-per-island counts of cone-state constraint rows; the fold
// threshold gates on the per-island count when islands are enabled
void CountCones(const mjData* d, int* ncone_total, int* ncone_island) {
  *ncone_total = 0;
  std::vector<int> per_island(d->nisland > 0 ? d->nisland : 1, 0);
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_state[i] == mjCNSTRSTATE_CONE) {
      (*ncone_total)++;
      int isl = d->nisland > 0 ? d->efc_island[i] : 0;
      if (isl >= 0) per_island[isl]++;
    }
  }
  *ncone_island = *std::max_element(per_island.begin(), per_island.end());
}

double Pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, (size_t)(p * v.size()))];
}
double Mean(const std::vector<double>& v) {
  double s = 0;
  for (double x : v) s += x;
  return v.empty() ? 0 : s / v.size();
}
double Max(const std::vector<double>& v) {
  return v.empty() ? 0 : *std::max_element(v.begin(), v.end());
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "Usage: %s model.xml [nsteps] [threshold] [reps] [dump.csv]\n"
                 "       [-tilt T] [-condim N] [-noisland] [-nowarmstart] [-settle N]\n",
                 argv[0]);
    return 1;
  }

  // positionals then flags
  const char* path = nullptr;
  const char* dump_path = nullptr;
  int nsteps = 2000, threshold = 0, reps = 3, settle = 0, condim = 0, npos = 0;
  double tilt = -1;
  bool noisland = false, nowarmstart = false, rollout = false;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "-rollout")) {
      rollout = true;
    } else if (!std::strcmp(argv[i], "-tilt") && i + 1 < argc) {
      tilt = std::atof(argv[++i]);
    } else if (!std::strcmp(argv[i], "-condim") && i + 1 < argc) {
      condim = std::atoi(argv[++i]);
    } else if (!std::strcmp(argv[i], "-settle") && i + 1 < argc) {
      settle = std::atoi(argv[++i]);
    } else if (!std::strcmp(argv[i], "-noisland")) {
      noisland = true;
    } else if (!std::strcmp(argv[i], "-nowarmstart")) {
      nowarmstart = true;
    } else if (argv[i][0] == '-') {
      std::fprintf(stderr, "unknown flag %s\n", argv[i]);
      return 1;
    } else {
      switch (npos++) {
        case 0: path = argv[i]; break;
        case 1: nsteps = std::atoi(argv[i]); break;
        case 2: threshold = std::atoi(argv[i]); break;
        case 3: reps = std::atoi(argv[i]); break;
        case 4: dump_path = argv[i]; break;
      }
    }
  }
  if (!path) {
    std::fprintf(stderr, "model path required\n");
    return 1;
  }

  mjcb_time = TimeCb;

  char err[1024];
  mjModel* m = mj_loadXML(path, nullptr, err, sizeof(err));
  if (!m) {
    std::fprintf(stderr, "%s\n", err);
    return 1;
  }

  // apply sweep axes
  if (tilt >= 0) {
    m->opt.gravity[0] = m->opt.gravity[1] = -tilt;
    m->opt.gravity[2] = -10;
  }
  if (condim) {
    for (int i = 0; i < m->ngeom; i++) m->geom_condim[i] = condim;
  }
  if (noisland) {
    m->opt.disableflags |= mjDSBL_ISLAND;
  }
  if (nowarmstart) {
    m->opt.disableflags |= mjDSBL_WARMSTART;
  }

  mjData* d = mj_makeData(m);

  // settle into the working regime (fold disabled: reference trajectory)
  SetFold(-1);
  for (int i = 0; i < settle; i++) mj_step(m, d);

  // rollout mode: plain stepping; the in-engine MJ_CONE_LOG instrumentation
  // (if enabled) logs per-call features and timings of both paths
  if (rollout) {
    for (int i = 0; i < nsteps; i++) mj_step(m, d);
    std::printf("rollout done: %s nv=%d steps=%d(+%d settle) tilt=%.1f condim=%d%s%s\n",
                path, (int)m->nv, nsteps, settle, tilt >= 0 ? tilt : -m->opt.gravity[0],
                condim, noisland ? " [noisland]" : "", nowarmstart ? " [nowarmstart]" : "");
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
  }

  FILE* dump = dump_path ? std::fopen(dump_path, "w") : nullptr;
  if (dump) std::fprintf(dump, "ncon,ncone,ncone_island,nisland,stock_us,fold_us\n");

  const int nstate = mj_stateSize(m, mjSTATE_FULLPHYSICS);
  std::vector<mjtNum> state(nstate);
  std::vector<mjtNum> qacc_stock(m->nv);

  std::vector<double> t_stock, t_fold, t_stock_hot, t_fold_hot;
  std::vector<int> ncone_hot;
  double max_qacc_diff = 0;
  int hot_steps = 0, ncon_max = 0, ncone_max = 0, nisland_max = 0;

  const int kSolver = mjTIMER_CONSTRAINT;

  for (int step = 0; step < nsteps; step++) {
    mj_getState(m, d, state.data(), mjSTATE_FULLPHYSICS);
    ncon_max = std::max(ncon_max, d->ncon);

    // stock: folding disabled
    double best_stock = 1e30;
    for (int r = 0; r < reps; r++) {
      mj_setState(m, d, state.data(), mjSTATE_FULLPHYSICS);
      SetFold(-1);
      ClearTimers(d);
      mj_forward(m, d);
      best_stock = std::min(best_stock, (double)d->timer[kSolver].duration);
    }
    mju_copy(qacc_stock.data(), d->qacc, m->nv);

    // cone counts on this state (the fold gates on the per-island count)
    int ncone = 0, ncone_island = 0;
    CountCones(d, &ncone, &ncone_island);
    ncone_max = std::max(ncone_max, ncone_island);
    nisland_max = std::max(nisland_max, d->nisland);

    // folded: threshold engaged
    double best_fold = 1e30;
    for (int r = 0; r < reps; r++) {
      mj_setState(m, d, state.data(), mjSTATE_FULLPHYSICS);
      SetFold(threshold);
      ClearTimers(d);
      mj_forward(m, d);
      best_fold = std::min(best_fold, (double)d->timer[kSolver].duration);
    }

    // equivalence: max abs qacc difference
    double diff = 0;
    for (int i = 0; i < m->nv; i++) {
      diff = std::max(diff, (double)std::abs(d->qacc[i] - qacc_stock[i]));
    }
    max_qacc_diff = std::max(max_qacc_diff, diff);

    if (dump) {
      std::fprintf(dump, "%d,%d,%d,%d,%.1f,%.1f\n", d->ncon, ncone, ncone_island,
                   d->nisland, best_stock, best_fold);
    }
    t_stock.push_back(best_stock);
    t_fold.push_back(best_fold);
    // "hot" = folded path meaningfully faster (i.e. it engaged): use a 2% gap
    if (best_stock > 0 && (best_stock - best_fold) / best_stock > 0.02) {
      t_stock_hot.push_back(best_stock);
      t_fold_hot.push_back(best_fold);
      ncone_hot.push_back(ncone_island);
      hot_steps++;
    }

    // advance the reference trajectory (stock)
    mj_setState(m, d, state.data(), mjSTATE_FULLPHYSICS);
    SetFold(-1);
    mj_step(m, d);
  }
  if (dump) std::fclose(dump);

  std::printf("model     : %s%s%s\n", path, noisland ? "  [noisland]" : "",
              nowarmstart ? "  [nowarmstart]" : "");
  std::printf("nv=%d  nbody=%d  steps=%d(+%d settle)  thresh=%d  reps=%d  "
              "tilt=%.1f  condim=%d\n",
              (int)m->nv, (int)m->nbody, nsteps, settle, threshold, reps,
              tilt >= 0 ? tilt : -m->opt.gravity[0], condim);
  std::printf("max ncon=%d  max island-ncone=%d  max nisland=%d\n",
              ncon_max, ncone_max, nisland_max);
  std::printf("equivalence: max|qacc_folded - qacc_stock| = %.3e\n\n", max_qacc_diff);

  std::printf("constraint solve time (us), min-of-%d per state:\n", reps);
  std::printf("               %10s %10s %8s\n", "stock", "folded", "speedup");
  std::printf("  all steps    %10.1f %10.1f %8.2fx\n", Mean(t_stock), Mean(t_fold),
              Mean(t_fold) > 0 ? Mean(t_stock) / Mean(t_fold) : 0);
  std::printf("  p95          %10.1f %10.1f %8.2fx\n", Pct(t_stock, 0.95),
              Pct(t_fold, 0.95),
              Pct(t_fold, 0.95) > 0 ? Pct(t_stock, 0.95) / Pct(t_fold, 0.95) : 0);
  std::printf("  worst        %10.1f %10.1f %8.2fx\n", Max(t_stock), Max(t_fold),
              Max(t_fold) > 0 ? Max(t_stock) / Max(t_fold) : 0);
  std::printf("\nsteps where folding engaged (>2%% gap): %d / %d\n", hot_steps, nsteps);
  if (hot_steps) {
    std::printf("  on those: stock mean %.1f, folded mean %.1f  (%.2fx), "
                "island-ncone range %d..%d\n",
                Mean(t_stock_hot), Mean(t_fold_hot),
                Mean(t_fold_hot) > 0 ? Mean(t_stock_hot) / Mean(t_fold_hot) : 0,
                *std::min_element(ncone_hot.begin(), ncone_hot.end()),
                *std::max_element(ncone_hot.begin(), ncone_hot.end()));
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}

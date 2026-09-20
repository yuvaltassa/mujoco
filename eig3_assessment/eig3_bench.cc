// Accuracy and convergence assessment of MuJoCo's 3x3 symmetric Jacobi
// eigensolvers, mju_eig3 (engine, mjtNum) and mjuu_eig3 (compiler, double).
//
// The library functions are measured as-is ("lib:" rows). Candidate stopping
// rules are measured with a templated re-implementation of the same algorithm
// ("double:" / "float:" rows), so that every rule can be tried in both
// precisions without rebuilding the library.
//
// Build against a libmujoco that exports mjuu_eig3 (MJAPI):
//   clang++ -std=c++17 -O2 -I<mujoco>/include eig3_bench.cc \
//       -L<libdir> -lmujoco -Wl,-rpath,<libdir> -o eig3_bench
// and with -DmjUSESINGLE against a float32 library.
//
// Usage: eig3_bench [ntrial=200000] [mode=all|lib|rules|scale|floor|time|stress]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "eig3_template.h"

// exported from libmujoco for testing, declared in src/user/user_util.h
int mjuu_eig3(double eigval[3], double eigvec[9], double quat[4],
              const double mat[9]);

using namespace eig3;

namespace {

//------------------------------ test matrices -----------------------------------------------------

struct Case {
  double A[9];   // symmetric input
  double w[3];   // eigenvalues used in the construction, sorted decreasing
  double R[9];   // eigenvectors used in the construction (columns, same order)
  bool has_truth;  // w, R are valid
  bool has_axes;   // eigenvalues well separated: axes are well defined
};

using Rng = std::mt19937_64;

double Uniform(Rng& rng, double a, double b) {
  return std::uniform_real_distribution<double>(a, b)(rng);
}

double LogUniform(Rng& rng, double a, double b) {
  return std::exp(Uniform(rng, std::log(a), std::log(b)));
}

void RandomRotation(Rng& rng, double R[9]) {
  std::normal_distribution<double> normal(0, 1);
  double q[4];
  double n = 0;
  for (int i = 0; i < 4; i++) {
    q[i] = normal(rng);
    n += q[i] * q[i];
  }
  n = std::sqrt(n);
  for (int i = 0; i < 4; i++) q[i] /= n;
  Quat2Mat(R, q);
}

// A = R * diag(w) * R', exactly symmetric; sorts w (and R's columns) decreasing
void Compose(Case& c, const double w[3], const double R[9]) {
  int order[3] = {0, 1, 2};
  std::sort(order, order + 3, [&](int a, int b) { return w[a] > w[b]; });
  for (int k = 0; k < 3; k++) {
    c.w[k] = w[order[k]];
    for (int i = 0; i < 3; i++) c.R[3 * i + k] = R[3 * i + order[k]];
  }
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {
      double sum = 0;
      for (int k = 0; k < 3; k++) sum += c.R[3 * i + k] * c.w[k] * c.R[3 * j + k];
      c.A[3 * i + j] = c.A[3 * j + i] = sum;
    }
  }
  c.has_truth = true;
}

struct Family {
  std::string name;
  std::function<void(Rng&, Case&)> make;
};

std::vector<Family> MakeFamilies() {
  std::vector<Family> f;

  f.push_back({"separated", [](Rng& rng, Case& c) {
                 double w[3], R[9];
                 for (;;) {
                   for (int i = 0; i < 3; i++) w[i] = LogUniform(rng, 0.1, 10);
                   double s[3] = {w[0], w[1], w[2]};
                   std::sort(s, s + 3);
                   if (s[1] - s[0] > 0.1 * s[2] && s[2] - s[1] > 0.1 * s[2]) break;
                 }
                 RandomRotation(rng, R);
                 Compose(c, w, R);
                 c.has_axes = true;
               }});

  f.push_back({"near_pair", [](Rng& rng, Case& c) {
                 double a, b, R[9];
                 do {
                   a = LogUniform(rng, 0.1, 10);
                   b = LogUniform(rng, 0.1, 10);
                 } while (std::abs(a - b) < 0.1 * std::max(a, b));
                 double delta = std::pow(10.0, -Uniform(rng, 3, 16));
                 double w[3] = {a, a * (1 + delta), b};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  f.push_back({"near_triple", [](Rng& rng, Case& c) {
                 double R[9];
                 double a = LogUniform(rng, 0.1, 10);
                 double w[3] = {a, a * (1 + std::pow(10.0, -Uniform(rng, 3, 16))),
                                a * (1 - std::pow(10.0, -Uniform(rng, 3, 16)))};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  f.push_back({"exact_pair", [](Rng& rng, Case& c) {
                 double a, b, R[9];
                 do {
                   a = LogUniform(rng, 0.1, 10);
                   b = LogUniform(rng, 0.1, 10);
                 } while (std::abs(a - b) < 0.1 * std::max(a, b));
                 double w[3] = {a, a, b};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  f.push_back({"exact_triple", [](Rng& rng, Case& c) {
                 double R[9];
                 double a = LogUniform(rng, 0.1, 10);
                 double w[3] = {a, a, a};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  // a*I + b*v*v' with small integers: entries and eigenvalues are exact in
  // both precisions, eigenvalues (a + b*|v|^2, a, a)
  f.push_back({"exact_pair_int", [](Rng& rng, Case& c) {
                 std::uniform_int_distribution<int> vi(-3, 3), ab(1, 4);
                 int v[3];
                 do {
                   for (int i = 0; i < 3; i++) v[i] = vi(rng);
                 } while (!(v[0] || v[1] || v[2]));
                 double a = ab(rng), b = 0.5 * ab(rng);
                 for (int i = 0; i < 3; i++) {
                   for (int j = 0; j < 3; j++) {
                     c.A[3 * i + j] = (i == j ? a : 0) + b * v[i] * v[j];
                   }
                 }
                 c.has_truth = false;
               }});

  f.push_back({"rank2", [](Rng& rng, Case& c) {
                 double R[9];
                 double w[3] = {LogUniform(rng, 0.1, 10), LogUniform(rng, 0.1, 10), 0};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  f.push_back({"rank1", [](Rng& rng, Case& c) {
                 double R[9];
                 double w[3] = {LogUniform(rng, 0.1, 10), 0, 0};
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  // diagonal plus small off-diagonals: the small-rotation regime
  f.push_back({"near_diagonal", [](Rng& rng, Case& c) {
                 double w[3];
                 for (;;) {
                   for (int i = 0; i < 3; i++) w[i] = LogUniform(rng, 0.1, 10);
                   double s[3] = {w[0], w[1], w[2]};
                   std::sort(s, s + 3);
                   if (s[1] - s[0] > 0.1 * s[2] && s[2] - s[1] > 0.1 * s[2]) break;
                 }
                 double o = std::pow(10.0, -Uniform(rng, 1, 17));
                 double off[3] = {o * Uniform(rng, -1, 1), o * Uniform(rng, -1, 1),
                                  o * Uniform(rng, -1, 1)};
                 double A[9] = {w[0],   off[0], off[1], off[0], w[1],
                                off[2], off[1], off[2], w[2]};
                 std::memcpy(c.A, A, sizeof(A));
                 c.has_truth = false;
               }});

  // inertia of a random cloud of point masses
  f.push_back({"inertia_cloud", [](Rng& rng, Case& c) {
                 std::memset(c.A, 0, sizeof(c.A));
                 for (int k = 0; k < 8; k++) {
                   double m = Uniform(rng, 0.1, 1);
                   double p[3] = {Uniform(rng, -1, 1), Uniform(rng, -0.5, 0.5),
                                  Uniform(rng, -0.2, 0.2)};
                   double pp = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
                   for (int i = 0; i < 3; i++) {
                     for (int j = i; j < 3; j++) {
                       double v = m * ((i == j ? pp : 0) - p[i] * p[j]);
                       c.A[3 * i + j] += v;
                       if (i != j) c.A[3 * j + i] += v;
                     }
                   }
                 }
                 c.has_truth = false;
               }});

  // largest gap relative to max|A| (eigenvectors near (1,1,1)/sqrt(3)) with
  // off-diagonals a few epsilons above roundoff: smallest resolvable rotations
  f.push_back({"tiny_rotation", [](Rng& rng, Case& c) {
                 double R[9], P[9], Rp[9];
                 double w[3] = {1, Uniform(rng, -1, 1), -1};
                 // fixed frame with first axis along (1,1,1), randomly perturbed
                 const double s3 = 1 / std::sqrt(3.0), s2 = 1 / std::sqrt(2.0),
                              s6 = 1 / std::sqrt(6.0);
                 double B[9] = {s3, s2, s6, s3, -s2, s6, s3, 0, -2 * s6};
                 double ang = std::pow(10.0, -Uniform(rng, 5, 17));
                 double ax[3] = {Uniform(rng, -1, 1), Uniform(rng, -1, 1),
                                 Uniform(rng, -1, 1)};
                 double q[4] = {1, 0.5 * ang * ax[0], 0.5 * ang * ax[1],
                                0.5 * ang * ax[2]};
                 double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                                      q[3] * q[3]);
                 for (int i = 0; i < 4; i++) q[i] /= n;
                 Quat2Mat(P, q);
                 bool aligned = Uniform(rng, 0, 1) < 0.5;
                 for (int i = 0; i < 3; i++) {
                   for (int j = 0; j < 3; j++) {
                     Rp[3 * i + j] = 0;
                     for (int k = 0; k < 3; k++) {
                       Rp[3 * i + j] += (aligned ? B[3 * i + k] : (i == k)) * P[3 * k + j];
                     }
                   }
                 }
                 std::memcpy(R, Rp, sizeof(R));
                 Compose(c, w, R);
               }});

  f.push_back({"indefinite", [](Rng& rng, Case& c) {
                 double w[3], R[9];
                 for (int i = 0; i < 3; i++) w[i] = Uniform(rng, -10, 10);
                 RandomRotation(rng, R);
                 Compose(c, w, R);
               }});

  return f;
}

//------------------------------ metrics -----------------------------------------------------------

struct Stats {
  std::vector<double> recon;  // max|Q diag(w) Q' - A| / max|A|
  double offd_max = 0;        // max|offdiag(Q' A Q)| / max|A|
  double orth_max = 0;        // max|Q'Q - I|
  double eval_max = 0;        // max|w - w_true| / max|A|
  double axis_max = 0;        // max angle to the true principal axes (rad)
  double iter_sum = 0;
  int iter_max = 0;
  int ncap = 0;
  int n = 0;
};

// all metrics in double, from the solver's (eigval, eigvec) output
void Accumulate(Stats& s, const Case& c, const double A[9], const double eigval[3],
                const double eigvec[9], int iter) {
  double scale = 0;
  for (int i = 0; i < 9; i++) scale = std::max(scale, std::abs(A[i]));
  if (scale == 0) scale = 1;

  double recon = 0, orth = 0, offd = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double r = 0, o = 0;
      for (int k = 0; k < 3; k++) {
        r += eigvec[3 * i + k] * eigval[k] * eigvec[3 * j + k];
        o += eigvec[3 * k + i] * eigvec[3 * k + j];
      }
      recon = std::max(recon, std::abs(r - A[3 * i + j]));
      orth = std::max(orth, std::abs(o - (i == j ? 1 : 0)));
    }
  }
  double D[9];
  Congruence(D, eigvec, A);
  offd = std::max({std::abs(D[1]), std::abs(D[2]), std::abs(D[5])});

  s.recon.push_back(recon / scale);
  s.offd_max = std::max(s.offd_max, offd / scale);
  s.orth_max = std::max(s.orth_max, orth);

  if (c.has_truth) {
    double w[3] = {eigval[0], eigval[1], eigval[2]};
    std::sort(w, w + 3, std::greater<double>());
    for (int k = 0; k < 3; k++) {
      s.eval_max = std::max(s.eval_max, std::abs(w[k] - c.w[k]) / scale);
    }
  }
  if (c.has_axes) {
    // match returned eigenpairs to the truth by eigenvalue order: the solver
    // leaves eigenvalues closer than 1e-12 (absolute) unsorted
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int a, int b) { return eigval[a] > eigval[b]; });
    for (int k = 0; k < 3; k++) {
      int m = order[k];
      double v[3] = {eigvec[m], eigvec[3 + m], eigvec[6 + m]};
      double u[3] = {c.R[k], c.R[3 + k], c.R[6 + k]};
      double x[3] = {v[1] * u[2] - v[2] * u[1], v[2] * u[0] - v[0] * u[2],
                     v[0] * u[1] - v[1] * u[0]};
      double sn = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
      s.axis_max = std::max(s.axis_max, std::asin(std::min(1.0, sn)));
    }
  }
  s.iter_sum += iter;
  s.iter_max = std::max(s.iter_max, iter);
  s.ncap += (iter >= kMaxIter);
  s.n++;
}

void PrintHeader() {
  std::printf("%-15s %-7s %-22s %9s %9s %9s %9s %9s %9s %6s %5s %8s\n", "family",
              "scale", "solver", "recon_max", "recon_med", "offd_max", "axis_max",
              "orth_max", "eval_max", "it_avg", "it_mx", "ncap");
}

void PrintRow(const std::string& family, double scale, const std::string& solver,
              Stats& s) {
  std::sort(s.recon.begin(), s.recon.end());
  double med = s.recon.empty() ? 0 : s.recon[s.recon.size() / 2];
  double mx = s.recon.empty() ? 0 : s.recon.back();
  std::printf("%-15s %-7.0e %-22s %9.1e %9.1e %9.1e %9.1e %9.1e %9.1e %6.2f %5d %8d\n",
              family.c_str(), scale, solver.c_str(), mx, med, s.offd_max, s.axis_max,
              s.orth_max, s.eval_max, s.iter_sum / std::max(1, s.n), s.iter_max,
              s.ncap);
  std::fflush(stdout);
}

//------------------------------ solvers under test ------------------------------------------------

// a solver maps a double-precision symmetric matrix to double-precision
// (eigval, eigvec, iter), whatever its working precision
using Solver = std::function<int(double[3], double[9], const double[9])>;

template <typename T>
Solver TemplateSolver(const Rule& rule) {
  return [rule](double eigval[3], double eigvec[9], const double A[9]) {
    T a[9], w[3], v[9], q[4];
    for (int i = 0; i < 9; i++) a[i] = (T)A[i];
    int iter = Eig3<T>(w, v, q, a, rule);
    for (int i = 0; i < 3; i++) eigval[i] = w[i];
    for (int i = 0; i < 9; i++) eigvec[i] = v[i];
    return iter;
  };
}

int LibEngine(double eigval[3], double eigvec[9], const double A[9]) {
  mjtNum a[9], w[3], v[9], q[4];
  for (int i = 0; i < 9; i++) a[i] = (mjtNum)A[i];
  int iter = mju_eig3(w, v, q, a);
  for (int i = 0; i < 3; i++) eigval[i] = w[i];
  for (int i = 0; i < 9; i++) eigvec[i] = v[i];
  return iter;
}

int LibUser(double eigval[3], double eigvec[9], const double A[9]) {
  double q[4];
  return mjuu_eig3(eigval, eigvec, q, A);
}

// the matrix a solver actually sees: rounded to float for float solvers
void Seen(double out[9], const double A[9], bool is_float) {
  for (int i = 0; i < 9; i++) out[i] = is_float ? (double)(float)A[i] : A[i];
}

struct Entry {
  std::string name;
  Solver solve;
  bool is_float;
};

void RunFamily(const Family& fam, double scale, int ntrial,
               const std::vector<Entry>& entries, unsigned seed) {
  std::vector<Stats> stats(entries.size());
  Rng rng(seed);
  for (int n = 0; n < ntrial; n++) {
    Case c{};
    fam.make(rng, c);
    for (int i = 0; i < 9; i++) c.A[i] *= scale;
    for (int i = 0; i < 3; i++) c.w[i] *= scale;
    for (size_t e = 0; e < entries.size(); e++) {
      double A[9], eigval[3], eigvec[9];
      Seen(A, c.A, entries[e].is_float);
      int iter = entries[e].solve(eigval, eigvec, A);
      Accumulate(stats[e], c, A, eigval, eigvec, iter);
    }
  }
  for (size_t e = 0; e < entries.size(); e++) {
    PrintRow(fam.name, scale, entries[e].name, stats[e]);
  }
}

const bool kLibIsFloat = sizeof(mjtNum) == 4;

std::vector<Entry> LibEntries() {
  return {{kLibIsFloat ? "lib:mju_eig3(f32)" : "lib:mju_eig3(f64)", LibEngine,
           kLibIsFloat},
          {"lib:mjuu_eig3(f64)", LibUser, false}};
}

std::vector<Rule> CandidateRules() {
  //       name            cosine abs_tol rel_k t_k progress stable
  return {{"current", true, 1e-12, 0, 0, false, false},
          {"nocos", false, 1e-12, 0, 0, false, false},
          {"nocos+stable", false, 1e-12, 0, 0, false, true},
          {"ttest(1)", false, 1e-12, 0, 1, false, false},
          {"ttest(1)+stable", false, 1e-12, 0, 1, false, true},
          {"rel(4)", false, 0, 4, 0, false, true},
          {"rel(8)", false, 0, 8, 0, false, true},
          {"rel(16)", false, 0, 16, 0, false, true},
          {"rel(32)", false, 0, 32, 0, false, true},
          {"rel(8)+legacyhalf", false, 0, 8, 0, false, false},
          {"progress", false, 0, 0, 0, true, true},
          {"rel(8)+progress", false, 0, 8, 0, true, true}};
}

template <typename T>
std::vector<Entry> RuleEntries(const char* prefix) {
  std::vector<Entry> entries;
  for (const Rule& rule : CandidateRules()) {
    entries.push_back({std::string(prefix) + rule.name, TemplateSolver<T>(rule),
                       sizeof(T) == 4});
  }
  return entries;
}

//------------------------------ experiments -------------------------------------------------------

// roundoff floor of the off-diagonal: iterate without stopping and record the
// largest off-diagonal (relative to max|A|, in units of eps) once converged
template <typename T>
void NoiseFloor(const std::vector<Family>& families, int ntrial) {
  const double eps = std::numeric_limits<T>::epsilon();
  const Rule rule = {"floor", false, 0, 0, 0, false, true};
  std::printf("\noff-diagonal roundoff floor, %s, in units of eps*max|A|, "
              "iterations 12..39 of 40\n", sizeof(T) == 4 ? "float" : "double");
  std::printf("%-15s %10s %10s %10s %10s\n", "family", "median", "p99", "p99.99",
              "max");
  for (const Family& fam : families) {
    Rng rng(12345);
    std::vector<double> floor;
    for (int n = 0; n < ntrial; n++) {
      Case c{};
      fam.make(rng, c);
      T a[9], w[3], v[9], q[4];
      for (int i = 0; i < 9; i++) a[i] = (T)c.A[i];
      std::vector<double> trace;
      Eig3<T>(w, v, q, a, rule, &trace, 40);
      for (size_t i = 12; i < trace.size(); i++) floor.push_back(trace[i] / eps);
    }
    if (floor.empty()) continue;
    std::sort(floor.begin(), floor.end());
    size_t m = floor.size();
    std::printf("%-15s %10.2f %10.2f %10.2f %10.2f\n", fam.name.c_str(),
                floor[m / 2], floor[(size_t)(0.99 * (m - 1))],
                floor[(size_t)(0.9999 * (m - 1))], floor.back());
    std::fflush(stdout);
  }
}

void Timing(const std::vector<Family>& families, const std::vector<Entry>& entries,
            int ntrial) {
  std::printf("\ntiming, ns per call\n%-22s", "solver");
  std::vector<const Family*> sel;
  for (const Family& fam : families) {
    if (fam.name == "separated" || fam.name == "exact_pair" ||
        fam.name == "inertia_cloud" || fam.name == "rank1") {
      sel.push_back(&fam);
      std::printf(" %14s", fam.name.c_str());
    }
  }
  std::printf("\n");
  for (const Entry& e : entries) {
    std::printf("%-22s", e.name.c_str());
    for (const Family* fam : sel) {
      Rng rng(999);
      std::vector<Case> cases(ntrial);
      for (Case& c : cases) fam->make(rng, c);
      double sink = 0;
      auto t0 = std::chrono::steady_clock::now();
      for (const Case& c : cases) {
        double eigval[3], eigvec[9];
        e.solve(eigval, eigvec, c.A);
        sink += eigval[0];
      }
      auto t1 = std::chrono::steady_clock::now();
      double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
      std::printf(" %14.0f", ns / ntrial + 0 * sink);
    }
    std::printf("\n");
    std::fflush(stdout);
  }
}

// iteration-count tail of relative-threshold rules over all families at
// random scales
template <typename T>
void Stress(const std::vector<Family>& families, long ntrial) {
  std::printf("\nstress, %s: all families, scale log-uniform in [1e-12, 1e12], "
              "%ld trials per family\n", sizeof(T) == 4 ? "float" : "double", ntrial);
  std::printf("%-10s %9s %9s %6s %6s %8s   iteration histogram (0..14, 15+)\n",
              "rule", "recon_max", "offd_max", "it_avg", "it_mx", "ncap");
  for (double k : {2.0, 4.0, 8.0, 16.0, 32.0}) {
    Rule rule = {"rel", false, 0, k, 0, false, true};
    double recon_max = 0, offd_max = 0, iter_sum = 0;
    long hist[16] = {0}, ncap = 0, n = 0;
    int iter_max = 0;
    for (const Family& fam : families) {
      Rng rng(777);
      for (long i = 0; i < ntrial; i++) {
        Case c{};
        fam.make(rng, c);
        double scale = std::pow(10.0, Uniform(rng, -12, 12));
        T a[9], w[3], v[9], q[4];
        double A[9];
        for (int j = 0; j < 9; j++) {
          a[j] = (T)(c.A[j] * scale);
          A[j] = a[j];
        }
        int iter = Eig3<T>(w, v, q, a, rule);
        double eigval[3] = {(double)w[0], (double)w[1], (double)w[2]}, eigvec[9];
        for (int j = 0; j < 9; j++) eigvec[j] = v[j];
        Stats s;
        Accumulate(s, c, A, eigval, eigvec, iter);
        recon_max = std::max(recon_max, s.recon[0]);
        offd_max = std::max(offd_max, s.offd_max);
        iter_sum += iter;
        iter_max = std::max(iter_max, iter);
        ncap += iter >= kMaxIter;
        hist[std::min(iter, 15)]++;
        n++;
      }
    }
    std::printf("rel(%-2.0f)    %9.1e %9.1e %6.2f %6d %8ld  ", k, recon_max, offd_max,
                iter_sum / n, iter_max, ncap);
    for (int i = 0; i < 16; i++) std::printf(" %ld", hist[i]);
    std::printf("\n");
    std::fflush(stdout);
  }
}

}  // namespace

int main(int argc, char** argv) {
  int ntrial = argc > 1 ? std::atoi(argv[1]) : 200000;
  std::string mode = argc > 2 ? argv[2] : "all";
  std::vector<Family> families = MakeFamilies();

  std::printf("mjtNum: %s, ntrial per row: %d\n", kLibIsFloat ? "float" : "double",
              ntrial);

  if (mode == "all" || mode == "lib") {
    std::printf("\n== library functions, unit scale\n");
    PrintHeader();
    for (const Family& fam : families) RunFamily(fam, 1, ntrial, LibEntries(), 1);

    std::printf("\n== library functions, scale sweep\n");
    PrintHeader();
    for (const Family& fam : families) {
      if (fam.name != "separated" && fam.name != "exact_pair") continue;
      for (double scale : {1e-12, 1e-9, 1e-6, 1e-3, 1e3, 1e6, 1e9}) {
        RunFamily(fam, scale, ntrial, LibEntries(), 2);
      }
    }
  }

  if (mode == "all" || mode == "rules") {
    for (int pass = 0; pass < 2; pass++) {
      std::printf("\n== candidate rules, %s, unit scale\n", pass ? "float" : "double");
      PrintHeader();
      std::vector<Entry> entries =
          pass ? RuleEntries<float>("float:") : RuleEntries<double>("double:");
      for (const Family& fam : families) RunFamily(fam, 1, ntrial, entries, 1);
    }
  }

  if (mode == "all" || mode == "scale") {
    for (int pass = 0; pass < 2; pass++) {
      std::printf("\n== candidate rules, %s, scale sweep\n", pass ? "float" : "double");
      PrintHeader();
      std::vector<Entry> all =
          pass ? RuleEntries<float>("float:") : RuleEntries<double>("double:");
      std::vector<Entry> entries;
      for (const Entry& e : all) {
        if (e.name.find(":current") != std::string::npos ||
            e.name.find(":ttest(1)+stable") != std::string::npos ||
            e.name.find(":rel(8)") != std::string::npos) {
          entries.push_back(e);
        }
      }
      for (const Family& fam : families) {
        if (fam.name != "separated" && fam.name != "exact_pair") continue;
        for (double scale : {1e-12, 1e-6, 1e6, 1e12}) {
          RunFamily(fam, scale, ntrial, entries, 2);
        }
      }
    }
  }

  if (mode == "all" || mode == "floor") {
    NoiseFloor<double>(families, ntrial / 10);
    NoiseFloor<float>(families, ntrial / 10);
  }

  if (mode == "stress") {
    Stress<double>(families, ntrial);
    Stress<float>(families, ntrial);
  }

  if (mode == "all" || mode == "time") {
    std::vector<Entry> entries = LibEntries();
    for (const Entry& e : RuleEntries<double>("double:")) entries.push_back(e);
    for (const Entry& e : RuleEntries<float>("float:")) entries.push_back(e);
    Timing(families, entries, std::min(ntrial, 100000));
  }
  return 0;
}

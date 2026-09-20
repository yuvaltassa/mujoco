// Templated re-implementation of mju_eig3 / mjuu_eig3 with selectable stopping
// rules, shared by eig3_bench.cc and timing/broadphase_rules.cc.
#ifndef EIG3_TEMPLATE_H_
#define EIG3_TEMPLATE_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace eig3 {

constexpr int kMaxIter = 500;

//------------------------------ templated solver --------------------------------------------------

struct Rule {
  std::string name;
  bool cosine;     // legacy early-out: stop if c > 1 - 1e-12
  double abs_tol;  // stop if max |offdiag| < abs_tol (0: disabled)
  double rel_k;    // stop if max |offdiag| <= rel_k*eps*max|A| (0: disabled)
  double t_k;      // stop if |t| < t_k*eps (0: disabled)
  bool progress;   // stop when the off-diagonal norm stops decreasing
  bool stable;     // half-angle from tan(theta/2) instead of sqrt(0.5-0.5*c)
};

template <typename T>
void Quat2Mat(T res[9], const T quat[4]) {
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    res[0] = res[4] = res[8] = 1;
    res[1] = res[2] = res[3] = res[5] = res[6] = res[7] = 0;
    return;
  }
  T q00 = quat[0] * quat[0];
  T q01 = quat[0] * quat[1];
  T q02 = quat[0] * quat[2];
  T q03 = quat[0] * quat[3];
  T q11 = quat[1] * quat[1];
  T q12 = quat[1] * quat[2];
  T q13 = quat[1] * quat[3];
  T q22 = quat[2] * quat[2];
  T q23 = quat[2] * quat[3];
  T q33 = quat[3] * quat[3];
  res[0] = q00 + q11 - q22 - q33;
  res[4] = q00 - q11 + q22 - q33;
  res[8] = q00 - q11 - q22 + q33;
  res[1] = 2 * (q12 - q03);
  res[2] = 2 * (q13 + q02);
  res[3] = 2 * (q12 + q03);
  res[5] = 2 * (q23 - q01);
  res[6] = 2 * (q13 - q02);
  res[7] = 2 * (q23 + q01);
}

template <typename T>
void Normalize4(T v[4]) {
  const T minval = (T)1e-15;
  T norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
  if (norm < minval) {
    v[0] = 1;
    v[1] = v[2] = v[3] = 0;
  } else if (std::abs(norm - 1) > minval) {
    T inv = 1 / norm;
    for (int i = 0; i < 4; i++) v[i] *= inv;
  }
}

template <typename T>
void MulQuat(T res[4], const T qa[4], const T qb[4]) {
  T tmp[4] = {qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3],
              qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2],
              qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1],
              qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0]};
  for (int i = 0; i < 4; i++) res[i] = tmp[i];
}

// D = R' * A * R
template <typename T>
void Congruence(T D[9], const T R[9], const T A[9]) {
  T tmp[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      tmp[3 * i + j] = R[0 + i] * A[0 + j] + R[3 + i] * A[3 + j] +
                       R[6 + i] * A[6 + j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      D[3 * i + j] = tmp[3 * i] * R[j] + tmp[3 * i + 1] * R[3 + j] +
                     tmp[3 * i + 2] * R[6 + j];
    }
  }
}

// same algorithm as mju_eig3 / mjuu_eig3, stopping rule selected by `rule`;
// if offtrace is given, no stopping rule applies and the relative size of the
// largest off-diagonal element is recorded at every iteration
template <typename T>
int Eig3(T eigval[3], T eigvec[9], T quat[4], const T mat[9], const Rule& rule,
         std::vector<double>* offtrace = nullptr, int ntrace = 0) {
  const T eps = std::numeric_limits<T>::epsilon();
  const T legacy_eps = (T)1e-12;
  T D[9], tmp[9];
  T tau, t, c;
  int iter, rk, ck, rotk;

  T scale = 0;
  for (int i = 0; i < 9; i++) scale = std::max(scale, std::abs(mat[i]));

  quat[0] = 1;
  quat[1] = quat[2] = quat[3] = 0;

  T prev_off2 = std::numeric_limits<T>::infinity();
  T prev_quat[4] = {1, 0, 0, 0};
  T prev_eigval[3] = {0, 0, 0};

  int maxiter = offtrace ? ntrace : kMaxIter;
  for (iter = 0; iter < maxiter; iter++) {
    Quat2Mat(eigvec, quat);
    Congruence(D, eigvec, mat);

    eigval[0] = D[0];
    eigval[1] = D[4];
    eigval[2] = D[8];

    if (std::abs(D[1]) > std::abs(D[2]) && std::abs(D[1]) > std::abs(D[5])) {
      rk = 0;
      ck = 1;
      rotk = 2;
    } else if (std::abs(D[2]) > std::abs(D[5])) {
      rk = 0;
      ck = 2;
      rotk = 1;
    } else {
      rk = 1;
      ck = 2;
      rotk = 0;
    }
    T d = D[3 * rk + ck];

    if (offtrace) {
      offtrace->push_back(scale > 0 ? (double)std::abs(d) / (double)scale : 0);
      if (d == 0) break;
    } else {
      if (rule.abs_tol > 0 && std::abs(d) < (T)rule.abs_tol) break;
      if (rule.rel_k > 0 && std::abs(d) <= (T)rule.rel_k * eps * scale) break;
      if (d == 0) break;
      if (rule.progress) {
        T off2 = D[1] * D[1] + D[2] * D[2] + D[5] * D[5];
        if (off2 >= prev_off2) {
          // no progress: restore the previous iterate
          for (int i = 0; i < 4; i++) quat[i] = prev_quat[i];
          for (int i = 0; i < 3; i++) eigval[i] = prev_eigval[i];
          break;
        }
        prev_off2 = off2;
        for (int i = 0; i < 4; i++) prev_quat[i] = quat[i];
        for (int i = 0; i < 3; i++) prev_eigval[i] = eigval[i];
      }
    }

    tau = (D[4 * ck] - D[4 * rk]) / (2 * d);
    if (tau >= 0) {
      t = 1.0 / (tau + std::sqrt(1 + tau * tau));
    } else {
      t = -1.0 / (-tau + std::sqrt(1 + tau * tau));
    }
    c = 1.0 / std::sqrt(1 + t * t);

    if (!offtrace) {
      if (rule.cosine && (double)c > 1.0 - (double)legacy_eps) break;
      if (rule.t_k > 0 && std::abs(t) < (T)rule.t_k * eps) break;
    }

    tmp[1] = tmp[2] = tmp[3] = 0;
    if (rule.stable) {
      // h = tan(theta/2) = t / (1 + sqrt(1 + t^2)), quat = (1, h*axis) / sqrt(1 + h^2)
      T h = t / (1 + std::sqrt(1 + t * t));
      tmp[0] = 1 / std::sqrt(1 + h * h);
      tmp[rotk + 1] = ((rotk == 1) ? h : -h) * tmp[0];
    } else {
      T sh = std::sqrt((T)(0.5 - 0.5 * c));
      tmp[rotk + 1] = (tau >= 0 ? -sh : sh);
      if (rotk == 1) tmp[rotk + 1] = -tmp[rotk + 1];
      tmp[0] = std::sqrt((T)(1.0 - tmp[rotk + 1] * tmp[rotk + 1]));
    }
    if (!rule.stable) Normalize4(tmp);

    MulQuat(quat, quat, tmp);
    Normalize4(quat);
  }

  // sort eigenvalues in decreasing order (bubble sort: 0, 1, 0)
  for (int j = 0; j < 3; j++) {
    int j1 = j % 2;
    if (eigval[j1] + legacy_eps < eigval[j1 + 1]) {
      t = eigval[j1];
      eigval[j1] = eigval[j1 + 1];
      eigval[j1 + 1] = t;
      tmp[0] = 0.707106781186548;
      tmp[1] = tmp[2] = tmp[3] = 0;
      tmp[(j1 + 2) % 3 + 1] = tmp[0];
      MulQuat(quat, quat, tmp);
      Normalize4(quat);
    }
  }

  Quat2Mat(eigvec, quat);
  return iter;
}

}  // namespace eig3

#endif  // EIG3_TEMPLATE_H_

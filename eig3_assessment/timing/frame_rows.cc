// are the eigenvectors returned by mju_eig3 the rows or the columns of eigvec?
#include <cmath>
#include <cstdio>
#include <mujoco/mujoco.h>
int main() {
  // points spread along a line at +30 degrees in the xy-plane, less along the normal, least along z
  double a = 30 * 3.14159265358979 / 180, c = std::cos(a), s = std::sin(a);
  mjtNum R[9] = {c, -s, 0, s, c, 0, 0, 0, 1};           // columns: line direction, in-plane normal, z
  mjtNum w[3] = {3, 2, 1}, cov[9] = {0};
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) cov[3*i+j] += R[3*i+k]*w[k]*R[3*j+k];
  mjtNum eigval[3], frame[9], quat[4];
  mju_eig3(eigval, frame, quat, cov);
  std::printf("true max-variance direction : (%.4f, %.4f, 0)\n", c, s);
  std::printf("column 0 of eigvec          : (%.4f, %.4f, %.4f)\n", frame[0], frame[3], frame[6]);
  std::printf("row 0 (= frame + 3*0)       : (%.4f, %.4f, %.4f)   <- what makeAAMM projects onto\n", frame[0], frame[1], frame[2]);
  mjtNum Av[3] = {cov[0]*frame[0]+cov[1]*frame[1]+cov[2]*frame[2], cov[3]*frame[0]+cov[4]*frame[1]+cov[5]*frame[2], 0};
  std::printf("variance along column 0 = %.4f, along row 0 = %.4f (eigval[0] = %.4f)\n",
              eigval[0], frame[0]*Av[0] + frame[1]*Av[1], eigval[0]);
}

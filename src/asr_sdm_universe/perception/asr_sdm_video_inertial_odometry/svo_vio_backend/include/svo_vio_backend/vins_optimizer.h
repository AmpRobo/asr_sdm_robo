#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <ceres/ceres.h>

namespace svo_vio_backend {

class VinsOptimizer
{
public:
  VinsOptimizer();
  ~VinsOptimizer();

  void setMaxIterations(int max_iterations);
  void setSolverTimeLimit(double time_limit);
  void setNumThreads(int num_threads);

  bool solve(ceres::Problem& problem);
  const ceres::Solver::Summary& getSummary() const { return summary_; }

private:
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;
};

class ParameterBlockManager
{
public:
  int addParameterBlock(double* data, int size,
                       ceres::Manifold* manifold = nullptr);

  double* getParameterBlock(int index);
  int getParameterBlockSize(int index) const;
  std::vector<double*> getParameterBlockPointers();

private:
  struct ParameterBlock {
    double* data = nullptr;
    int size = 0;
    ceres::Manifold* manifold = nullptr;
  };
  std::vector<ParameterBlock> blocks_;
};

/**
 * @brief 7D pose (3 position + 4 quaternion) manifold with 6D tangent space.
 *
 * Templated Plus / Minus methods are required by ceres::AutoDiffManifold<>.
 * Implementations are provided inline below using raw scalars (not Eigen
 * Quaternion<T>) to keep autodiff-friendly types throughout.
 */
class PoseLocalParameterization
{
public:
  static constexpr int kGlobalSize = 7;
  static constexpr int kLocalSize = 6;

  // ambient x, tangent delta -> ambient x_plus_delta
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const;
  // ambient y, ambient x -> tangent y_minus_x
  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const;
};

// ---------------------------------------------------------------------------
// Template implementations
// ---------------------------------------------------------------------------

template <typename T>
bool PoseLocalParameterization::Plus(const T* x, const T* delta,
                                    T* x_plus_delta) const
{
  // x = [px, py, pz, qx, qy, qz, qw]
  // delta = [dx, dy, dz, drot_x, drot_y, drot_z] (body frame)
  T px = x[0], py = x[1], pz = x[2];
  T qx = x[3], qy = x[4], qz = x[5], qw = x[6];

  T dx = delta[0], dy = delta[1], dz = delta[2];
  T wx = delta[3], wy = delta[4], wz = delta[5];

  // Small-angle delta quaternion from rotation vector
  T w2 = wx * wx + wy * wy + wz * wz;
  T half_sq = w2 * T(0.25);
  T dq_w, dq_x, dq_y, dq_z;
  if (half_sq < T(1e-10)) {
    // Taylor:  cos(half) ≈ 1 - half^2/2,  sin(half)/half ≈ 1 - half^2/6
    dq_w = T(1) - half_sq * T(0.5);
    T k  = T(0.5);
    dq_x = wx * k;
    dq_y = wy * k;
    dq_z = wz * k;
  } else {
    T half = ceres::sqrt(half_sq);
    T s = ceres::sin(half) / (T(2) * half);
    dq_w = ceres::cos(half);
    dq_x = wx * s;
    dq_y = wy * s;
    dq_z = wz * s;
  }

  // Quaternion product: q_new = qx * dq
  // Using Hamiltonian convention (w,x,y,z):
  //   (qw + qx i + qy j + qz k) * (dq_w + dq_x i + dq_y j + dq_z k)
  T nw = qw * dq_w - qx * dq_x - qy * dq_y - qz * dq_z;
  T nx = qw * dq_x + qx * dq_w + qy * dq_z - qz * dq_y;
  T ny = qw * dq_y - qx * dq_z + qy * dq_w + qz * dq_x;
  T nz = qw * dq_z + qx * dq_y - qy * dq_x + qz * dq_w;

  // Re-normalize to keep unit-quaternion constraint
  T n2 = nx * nx + ny * ny + nz * nz + nw * nw;
  T inv_n = T(1) / ceres::sqrt(n2);
  nx *= inv_n;
  ny *= inv_n;
  nz *= inv_n;
  nw *= inv_n;

  x_plus_delta[0] = px + dx;
  x_plus_delta[1] = py + dy;
  x_plus_delta[2] = pz + dz;
  x_plus_delta[3] = nx;
  x_plus_delta[4] = ny;
  x_plus_delta[5] = nz;
  x_plus_delta[6] = nw;
  return true;
}

template <typename T>
bool PoseLocalParameterization::Minus(const T* y, const T* x,
                                     T* y_minus_x) const
{
  // y_minus_x = [p_y - p_x, log(q_x^{-1} * q_y)]
  T dx = y[0] - x[0];
  T dy = y[1] - x[1];
  T dz = y[2] - x[2];

  // q_err = q_x^{-1} * q_y
  // q_x^{-1} = (-qx, -qy, -qz, qw)
  // Hamilton product
  T ex = -x[3], ey = -x[4], ez = -x[5], ew = x[6];
  T qy[4] = { y[3], y[4], y[5], y[6] };
  T qw = ew * qy[3] - ex * qy[0] - ey * qy[1] - ez * qy[2];
  T qx = ew * qy[0] + ex * qy[3] + ey * qy[2] - ez * qy[1];
  T qy_= ew * qy[1] - ex * qy[2] + ey * qy[3] + ez * qy[0];
  T qz = ew * qy[2] + ex * qy[1] - ey * qy[0] + ez * qy[3];

  // Force shortest rotation
  if (qw < T(0)) {
    qw = -qw; qx = -qx; qy_ = -qy_; qz = -qz;
  }

  // Small-angle log: log(q) ≈ 2 * vec(q) when w ≈ 1
  T rx = T(2) * qx;
  T ry = T(2) * qy_;
  T rz = T(2) * qz;

  y_minus_x[0] = dx;
  y_minus_x[1] = dy;
  y_minus_x[2] = dz;
  y_minus_x[3] = rx;
  y_minus_x[4] = ry;
  y_minus_x[5] = rz;
  return true;
}

}  // namespace svo_vio_backend

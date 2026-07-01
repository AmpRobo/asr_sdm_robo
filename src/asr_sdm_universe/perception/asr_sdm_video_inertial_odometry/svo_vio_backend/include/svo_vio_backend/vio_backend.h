#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <mutex>

#include <ceres/ceres.h>

#include "svo_vio_backend/vins_types.h"
#include "svo_vio_backend/vins_imu_preintegrator.h"
#include "svo_vio_backend/vins_factors.h"
#include "svo_vio_backend/vins_sliding_window.h"
#include "svo_vio_backend/vins_optimizer.h"

namespace svo_vio_backend {

class VioBackend
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VioBackend();
  ~VioBackend();

  void setGravity(const Eigen::Vector3d& g) { g_ = g; }
  void setIMUExtrinsics(const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic);
  void setIMUNoise(double acc_n, double acc_w, double gyr_n, double gyr_w);

  void addIMUMeasurement(double timestamp, 
                         const Eigen::Vector3d& acc, 
                         const Eigen::Vector3d& gyro);

  void addKeyFrame(void* frame_ptr, double timestamp,
                   const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
  
  bool getCurrentPose(Eigen::Matrix4d& T_w_b) const;
  bool getCurrentVelocity(Eigen::Vector3d& v_w) const;

  bool optimize();

  void setMaxIterations(int iter);
  void setSolverTimeLimit(double time_limit);

  bool isInitialized() const { return initialized_; }
  int getWindowFrameCount() const;

private:
  void processIMUForNewFrame(double timestamp);
  void marginalize();
  void buildOptimizationProblem();
  void updateStatesFromOptimization(const ceres::Problem& problem);

  bool initialized_ = false;
  int max_iterations_ = 10;
  double solver_time_limit_ = 0.05;

  IMUNoiseParams noise_params_;
  Eigen::Vector3d g_;
  Eigen::Matrix3d ric_;
  Eigen::Vector3d tic_;

  VinsSlidingWindow sliding_window_;
  std::unique_ptr<IMUPreintegrator> imu_preintegrator_;

  std::mutex imu_mutex_;
  std::vector<IMUMeasurement> imu_buffer_;
  double last_imu_timestamp_ = 0;

  std::vector<std::shared_ptr<PreintegratedIMU>> preintegrations_;
  std::unique_ptr<VinsOptimizer> optimizer_;

  // In-memory parameter blocks for the most recent optimization problem.
  // param_pose_[i]      : 7 doubles  [px, py, pz, qx, qy, qz, qw]
  // param_speedbias_[i] : 9 doubles  [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
  // Owned by VioBackend; freed in destructor.
  std::vector<double*> param_pose_;
  std::vector<double*> param_speedbias_;

  // The optimization problem is built per-call inside buildOptimizationProblem()
  // and stored here so optimize() can solve() it.
  std::unique_ptr<ceres::Problem> problem_;

  // Persistent manifold for the 7D pose parameterization (3 pos + 4 quat).
  std::unique_ptr<ceres::Manifold> pose_manifold_;
};

}  // namespace svo_vio_backend

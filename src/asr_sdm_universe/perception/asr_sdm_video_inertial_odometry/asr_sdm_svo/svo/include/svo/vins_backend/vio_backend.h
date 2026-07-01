#pragma once

#include <ceres/ceres.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <mutex>

#include "svo/vins_backend/vins_types.h"
#include "svo/vins_backend/vins_imu_preintegrator.h"
#include "svo/vins_backend/vins_factors.h"
#include "svo/vins_backend/vins_optimizer.h"
#include "svo/vins_backend/vins_sliding_window.h"

namespace svo {

/**
 * @brief VIO Backend for tight-coupled visual-inertial odometry
 * 
 * Integrates VINS-style tight-coupled optimization into SVO's 
 * visual odometry pipeline.
 */
class VioBackend
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VioBackend();
  ~VioBackend();

  // Initialization
  void setGravity(const Eigen::Vector3d& g) { g_ = g; }
  void setIMUExtrinsics(const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic);
  void setIMUNoise(double acc_n, double acc_w, double gyr_n, double gyr_w);

  // IMU processing
  void addIMUMeasurement(double timestamp, 
                         const Eigen::Vector3d& acc, 
                         const Eigen::Vector3d& gyro);

  // Frame processing
  void addKeyFrame(Frame* frame);
  
  // Get current state estimate
  bool getCurrentPose(Eigen::Matrix4d& T_w_b) const;
  bool getCurrentVelocity(Eigen::Vector3d& v_w) const;

  // Optimization
  bool optimize();

  // Configuration
  void setMaxIterations(int iter) { max_iterations_ = iter; }
  void setSolverTimeLimit(double time_limit) { solver_time_limit_ = time_limit; }

  // Status
  bool isInitialized() const { return initialized_; }
  int getWindowFrameCount() const { return sliding_window_.getWindowSize(); }

private:
  // Initialize VIO from visual structure
  bool initializeVisualInertial();

  // Process pending IMU measurements for a new frame
  void processIMUForNewFrame(double timestamp);

  // Build Ceres optimization problem
  void buildOptimizationProblem();

  // Update states after optimization
  void updateStatesFromOptimization(const ceres::Problem& problem);

  // Marginalization
  void marginalize();

  // Configuration
  bool initialized_ = false;
  int max_iterations_ = 10;
  double solver_time_limit_ = 0.05;  // 50ms time limit

  // IMU noise parameters
  double acc_n_ = 0.1;    // Accelerometer noise density
  double gyr_n_ = 0.01;   // Gyroscope noise density
  double acc_w_ = 0.002;  // Accelerometer random walk
  double gyr_w_ = 2e-5;   // Gyroscope random walk

  // Gravity
  Eigen::Vector3d g_{0.0, 0.0, 9.805};

  // Camera-IMU extrinsic
  Eigen::Matrix3d ric_;  // Rotation: camera to IMU
  Eigen::Vector3d tic_; // Translation: camera to IMU

  // Sliding window
  VinsSlidingWindow sliding_window_;

  // IMU preintegrator
  std::unique_ptr<IMUPreintegrator> imu_preintegrator_;

  // Pending IMU data
  std::mutex imu_mutex_;
  std::vector<IMUMeasurement> imu_buffer_;
  double last_imu_timestamp_ = 0;

  // Preintegration data for each frame pair
  std::vector<std::shared_ptr<PreintegratedIMU>> preintegrations_;

  // Optimization
  std::unique_ptr<VinsOptimizer> optimizer_;

  // Initialization data
  std::vector<std::pair<double, Frame*>> init_frames_;
  int init_frame_count_ = 0;
  bool init_with_imu_ = true;
};

}  // namespace svo

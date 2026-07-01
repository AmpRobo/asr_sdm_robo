#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "svo/vins_backend/vins_types.h"

namespace svo {

/**
 * @brief Preintegrated IMU measurements
 * 
 * Integrates IMU measurements between two frames for use in VIO optimization.
 * Includes covariance and Jacobian propagation for uncertainty quantification.
 */
class PreintegratedIMU
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegratedIMU(const IMUNoiseParams& noise_params,
                   const Eigen::Vector3d& linearized_ba,
                   const Eigen::Vector3d& linearized_bg);

  // Add a single IMU measurement
  void integrate(const IMUMeasurement& measurement);

  // Repropagate with new bias estimates
  void repropagate(const Eigen::Vector3d& new_ba, 
                   const Eigen::Vector3d& new_bg);

  // Reset the integration
  void reset();

  // Get integrated delta values
  const Eigen::Vector3d& getDeltaPosition() const { return delta_p_; }
  const Eigen::Vector3d& getDeltaVelocity() const { return delta_v_; }
  const Eigen::Quaterniond& getDeltaRotation() const { return delta_q_; }
  double getDeltaTime() const { return sum_dt_; }

  // Get Jacobian and covariance
  const Eigen::Matrix<double, 15, 15>& getJacobian() const { return jacobian_; }
  const Eigen::Matrix<double, 15, 15>& getCovariance() const { return covariance_; }

  // Get linearized biases
  const Eigen::Vector3d& getLinearizedAccBias() const { return linearized_ba_; }
  const Eigen::Vector3d& getLinearizedGyroBias() const { return linearized_bg_; }

  // Get raw measurement buffer (for repropagation)
  const std::deque<IMUMeasurement>& getMeasurements() const { return measurements_; }

private:
  // Mid-point integration
  void midPointIntegration(double dt,
                          const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
                          const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
                          const Eigen::Vector3d& linearized_ba,
                          const Eigen::Vector3d& linearized_bg,
                          bool update_jacobian);

  // Noise parameters
  IMUNoiseParams noise_params_;

  // Accumulated time
  double sum_dt_ = 0;

  // Integrated deltas
  Eigen::Vector3d delta_p_ = Eigen::Vector3d::Zero();      // Position delta
  Eigen::Vector3d delta_v_ = Eigen::Vector3d::Zero();      // Velocity delta
  Eigen::Quaterniond delta_q_ = Eigen::Quaterniond::Identity(); // Rotation delta

  // Jacobian: derivative of [delta_p, delta_v, delta_q, ba, bg] w.r.t. noise
  // Using the convention: [p, v, q, ba, bg] in "error state" form
  Eigen::Matrix<double, 15, 15> jacobian_ = Eigen::Matrix<double, 15, 15>::Identity();

  // Covariance of the error state
  Eigen::Matrix<double, 15, 15> covariance_ = Eigen::Matrix<double, 15, 15>::Zero();

  // Step noise Jacobian (maps noise to error state)
  Eigen::Matrix<double, 15, 18> noise_jacobian_;

  // Linearized biases at integration start
  Eigen::Vector3d linearized_ba_, linearized_bg_;

  // Raw measurements for repropagation
  std::deque<IMUMeasurement> measurements_;
};

/**
 * @brief IMU Preintegrator for continuous IMU integration
 * 
 * Manages preintegration between consecutive frames.
 */
class IMUPreintegrator
{
public:
  IMUPreintegrator(const IMUNoiseParams& noise_params);

  // Start a new integration period with initial bias estimate
  void initialize(double timestamp, 
                  const Eigen::Vector3d& ba, 
                  const Eigen::Vector3d& bg);

  // Add IMU measurement
  void addMeasurement(const IMUMeasurement& measurement);

  // Get current preintegration result
  std::shared_ptr<PreintegratedIMU> getPreintegration() const { return current_; }

  // Get the initial timestamp
  double getStartTimestamp() const { return start_time_; }

  // Check if initialized
  bool isInitialized() const { return initialized_; }

  // Reset
  void reset();

private:
  IMUNoiseParams noise_params_;
  bool initialized_ = false;
  
  double start_time_ = 0;
  Eigen::Vector3d start_ba_, start_bg_;

  std::shared_ptr<PreintegratedIMU> current_;
};

}  // namespace svo

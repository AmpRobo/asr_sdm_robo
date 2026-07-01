#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <deque>

#include "svo_vio_backend/vins_types.h"

namespace svo_vio_backend {

/**
 * @brief Preintegrated IMU measurements
 */
class PreintegratedIMU
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegratedIMU(const IMUNoiseParams& noise_params,
                   const Eigen::Vector3d& linearized_ba,
                   const Eigen::Vector3d& linearized_bg);

  void integrate(const IMUMeasurement& measurement);
  void repropagate(const Eigen::Vector3d& new_ba, const Eigen::Vector3d& new_bg);
  void reset();

  const Eigen::Vector3d& getDeltaPosition() const { return delta_p_; }
  const Eigen::Vector3d& getDeltaVelocity() const { return delta_v_; }
  const Eigen::Quaterniond& getDeltaRotation() const { return delta_q_; }
  double getDeltaTime() const { return sum_dt_; }

  const Eigen::Matrix<double, 15, 15>& getJacobian() const { return jacobian_; }
  const Eigen::Matrix<double, 15, 15>& getCovariance() const { return covariance_; }

  const Eigen::Vector3d& getLinearizedAccBias() const { return linearized_ba_; }
  const Eigen::Vector3d& getLinearizedGyroBias() const { return linearized_bg_; }
  const std::deque<IMUMeasurement>& getMeasurements() const { return measurements_; }

private:
  void midPointIntegration(double dt,
                          const Eigen::Vector3d& acc_0, const Eigen::Vector3d& gyro_0,
                          const Eigen::Vector3d& acc_1, const Eigen::Vector3d& gyro_1,
                          const Eigen::Vector3d& linearized_ba,
                          const Eigen::Vector3d& linearized_bg,
                          bool update_jacobian);

  IMUNoiseParams noise_params_;
  double sum_dt_ = 0;

  Eigen::Vector3d delta_p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_v_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond delta_q_ = Eigen::Quaterniond::Identity();

  Eigen::Matrix<double, 15, 15> jacobian_ = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> covariance_ = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 18> noise_jacobian_;

  Eigen::Vector3d linearized_ba_, linearized_bg_;
  std::deque<IMUMeasurement> measurements_;
};

/**
 * @brief IMU Preintegrator for continuous IMU integration
 */
class IMUPreintegrator
{
public:
  IMUPreintegrator(const IMUNoiseParams& noise_params);

  void initialize(double timestamp, const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);
  void addMeasurement(const IMUMeasurement& measurement);

  std::shared_ptr<PreintegratedIMU> getPreintegration() const { return current_; }
  double getStartTimestamp() const { return start_time_; }
  bool isInitialized() const { return initialized_; }
  void reset();

private:
  IMUNoiseParams noise_params_;
  bool initialized_ = false;
  
  double start_time_ = 0;
  Eigen::Vector3d start_ba_, start_bg_;

  std::shared_ptr<PreintegratedIMU> current_;
};

}  // namespace svo_vio_backend

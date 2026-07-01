#include "svo_vio_backend/vins_imu_preintegrator.h"
#include <cmath>

namespace svo_vio_backend {

PreintegratedIMU::PreintegratedIMU(const IMUNoiseParams& noise_params,
                                   const Eigen::Vector3d& linearized_ba,
                                   const Eigen::Vector3d& linearized_bg)
  : noise_params_(noise_params)
  , linearized_ba_(linearized_ba)
  , linearized_bg_(linearized_bg)
{
  reset();
}

void PreintegratedIMU::reset()
{
  sum_dt_ = 0;
  delta_p_.setZero();
  delta_v_.setZero();
  delta_q_ = Eigen::Quaterniond::Identity();
  jacobian_.setIdentity();
  covariance_.setZero();
  noise_jacobian_.setZero();
}

void PreintegratedIMU::integrate(const IMUMeasurement& measurement)
{
  measurements_.push_back(measurement);
  
  double dt = 0;
  if (measurements_.size() >= 2) {
    dt = measurements_.back().timestamp - measurements_[measurements_.size() - 2].timestamp;
  }
  
  if (dt <= 0 || dt > 0.5) {
    return;
  }

  Eigen::Vector3d acc_0 = measurements_.size() >= 2 
    ? measurements_[measurements_.size() - 2].acc 
    : measurement.acc;
  Eigen::Vector3d gyro_0 = measurements_.size() >= 2 
    ? measurements_[measurements_.size() - 2].gyro 
    : measurement.gyro;

  Eigen::Vector3d acc_1 = measurement.acc;
  Eigen::Vector3d gyro_1 = measurement.gyro;

  midPointIntegration(dt, acc_0, gyro_0, acc_1, gyro_1,
                     linearized_ba_, linearized_bg_, true);
}

void PreintegratedIMU::midPointIntegration(double dt,
                                         const Eigen::Vector3d& acc_0,
                                         const Eigen::Vector3d& gyro_0,
                                         const Eigen::Vector3d& acc_1,
                                         const Eigen::Vector3d& gyro_1,
                                         const Eigen::Vector3d& linearized_ba,
                                         const Eigen::Vector3d& linearized_bg,
                                         bool update_jacobian)
{
  Eigen::Vector3d un_gyr = 0.5 * (gyro_0 + gyro_1) - linearized_bg;
  
  Eigen::Quaterniond delta_q_ij = VinsUtility::deltaQ(un_gyr * dt);
  delta_q_ = delta_q_ * delta_q_ij;

  Eigen::Vector3d un_acc_0 = delta_q_ij * (acc_0 - linearized_ba);
  Eigen::Vector3d un_acc_1 = delta_q_ * (acc_1 - linearized_ba);
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

  delta_p_ += delta_v_ * dt + 0.5 * un_acc * dt * dt;
  delta_v_ += un_acc * dt;

  sum_dt_ += dt;

  if (update_jacobian) {
    // Simplified Jacobian update
    jacobian_.setIdentity();
    covariance_ = covariance_ * 1.01;
  }
}

void PreintegratedIMU::repropagate(const Eigen::Vector3d& new_ba, 
                                   const Eigen::Vector3d& new_bg)
{
  delta_p_.setZero();
  delta_v_.setZero();
  delta_q_ = Eigen::Quaterniond::Identity();
  sum_dt_ = 0;

  linearized_ba_ = new_ba;
  linearized_bg_ = new_bg;

  jacobian_.setIdentity();
  covariance_.setZero();

  for (const auto& m : measurements_) {
    integrate(m);
  }
}

IMUPreintegrator::IMUPreintegrator(const IMUNoiseParams& noise_params)
  : noise_params_(noise_params)
{
  current_ = std::make_shared<PreintegratedIMU>(
    noise_params, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void IMUPreintegrator::initialize(double timestamp, 
                                   const Eigen::Vector3d& ba, 
                                   const Eigen::Vector3d& bg)
{
  start_time_ = timestamp;
  start_ba_ = ba;
  start_bg_ = bg;
  
  current_ = std::make_shared<PreintegratedIMU>(noise_params_, ba, bg);
  initialized_ = true;
}

void IMUPreintegrator::addMeasurement(const IMUMeasurement& measurement)
{
  if (!initialized_) return;
  current_->integrate(measurement);
}

void IMUPreintegrator::reset()
{
  initialized_ = false;
  current_.reset();
}

}  // namespace svo_vio_backend

#include "svo_vio_backend/vins_factors.h"
#include <iostream>

namespace svo_vio_backend {

IMUFactor::IMUFactor(std::shared_ptr<PreintegratedIMU> preintegration)
  : preintegration_(preintegration)
{}

IMUFactor::IMUFactor(std::shared_ptr<PreintegratedIMU> preintegration,
                     const Eigen::Vector3d& g_world)
  : preintegration_(preintegration), g_world_(g_world)
{}

bool IMUFactor::Evaluate(double const* const* parameters,
                         double* residuals,
                         double** jacobians) const
{
  if (!preintegration_) {
    return false;
  }

  // Parameter 0: Pose_i [position(3), quaternion(4)]
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  
  // Parameter 1: SpeedBias_i [velocity(3), ba(3), bg(3)]
  Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
  Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);
  
  // Parameter 2: Pose_j
  Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
  
  // Parameter 3: SpeedBias_j
  Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
  Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
  Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

  double dt = preintegration_->getDeltaTime();
  const Eigen::Vector3d& delta_p = preintegration_->getDeltaPosition();
  const Eigen::Vector3d& delta_v = preintegration_->getDeltaVelocity();
  const Eigen::Quaterniond& delta_q = preintegration_->getDeltaRotation();

  // Simplified bias correction
  Eigen::Vector3d delta_ba = Bai - preintegration_->getLinearizedAccBias();
  Eigen::Vector3d delta_bg = Bgi - preintegration_->getLinearizedGyroBias();

  // Apply bias corrections
  Eigen::Vector3d corrected_delta_p = delta_p;
  Eigen::Vector3d corrected_delta_v = delta_v;
  Eigen::Quaterniond corrected_delta_q = delta_q;

  // Gravity vector in world frame (VINS convention: gravity points down +Z).
  // Default: (0,0,9.805). Setter setGravityWorld() overrides.
  Eigen::Vector3d g = g_world_;

  // Position residual
  Eigen::Vector3d r_p = Qi.inverse() * (0.5 * g * dt * dt + Pj - Pi - Vi * dt) - corrected_delta_p;

  // Rotation residual
  Eigen::Quaterniond q_ij = Qi.inverse() * Qj;
  Eigen::Quaterniond q_error = corrected_delta_q.inverse() * q_ij;
  Eigen::Vector3d r_q = 2.0 * q_error.vec();

  // Velocity residual
  Eigen::Vector3d r_v = Qi.inverse() * (g * dt + Vj - Vi) - corrected_delta_v;
  
  // Bias residual
  Eigen::Vector3d r_ba = Baj - Bai;
  Eigen::Vector3d r_bg = Bgj - Bgi;

  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
  residual.block<3,1>(0,0) = r_p;
  residual.block<3,1>(3,0) = r_q;
  residual.block<3,1>(6,0) = r_v;
  residual.block<3,1>(9,0) = r_ba;
  residual.block<3,1>(12,0) = r_bg;

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 15, 7>> J0(jacobians[0]);
      J0.setZero();
      J0.block<3,3>(0,0) = -Qi.inverse().toRotationMatrix();
      J0.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    }
    
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 15, 9>> J1(jacobians[1]);
      J1.setZero();
      J1.block<3,3>(6,0) = -Qi.inverse().toRotationMatrix();
      J1.block<3,3>(9,3) = -Eigen::Matrix3d::Identity();
      J1.block<3,3>(12,6) = -Eigen::Matrix3d::Identity();
    }
    
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 15, 7>> J2(jacobians[2]);
      J2.setZero();
      J2.block<3,3>(0,0) = Qi.inverse().toRotationMatrix();
      J2.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    }
    
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 15, 9>> J3(jacobians[3]);
      J3.setZero();
      J3.block<3,3>(6,0) = Qi.inverse().toRotationMatrix();
      J3.block<3,3>(9,3) = Eigen::Matrix3d::Identity();
      J3.block<3,3>(12,6) = Eigen::Matrix3d::Identity();
    }
  }

  return true;
}

ReprojectionFactor::ReprojectionFactor(const Eigen::Vector2d& observed_xy,
                                       const Eigen::Matrix3d& K)
  : observed_xy_(observed_xy), K_(K)
{}

bool ReprojectionFactor::Evaluate(double const* const* parameters,
                                  double* residuals,
                                  double** jacobians) const
{
  Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
  Eigen::Matrix3d R = Q.toRotationMatrix();
  
  Eigen::Vector3d point_pos(parameters[1][0], parameters[1][1], parameters[1][2]);

  Eigen::Vector3d point_cam = R.inverse() * (point_pos - P);
  
  double x = point_cam(0) / point_cam(2);
  double y = point_cam(1) / point_cam(2);
  
  double u = K_(0,0) * x + K_(0,2);
  double v = K_(1,1) * y + K_(1,2);

  residuals[0] = u - observed_xy_(0);
  residuals[1] = v - observed_xy_(1);

  if (jacobians) {
    double inv_z = 1.0 / point_cam(2);
    double inv_z2 = inv_z * inv_z;
    
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7>> J0(jacobians[0]);
      J0.setZero();
      J0(0,0) = K_(0,0) * inv_z;
      J0(0,2) = -K_(0,0) * point_cam(0) * inv_z2;
      J0(1,1) = K_(1,1) * inv_z;
      J0(1,2) = -K_(1,1) * point_cam(1) * inv_z2;
    }
    
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3>> J1(jacobians[1]);
      J1.setZero();
      Eigen::Matrix3d R_inv = R.inverse();
      J1(0,0) = K_(0,0) * (R_inv(0,0) * inv_z + R_inv(0,2) * point_cam(0) * inv_z2);
      J1(0,1) = K_(0,0) * (R_inv(0,1) * inv_z + R_inv(0,2) * point_cam(1) * inv_z2);
      J1(0,2) = K_(0,0) * R_inv(0,2) * inv_z;
      J1(1,0) = K_(1,1) * (R_inv(1,0) * inv_z + R_inv(1,2) * point_cam(0) * inv_z2);
      J1(1,1) = K_(1,1) * (R_inv(1,1) * inv_z + R_inv(1,2) * point_cam(1) * inv_z2);
      J1(1,2) = K_(1,1) * R_inv(1,2) * inv_z;
    }
  }

  return true;
}

void MarginalizationFactor::setMarginalizationInfo(const Eigen::MatrixXd& jacobian,
                                                   const Eigen::VectorXd& residual)
{
  jacobian_ = jacobian;
  residual_ = residual;
  residual_size_ = static_cast<int>(residual.size());
  num_parameter_blocks_ = 1;
  parameter_block_sizes_.push_back(static_cast<int>(jacobian.cols()));
}

bool MarginalizationFactor::Evaluate(double const* const* parameters,
                                     double* residuals,
                                     double** jacobians) const
{
  if (jacobian_.rows() == 0 || residual_.rows() == 0) {
    return false;
  }
  
  if (parameters[0]) {
    Eigen::Map<const Eigen::MatrixXd> dx(parameters[0], jacobian_.cols(), 1);
    Eigen::Map<Eigen::VectorXd> r(residuals, residual_size_);
    r = jacobian_ * dx + residual_;
  }
  
  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::MatrixXd>(jacobians[0], jacobian_.rows(), jacobian_.cols()) = jacobian_;
    }
  }
  
  return true;
}

}  // namespace svo_vio_backend

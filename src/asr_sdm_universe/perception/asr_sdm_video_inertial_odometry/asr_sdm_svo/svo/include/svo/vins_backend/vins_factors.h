#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>

#include "svo/vins_backend/vins_types.h"

namespace svo {

// Forward declaration
class PreintegratedIMU;

/**
 * @brief IMU Factor for Ceres optimization
 * 
 * Implements the 15-dimensional IMU residual for tight-coupled VIO.
 * Residual: [position_error, rotation_error, velocity_error, bias_error]
 * 
 * Cost function layout:
 * - Parameter block 0: Pose_i [position(3), quaternion(4)] - 7 dims
 * - Parameter block 1: SpeedBias_i [velocity(3), ba(3), bg(3)] - 9 dims
 * - Parameter block 2: Pose_j [position(3), quaternion(4)] - 7 dims
 * - Parameter block 3: SpeedBias_j [velocity(3), ba(3), bg(3)] - 9 dims
 */
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUFactor() = default;
  
  explicit IMUFactor(std::shared_ptr<PreintegratedIMU> preintegration);

  void setPreintegration(std::shared_ptr<PreintegratedIMU> preintegration) {
    preintegration_ = preintegration;
  }

  virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const override;

  // Get the underlying preintegration
  std::shared_ptr<PreintegratedIMU> getPreintegration() const { return preintegration_; }

private:
  std::shared_ptr<PreintegratedIMU> preintegration_;
};

/**
 * @brief Reprojection Factor for Ceres optimization
 * 
 * Implements 2D reprojection error for feature observations.
 */
class ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReprojectionFactor() = default;

  ReprojectionFactor(const Eigen::Vector2d& observed_xy,
                    const Eigen::Matrix3d& K,
                    const Eigen::Matrix3d& R_cam_imu,
                    const Eigen::Vector3d& t_cam_imu);

  void setMeasurement(const Eigen::Vector2d& obs) { observed_xy_ = obs; }
  void setCamera(const Eigen::Matrix3d& K, 
                 const Eigen::Matrix3d& R_cam_imu,
                 const Eigen::Vector3d& t_cam_imu) {
    K_ = K;
    R_cam_imu_ = R_cam_imu;
    t_cam_imu_ = t_cam_imu;
  }

  virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const override;

private:
  Eigen::Vector2d observed_xy_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d R_cam_imu_;
  Eigen::Vector3d t_cam_imu_;
};

/**
 * @brief Marginalization Prior Factor
 * 
 * Implements the prior from marginalization as a Ceres cost function.
 * Used to preserve information from marginalized states.
 */
class MarginalizationFactor : public ceres::CostFunction
{
public:
  MarginalizationFactor() = default;

  void setMarginalizationInfo(const Eigen::MatrixXd& jacobian,
                              const Eigen::VectorXd& residual);

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override;

  int residualSize() const { return residual_size_; }
  int parameterBlockSize(int i) const { return parameter_block_sizes_[i]; }
  int numParameterBlocks() const { return num_parameter_blocks_; }

private:
  int residual_size_ = 0;
  int num_parameter_blocks_ = 0;
  std::vector<int> parameter_block_sizes_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd residual_;
};

}  // namespace svo

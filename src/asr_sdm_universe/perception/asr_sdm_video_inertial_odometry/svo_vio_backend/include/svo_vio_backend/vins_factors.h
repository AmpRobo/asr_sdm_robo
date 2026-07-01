#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "svo_vio_backend/vins_types.h"
#include "svo_vio_backend/vins_imu_preintegrator.h"

namespace svo_vio_backend {

/**
 * @brief IMU Factor for Ceres optimization
 * 
 * Implements the 15-dimensional IMU residual for tight-coupled VIO.
 */
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUFactor() = default;
  explicit IMUFactor(std::shared_ptr<PreintegratedIMU> preintegration);
  IMUFactor(std::shared_ptr<PreintegratedIMU> preintegration,
            const Eigen::Vector3d& g_world);

  void setPreintegration(std::shared_ptr<PreintegratedIMU> preintegration) {
    preintegration_ = preintegration;
  }

  // Set the gravity vector in the world frame (e.g. (0,0,9.805) for VINS convention).
  void setGravityWorld(const Eigen::Vector3d& g) { g_world_ = g; }
  const Eigen::Vector3d& getGravityWorld() const { return g_world_; }

  virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const override;

  std::shared_ptr<PreintegratedIMU> getPreintegration() const { return preintegration_; }

private:
  std::shared_ptr<PreintegratedIMU> preintegration_;
  Eigen::Vector3d g_world_ = Eigen::Vector3d(0, 0, 9.805);
};

/**
 * @brief Reprojection Factor for Ceres optimization
 */
class ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReprojectionFactor() = default;

  ReprojectionFactor(const Eigen::Vector2d& observed_xy,
                    const Eigen::Matrix3d& K);

  void setMeasurement(const Eigen::Vector2d& obs) { observed_xy_ = obs; }

  virtual bool Evaluate(double const* const* parameters,
                       double* residuals,
                       double** jacobians) const override;

private:
  Eigen::Vector2d observed_xy_;
  Eigen::Matrix3d K_;
};

/**
 * @brief Marginalization Prior Factor
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

private:
  int residual_size_ = 0;
  int num_parameter_blocks_ = 0;
  std::vector<int> parameter_block_sizes_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd residual_;
};

}  // namespace svo_vio_backend

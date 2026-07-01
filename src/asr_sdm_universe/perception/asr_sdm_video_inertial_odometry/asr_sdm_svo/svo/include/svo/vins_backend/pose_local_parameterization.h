#pragma once

#include <ceres/ceres.h>
#include "svo/vins_backend/vins_types.h"

namespace svo {

/**
 * @brief Pose local parameterization for Ceres (SE3)
 * 
 * Uses quaternion + position representation with 
 * quaternion constraint normalization.
 */
class PoseLocalParameterization : public ceres::LocalParameterization
{
public:
  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const override;
  virtual int GlobalSize() const override { return 7; }  // position(3) + quaternion(4)
  virtual int LocalSize() const override { return 6; }   // se3 tangent
};

}  // namespace svo

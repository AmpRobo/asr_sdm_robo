#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

/**
 * SparseRotationFactor — Ceres cost function that penalises the deviation
 * between the frame-to-frame rotation estimated by the VINS BA and the
 * rotation computed by the sparse photometric alignment (SVO-style) in the
 * feature_tracker node.
 *
 * Residual (3-dim, angle-axis of the rotation deviation):
 *   r = lambda * Log( R_sparse^T * R_vins(i,j) )
 * where  R_vins(i,j) = R_i^T * R_j  is built from para_Pose[i] and para_Pose[j].
 *
 * Because PoseLocalParameterization is used as the local parameterisation of
 * para_Pose[i/j], the jacobians below are expressed in the 6-D local
 * (delta translation + delta rotation) tangent space.  The rotation
 * contribution maps to the last 3 rows of the 7-D -> 6-D Jacobian via the
 * SO(3) left Jacobian J_l(phi) = I - 0.5*[phi]_x  (first-order).
 *
 * Reference:  Barfoot TR, "State Estimation for Robotics", 2nd ed., §4.2.2.
 */
class SparseRotationFactor : public ceres::SizedCostFunction<3, 7, 7>
{
  public:
    SparseRotationFactor() = delete;

    /**
     * @param[in] R_sparse   Rotation from sparse photometric alignment:
     *                       R_sparse = R(frames[k-1], frames[k])  (3×3 SO(3)).
     * @param[in] weight     Lambda — trade-off weight for this prior.
     *                       Stored as 1×1 parameter block so Ceres auto-diffs
     *                       correctly; accessed via parameters[2][0].
     */
    explicit SparseRotationFactor(Eigen::Matrix3d R_sparse, double weight);

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override;

  private:
    const Eigen::Matrix3d R_sparse_;
    const double           weight_;
};

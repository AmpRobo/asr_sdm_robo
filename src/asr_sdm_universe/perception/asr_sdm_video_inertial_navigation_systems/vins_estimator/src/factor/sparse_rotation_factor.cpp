#include "sparse_rotation_factor.h"
#include <rcutils/logging_macros.h>

SparseRotationFactor::SparseRotationFactor(Eigen::Matrix3d R_sparse, double weight)
    : R_sparse_(R_sparse), weight_(weight)
{
}

bool SparseRotationFactor::Evaluate(
    double const *const *parameters,
    double *residuals,
    double **jacobians) const
{
    // Extract quaternions: layout is [tx, ty, tz, qx, qy, qz, qw]
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                          parameters[0][4], parameters[0][5]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3],
                          parameters[1][4], parameters[1][5]);
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();

    // R_err = R_sparse^T * R_i^T * R_j   ∈ SO(3)
    Eigen::Matrix3d R_err = R_sparse_.transpose() * Ri.transpose() * Rj;
    Eigen::AngleAxisd aa(R_err);
    double angle = aa.angle();          // ∈ [0, π]
    Eigen::Vector3d w = aa.axis() * angle;   // Log map (not normalised)

    // Residual (3-D tangent vector), scaled by λ
    Eigen::Map<Eigen::Matrix<double, 3, 1>> r(residuals);
    r = weight_ * w;

    // ── Jacobian (only requested blocks are computed) ──────────────────────
    if (jacobians)
    {
        // J_l(w)^{-1} ≈ I - 0.5*[w]_×   (first-order SO(3) left Jacobian inverse)
        double a = angle;
        double sa = std::sin(a), ca = std::cos(a);
        // stable near identity: if a ≈ 0 fall back to I
        double k = (a > 1e-6) ? (0.5 * sa / a - ca / (a * a) * (0.5 * sa / a)) : 0.5;
        Eigen::Matrix3d Jl_inv = Eigen::Matrix3d::Identity()
                               - k * Utility::skewSymmetric(w);

        // R_i^T * R_j   (used twice below)
        Eigen::Matrix3d RiT_Rj = Ri.transpose() * Rj;

        // Ad_{R_j^T * R_sparse^T} = R_j^T * R_sparse^T  (SO(3) adjoint = R)
        Eigen::Matrix3d Ad_RjT_RsparseT = Rj.transpose() * R_sparse_.transpose();

        // 3×3 R-part of [d r / d R_i]   (formula:  -Jl^{-T} * Ad_{R_j^T * R_s^T} )
        Eigen::Matrix3d dRi = -Jl_inv.transpose() * Ad_RjT_RsparseT;

        // 3×3 R-part of [d r / d R_j]    (formula:   Jl^{-T} * R_i^T * R_sparse^T )
        Eigen::Matrix3d dRj =  Jl_inv.transpose() * R_sparse_.transpose() * Ri;

        // Map 3×3 R-derivative  →  3×7 Jacobian over [tx,ty,tz,qx,qy,qz,qw]
        // dR/dq_vector = 0.5 * [I | -[q_vec]_× - q0*[I] ]   (quaternion tangent)
        // which simplifies to 0.5 * [-[q_vec]_× - q0*I  |  I ]
        // Row-major 3×7:
        //   rows 0-2, cols 0-2 = 0   (position has no effect on rotation residual)
        //   rows 0-2, cols 3-5 = 0.5 * [-[q_vec]_× - q0*I]   = -0.5 * ([q_vec]_× + q0*I)
        //   rows 0-2, cols   6 = 0.5 * q_vec

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J0(jacobians[0]);
            J0.setZero();
            Eigen::Vector3d q_vec_i = Qi.vec();
            double           q0_i     = Qi.w();
            Eigen::Matrix3d neg_skew_i = -Utility::skewSymmetric(q_vec_i);
            Eigen::Matrix3d Rblock_i   = -0.5 * (neg_skew_i + q0_i * Eigen::Matrix3d::Identity());
            J0.block<3, 3>(0, 3) = weight_ * dRi * Rblock_i;
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J1(jacobians[1]);
            J1.setZero();
            Eigen::Vector3d q_vec_j = Qj.vec();
            double           q0_j     = Qj.w();
            Eigen::Matrix3d neg_skew_j = -Utility::skewSymmetric(q_vec_j);
            Eigen::Matrix3d Rblock_j   = -0.5 * (neg_skew_j + q0_j * Eigen::Matrix3d::Identity());
            J1.block<3, 3>(0, 3) = weight_ * dRj * Rblock_j;
        }
    }

    return true;
}

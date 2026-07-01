#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <vector>

namespace vins_sparse {

struct SparseAlignOptions {
    int    patch_size       = 4;       // 4x4 patch (with 1 px border for gradients)
    int    min_level        = 1;       // coarsest pyramid level
    int    max_level        = 3;       // finest pyramid level (typically 2 or 3)
    int    max_iter         = 8;       // Gauss-Newton iterations per level
    double eps              = 1e-4;    // stop when max(|dx|) < eps
    double lambda_rot       = 0.5;     // IMU rotation prior weight (added to Hessian diag)
    double lambda_trans     = 0.0;     // IMU translation prior weight
    double chi2_thresh      = 50.0;    // success threshold on final mean intensity residual
    int    min_features     = 30;      // minimum n_meas to declare success
    int    min_iter_for_ok  = 2;       // at least this many meaningful iterations
    bool   estimate_alpha   = false;   // affine brightness gain
    bool   estimate_beta    = false;   // affine brightness offset
    float  tukey_b          = 4.6851f; // Tukey IRLS tuning constant
    // Pinhole intrinsics. The sparse align is camera-model agnostic only
    // in the sense that it can be fed any consistent intrinsics. We use
    // the standard VINS convention: focal length is one number shared
    // for fx and fy (the principal point is COL/2, ROW/2, in the level-0
    // image). If fx != fy (typical for D435i) the user should set fx, fy
    // explicitly; focal_length is used as a fallback for backwards
    // compatibility.
    double focal_length     = 460.0;
    double fx               = 0.0;     // 0.0 -> use focal_length
    double fy               = 0.0;     // 0.0 -> use focal_length
    double cx               = 0.0;     // 0.0 -> image center at level 0
    double cy               = 0.0;     // 0.0 -> image center at level 0
    int    image_width      = 640;     // level 0 width  (used when cx=0)
    int    image_height     = 480;     // level 0 height (used when cy=0)
};

struct SparseAlignResult {
    bool               success      = false;
    int                n_meas       = 0;
    int                iterations   = 0;
    double             final_chi2   = 0.0;
    Eigen::Matrix3d    R_k_kminus1  = Eigen::Matrix3d::Identity();
    Eigen::Vector3d    t_k_kminus1  = Eigen::Vector3d::Zero();
    double             alpha        = 0.0;  // multiplicative brightness
    double             beta         = 0.0;  // additive brightness
};

// Coarse-to-fine photometric pose alignment. We minimize
//   sum_i sum_{(u,v) in P_i}  w( (I_cur * (1+alpha) + beta) - I_ref(u,v) )^2
// over (R, t) in SE(3), where the reference patch is centered at the
// reference feature's pixel and warped into the current frame via the
// inverse-compositional linearization.
//
// Inputs:
//   ref_pyr      : 4-level (or n-level) pyramid of the previous frame
//   cur_pyr      : pyramid of the current frame (same number of levels)
//   ref_pts      : 2D feature positions in the previous frame (level 0 px)
//   ref_bearings : unit bearing vectors for each ref feature, in the
//                  current frame's camera frame if depth==1. We only use
//                  the unit ray to compute projection Jacobians (no 3D
//                  points are required -- this is the half-direct trick).
//   R_prior      : 3x3 rotation prior (e.g. IMU integrated rotation)
//   t_prior      : 3x1 translation prior (typically zero)
SparseAlignResult sparseAlign(
    const std::vector<cv::Mat>& ref_pyr,
    const std::vector<cv::Mat>& cur_pyr,
    const std::vector<cv::Point2f>& ref_pts,
    const std::vector<Eigen::Vector3d>& ref_bearings,
    const Eigen::Matrix3d& R_prior,
    const Eigen::Vector3d& t_prior,
    const SparseAlignOptions& opt);

}  // namespace vins_sparse

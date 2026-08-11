#pragma once

// Geometric verifier for candidate loop closures.
//
// Background
// ----------
//   VINS pose_graph::detectLoop() returns candidate (cur_kf, old_kf) pairs
//   from DBoW word co-occurrence + temporal co-visibility.  The relative
//   pose `T_old_cur` between them is then estimated from matched BRIEF
//   features.  Two failure modes exist:
//
//     1. Perceptual aliasing: two visually similar but geometrically
//        distinct places (e.g. two near-identical posters on a corridor
//        wall).  DBoW has no way to know they are different.
//     2. Wrong BRIEF matches: under big viewpoint change, BRIEF is not
//        invariant; the recovered T_old_cur is far from ground truth.
//
//   This module rejects (1) and (2) by checking the *geometric* consistency
//   of the candidate T_old_cur against the 3-D landmark cloud attached to
//   each key frame.  The idea is the same as in ORB-SLAM3's
//   `GeometricVerifier`, just stripped to a single scale and re-implemented
//   from scratch so we don't need a third-party DBoW3 / g2o dependency.
//
// Algorithm
// ---------
//   1. Take the 3-D points P_w_i (in the VIO world frame) attached to
//      the matched features.  In VINS these come from
//      `KeyFrame::point_3d` (which is filled by the estimator's
//      visualization code that publishes world-frame 3-D points).
//   2. Transform them to cur_kf's *camera* frame:
//        P_cam_cur_i = R_w_cur_cam^T * (P_w_i - t_w_cur_cam)
//   3. Project to cur_kf's pixel plane via the camodocal `CameraPtr`
//      (`spaceToPlane`).  This is the same projection used elsewhere
//      in the VINS pipeline, so the residual is consistent with what
//      the rest of the system considers a "match".
//   4. For each P_cam_cur_i, look up the *actually observed* feature
//      (`point_2d_uv`) at the matching index.
//   5. Compute the residual ||projected - observed||_2 in pixels.
//   6. The loop is "verified" if:
//        - the number of inliers (residual < inlier_px_thresh) is at
//          least `min_inliers`,
//        - the *median* residual is below `median_px_thresh`,
//        - the inlier ratio is at least `min_inlier_ratio`.
//   We do the check **symmetrically**: also project the world landmarks
//   into old_kf's camera frame and verify there.  A real loop passes
//   both directions; a wrong match typically passes one and fails the
//   other.

#include <vector>
#include <Eigen/Core>
#include <camodocal/camera_models/CameraFactory.h>

namespace vins_sparse {

struct LoopVerifOptions {
    // Per-landmark inlier threshold in pixels.
    double inlier_px_thresh  = 8.0;
    // Median residual cap (in pixels).
    double median_px_thresh  = 5.0;
    // Minimum inlier count.
    int    min_inliers       = 12;
    // Minimum inlier ratio (out of the matched points tested).
    double min_inlier_ratio  = 0.30;
    // Symmetric check: re-test with the world landmarks projected into
    // old_kf's camera frame.
    bool   symmetric         = true;
    // Image dimensions, used to reject landmarks that fall off-screen.
    int    image_width       = 752;
    int    image_height      = 480;
};

struct LoopVerifResult {
    bool        verified          = false;
    int         n_tested          = 0;
    int         n_inliers_forward = 0;     // world -> cur camera
    int         n_inliers_backward= 0;     // world -> old camera
    double      median_residual_px = 0.0;  // inlier median (forward)
    double      mean_residual_px   = 0.0;  // inlier mean (forward)
    double      inlier_ratio       = 0.0;  // forward n_inliers / n_tested
};

class LoopGeometricVerifier {
  public:
    explicit LoopGeometricVerifier(const LoopVerifOptions& opt = {});

    // Verify a candidate loop closure.
    //
    //   R_w_cur, t_w_cur : 3x3 / 3x1 -- world -> cur_kf *camera* frame
    //                      (i.e. R_w_c, t_w_c; include qic/tic).
    //   R_w_old, t_w_old : 3x3 / 3x1 -- world -> old_kf *camera* frame.
    //   pts_3d_w         : N landmarks in the world frame.  In VINS
    //                      these are the `point_3d` of cur_kf.
    //   pts_2d_cur       : N observed feature pixels in cur_kf image
    //                      (cur_kf's `point_2d_uv`).
    //   pts_2d_old       : N observed feature pixels in old_kf image
    //                      (cur_kf's `matched_2d_old` after PnP).
    //   cam              : camodocal::CameraPtr used to project 3-D to
    //                      pixels (e.g. m_camera from the VINS global
    //                      state).
    //
    // pts_3d_w.size() must equal pts_2d_cur.size() and pts_2d_old.size().
    LoopVerifResult verify(
        const Eigen::Matrix3d& R_w_cur, const Eigen::Vector3d& t_w_cur,
        const Eigen::Matrix3d& R_w_old, const Eigen::Vector3d& t_w_old,
        const std::vector<Eigen::Vector3d>& pts_3d_w,
        const std::vector<Eigen::Vector2d>& pts_2d_cur,
        const std::vector<Eigen::Vector2d>& pts_2d_old,
        const camodocal::CameraPtr& cam);

  private:
    LoopVerifOptions opt_;
};

}  // namespace vins_sparse

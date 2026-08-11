#include "loop_geometric_verifier.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vins_sparse {

namespace {

// Project a 3-D point in the camera frame to pixel coordinates using
// the camodocal model (handles distortion if camodocal has been told so).
// Returns false if the point is behind the camera or non-finite.
inline bool projectCamodocal(const Eigen::Vector3d& p_cam,
                             const camodocal::CameraPtr& cam,
                             Eigen::Vector2d* px) {
    if (!cam) return false;
    if (!std::isfinite(p_cam.x()) ||
        !std::isfinite(p_cam.y()) ||
        !std::isfinite(p_cam.z())) return false;
    if (p_cam.z() <= 1e-6) return false;
    Eigen::Vector2d p2d;
    Eigen::Vector3d P = p_cam;
    cam->spaceToPlane(P, p2d);
    if (!std::isfinite(p2d.x()) || !std::isfinite(p2d.y())) return false;
    *px = p2d;
    return true;
}

struct DirResult {
    int    n_tested  = 0;
    int    n_inliers = 0;
    double median_px = 0.0;
    double mean_px   = 0.0;
};

// Forward direction: world landmarks, observations in target frame.
// R_w_t, t_w_t  transform world -> target camera frame.
DirResult checkDirection(const std::vector<Eigen::Vector3d>& pts_3d_w,
                         const std::vector<Eigen::Vector2d>& obs_2d_t,
                         const Eigen::Matrix3d& R_w_t,
                         const Eigen::Vector3d& t_w_t,
                         const camodocal::CameraPtr& cam,
                         const LoopVerifOptions& opt) {
    DirResult r;
    if (pts_3d_w.size() != obs_2d_t.size() || pts_3d_w.empty()) return r;
    r.n_tested = (int)pts_3d_w.size();

    int inlier_sum = 0;
    double sum = 0.0;
    std::vector<double> inlier_residuals;
    inlier_residuals.reserve(r.n_tested);

    for (size_t i = 0; i < pts_3d_w.size(); ++i) {
        // P_cam = R_w_t^T * (P_w - t_w_t)
        const Eigen::Vector3d p_cam =
            R_w_t.transpose() * (pts_3d_w[i] - t_w_t);
        Eigen::Vector2d px;
        if (!projectCamodocal(p_cam, cam, &px)) continue;
        // Reject off-screen (with a 1-pixel border).
        if (px.x() <  0.0 || px.x() >= opt.image_width  - 1.0) continue;
        if (px.y() <  0.0 || px.y() >= opt.image_height - 1.0) continue;
        const double res2 = (px - obs_2d_t[i]).norm();
        if (res2 < opt.inlier_px_thresh) {
            ++inlier_sum;
            sum += res2;
            inlier_residuals.push_back(res2);
        }
    }
    r.n_inliers = inlier_sum;
    if (inlier_sum > 0) {
        r.mean_px = sum / inlier_sum;
        std::sort(inlier_residuals.begin(), inlier_residuals.end());
        r.median_px = inlier_residuals[inlier_residuals.size() / 2];
    }
    return r;
}

}  // namespace

LoopGeometricVerifier::LoopGeometricVerifier(const LoopVerifOptions& opt)
    : opt_(opt) {}

LoopVerifResult LoopGeometricVerifier::verify(
    const Eigen::Matrix3d& R_w_cur, const Eigen::Vector3d& t_w_cur,
    const Eigen::Matrix3d& R_w_old, const Eigen::Vector3d& t_w_old,
    const std::vector<Eigen::Vector3d>& pts_3d_w,
    const std::vector<Eigen::Vector2d>& pts_2d_cur,
    const std::vector<Eigen::Vector2d>& pts_2d_old,
    const camodocal::CameraPtr& cam)
{
    LoopVerifResult res;

    // ----- forward: world -> cur camera -----
    const DirResult fwd = checkDirection(pts_3d_w, pts_2d_cur,
                                         R_w_cur, t_w_cur, cam, opt_);
    res.n_tested           = fwd.n_tested;
    res.n_inliers_forward  = fwd.n_inliers;
    res.median_residual_px = fwd.median_px;
    res.mean_residual_px   = fwd.mean_px;
    res.inlier_ratio       = (fwd.n_tested > 0)
                             ? double(fwd.n_inliers) / fwd.n_tested
                             : 0.0;

    // ----- backward: world -> old camera -----
    int n_inliers_back = 0;
    if (opt_.symmetric) {
        const DirResult bwd = checkDirection(pts_3d_w, pts_2d_old,
                                             R_w_old, t_w_old, cam, opt_);
        n_inliers_back = bwd.n_inliers;
    }
    res.n_inliers_backward = n_inliers_back;

    // Decision rule.
    const bool forward_ok =
        (fwd.n_inliers >= opt_.min_inliers) &&
        (res.inlier_ratio >= opt_.min_inlier_ratio) &&
        (fwd.median_px <= opt_.median_px_thresh);

    const bool backward_ok = !opt_.symmetric ||
        (n_inliers_back >= opt_.min_inliers);

    res.verified = forward_ok && backward_ok;
    return res;
}

}  // namespace vins_sparse

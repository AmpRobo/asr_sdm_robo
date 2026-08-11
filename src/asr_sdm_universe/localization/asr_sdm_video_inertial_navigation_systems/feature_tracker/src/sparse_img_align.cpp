#include "sparse_img_align.h"
#include "robust_weight.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <Eigen/Cholesky>

namespace vins_sparse {

namespace {

// SO(3) exponential map: rotation vector (axis*angle) -> 3x3 rotation matrix.
inline Eigen::Matrix3d expMapSO3(const Eigen::Vector3d& omega) {
    const double theta_sq = omega.squaredNorm();
    const double theta = std::sqrt(theta_sq);
    Eigen::Matrix3d Omega;
    Omega <<     0.0, -omega.z(),  omega.y(),
              omega.z(),     0.0, -omega.x(),
             -omega.y(),  omega.x(),     0.0;
    if (theta < 1e-9) {
        return Eigen::Matrix3d::Identity() + Omega;
    }
    const Eigen::Matrix3d Omega_sq = Omega * Omega;
    const double a = std::sin(theta) / theta;
    const double b = (1.0 - std::cos(theta)) / theta_sq;
    return Eigen::Matrix3d::Identity() + a * Omega + b * Omega_sq;
}

// Skew-symmetric matrix from a 3-vector.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),    0.0;
    return S;
}

// Bilinear-interpolated pixel lookup with edge clamping.
inline float sampleBilinear(const cv::Mat& img, double u, double v) {
    const int w = img.cols;
    const int h = img.rows;
    if (u < 0) u = 0;
    if (v < 0) v = 0;
    if (u > w - 1.0001) u = w - 1.0001;
    if (v > h - 1.0001) v = h - 1.0001;
    const int u0 = static_cast<int>(std::floor(u));
    const int v0 = static_cast<int>(std::floor(v));
    const int u1 = std::min(u0 + 1, w - 1);
    const int v1 = std::min(v0 + 1, h - 1);
    const double du = u - u0;
    const double dv = v - v0;
    const double w00 = (1.0 - du) * (1.0 - dv);
    const double w01 = du * (1.0 - dv);
    const double w10 = (1.0 - du) * dv;
    const double w11 = du * dv;
    const uint8_t* row0 = img.ptr<uint8_t>(v0);
    const uint8_t* row1 = img.ptr<uint8_t>(v1);
    return static_cast<float>(w00 * row0[u0] + w01 * row0[u1] +
                              w10 * row1[u0] + w11 * row1[u1]);
}

// Per-feature caches: ref patch (flat), bearing in camera frame, projection
// Jacobian d(uv)/d(xi) at the reference frame's pinhole (scaled by level).
struct FeatureCache {
    Eigen::Vector3d bearing;          // unit ray in cam frame (depth=1)
    Eigen::Matrix<double, 2, 6> J_proj;  // d(uv)/d(xi) w.r.t. SE(3) on ref img
};

// One pyramid level, with all ref patches in a (patch_area, n_features)
// matrix. Caches are precomputed once per level and reused across the
// Gauss-Newton iterations to keep the cost low.
struct LevelCache {
    int level = 0;
    double scale = 1.0;                  // 1.0 / (1 << level)
    int patch_size = 0;
    int patch_area = 0;
    int border = 1;                       // border for finite differences
    int patch_with_border = 0;            // patch_size + 2*border
    int patch_area_wb = 0;
    Eigen::MatrixXd ref_patches;         // patch_area x n_features (intensity)
    std::vector<FeatureCache> features;
    // 2x6 projection Jacobians evaluated at the reference pixel (level px).
    // We keep them cached so the inverse-compositional step is cheap.
};

}  // namespace

SparseAlignResult sparseAlign(
    const std::vector<cv::Mat>& ref_pyr,
    const std::vector<cv::Mat>& cur_pyr,
    const std::vector<cv::Point2f>& ref_pts,
    const std::vector<Eigen::Vector3d>& ref_bearings,
    const Eigen::Matrix3d& R_prior,
    const Eigen::Vector3d& t_prior,
    const SparseAlignOptions& opt) {
    SparseAlignResult out;
    if (ref_pyr.empty() || cur_pyr.empty() ||
        ref_pyr.size() != cur_pyr.size() ||
        ref_pts.empty() || ref_pts.size() != ref_bearings.size()) {
        return out;
    }
    if (opt.max_level >= static_cast<int>(ref_pyr.size()) ||
        opt.min_level < 0 || opt.min_level > opt.max_level) {
        return out;
    }
    if (opt.patch_size < 2) return out;

    // Pinhole projection helper. The reference feature's bearing is the
    // unit ray (X/Z, Y/Z, 1) in the *reference* camera frame. We treat
    // the unknown depth as 1 for the purpose of the photometric warp
    // (this is the standard half-direct trick: the affine brightness
    // term absorbs the depth-induced scale change in patch intensity).
    // If fx != fy (e.g. D435i: fx=453, fy=604) the caller should set
    // them via the options. We fall back to focal_length otherwise.
    const double fx = (opt.fx > 0.0) ? opt.fx : opt.focal_length;
    const double fy = (opt.fy > 0.0) ? opt.fy : opt.focal_length;
    const double cx = (opt.cx > 0.0) ? opt.cx : opt.image_width * 0.5;
    const double cy = (opt.cy > 0.0) ? opt.cy : opt.image_height * 0.5;

    // State.
    Eigen::Matrix3d R = R_prior;
    Eigen::Vector3d t = t_prior;
    double alpha = 0.0;
    double beta = 0.0;
    const bool est_alpha = opt.estimate_alpha;
    const bool est_beta = opt.estimate_beta;

    // Tukey weight.
    TukeyWeight tw(opt.tukey_b);

    // Precompute level caches from coarse to fine.
    const int n_features = static_cast<int>(ref_pts.size());
    const int patch_size = opt.patch_size;
    const int border = 1;
    const int patch_wb = patch_size + 2 * border;
    const int patch_area = patch_size * patch_size;
    const int patch_area_wb = patch_wb * patch_wb;

    // Project the pinhole projection Jacobian for a 3D point at bearing
    // b (depth=1) on the unit plane. We need d(uv)/d(xi) where xi is the
    // 6-vector SE(3) perturbation on the current pose, such that a point
    // at depth d and bearing b in the ref frame is at
    //     X_cur = R * (d * b) + t
    // and the resulting uv = (X_cur.x / X_cur.z, X_cur.y / X_cur.z) * f + c.
    //
    // For unit-depth (d=1), the Jacobian reduces to
    //   J_proj = (1 / X_cur.z^2) * [ X_cur.z * I_2 - X_cur * e_3^T ] * R * G
    // where G = [I_3 | -skew(d*b)]_3x6. We precompute the first part (the
    // 2x6 block) once per feature per level.
    auto computeJProj = [&](int level, const FeatureCache& f, double scale) {
        Eigen::Matrix<double, 2, 6> J;
        const Eigen::Vector3d X = R * f.bearing + t;
        if (X.z() <= 1e-6) {
            J.setZero();
            return J;
        }
        // 3D point in current frame is (X, Y, Z) = R*b + t.
        // Pinhole projection (level 0 pixels): u = fx * X/Z + cx,
        //                                       v = fy * Y/Z + cy.
        // d(uv)/d(X) = [fx/Z, 0, -fx*X/Z^2; 0, fy/Z, -fy*Y/Z^2]
        // d(X)/d(xi) for xi = (trans, rot):
        //   trans: dX/d(trans) = I_3
        //   rot : dX/d(rot) = -[X]_x   (small right-multiplied rotation)
        // Combine and scale to current level.
        const double inv_z = 1.0 / X.z();
        const double inv_z2 = inv_z * inv_z;
        Eigen::Matrix<double, 2, 3> Jproj;
        Jproj << fx * inv_z, 0.0, -fx * X.x() * inv_z2,
                 0.0, fy * inv_z, -fy * X.y() * inv_z2;
        // G for the SE(3) perturbation (translation + rotation) acting on
        // a reference point at unit depth. The rotation part is
        // -skew(bearing) in the world frame. We chain through R to
        // account for the current pose's rotation when we differentiate
        // the world-frame point with respect to the SE(3) increment.
        Eigen::Matrix<double, 3, 6> G;
        G.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        G.block<3, 3>(0, 3) = -R * skew(f.bearing);
        // Compose and rescale to the current pyramid level.
        J = Jproj * G;
        J *= scale;
        return J;
    };

    // Build a per-level cache. We re-evaluate the reference patches and
    // gradients at every level since the pyramid changes.
    std::vector<LevelCache> caches(opt.max_level + 1);
    for (int level = opt.min_level; level <= opt.max_level; ++level) {
        LevelCache& L = caches[level];
        L.level = level;
        L.scale = 1.0 / (1 << level);
        L.patch_size = patch_size;
        L.patch_area = patch_area;
        L.border = border;
        L.patch_with_border = patch_wb;
        L.patch_area_wb = patch_area_wb;
        L.ref_patches = Eigen::MatrixXd::Zero(patch_area, n_features);
        L.features.resize(n_features);

        const cv::Mat& ref_img = ref_pyr[level];
        const int rows = ref_img.rows;
        const int cols = ref_img.cols;
        const int stride = static_cast<int>(ref_img.step);
        const int patch_center = patch_size / 2;
        const int patch_center_wb = patch_wb / 2;

        for (int i = 0; i < n_features; ++i) {
            const double u = ref_pts[i].x * L.scale;
            const double v = ref_pts[i].y * L.scale;
            L.features[i].bearing = ref_bearings[i];
            L.features[i].J_proj = computeJProj(level, L.features[i], L.scale);

            // Skip the feature if the patch is fully outside the image.
            const double u_tl = u - patch_center_wb;
            const double v_tl = v - patch_center_wb;
            if (u_tl < 0 || v_tl < 0 ||
                u_tl + patch_wb > cols - 1 ||
                v_tl + patch_wb > rows - 1) {
                continue;  // ref patch stays zero, treated as invisible below
            }
            const int u_tl_i = static_cast<int>(std::floor(u_tl));
            const int v_tl_i = static_cast<int>(std::floor(v_tl));
            const double subpix_u = u_tl - u_tl_i;
            const double subpix_v = v_tl - v_tl_i;
            const double wtl = (1.0 - subpix_u) * (1.0 - subpix_v);
            const double wtr = subpix_u * (1.0 - subpix_v);
            const double wbl = (1.0 - subpix_u) * subpix_v;
            const double wbr = subpix_u * subpix_v;

            // Interpolate (patch_wb x patch_wb) patch with border.
            double buf[64];  // patch_wb^2 <= 64 for patch_size <= 6
            const int kSize = patch_wb * patch_wb;
            for (int y = 0; y < patch_wb; ++y) {
                const uint8_t* r0 = ref_img.ptr<uint8_t>(v_tl_i + y) + u_tl_i;
                const uint8_t* r1 = r0 + stride;
                for (int x = 0; x < patch_wb; ++x) {
                    const double v00 = r0[x];
                    const double v01 = r0[x + 1];
                    const double v10 = r1[x];
                    const double v11 = r1[x + 1];
                    buf[y * patch_wb + x] = wtl * v00 + wtr * v01 +
                                            wbl * v10 + wbr * v11;
                }
            }
            (void)kSize;

            // Fill ref_patches using central patch with central differences.
            for (int y = 0; y < patch_size; ++y) {
                for (int x = 0; x < patch_size; ++x) {
                    const int idx_center =
                        (x + border) + patch_wb * (y + border);
                    L.ref_patches(y * patch_size + x, i) =
                        buf[idx_center];
                }
            }
        }
    }

    // Coarse-to-fine Gauss-Newton with inverse-compositional state.
    int total_iter = 0;
    double final_chi2 = std::numeric_limits<double>::infinity();
    int last_n_meas = 0;

    const int kStateDim = 8;
    Eigen::Matrix<double, kStateDim, kStateDim> H;
    Eigen::Matrix<double, kStateDim, 1> g;
    int last_level = -1;

    for (int level = opt.max_level; level >= opt.min_level; --level) {
        last_level = level;
        const LevelCache& L = caches[level];

        for (int it = 0; it < opt.max_iter; ++it) {
            total_iter++;
            H.setZero();
            g.setZero();
            double chi2 = 0.0;
            last_n_meas = 0;

            // Linearization + accumulation.
            for (int i = 0; i < n_features; ++i) {
                const FeatureCache& f = L.features[i];
                // Skip if the reference patch is zero (feature outside image).

                // Project the unit-depth reference point into the current
                // frame's image plane. u, v are level-0 (full-res) pixel
                // coordinates; we scale them to the current level for the
                // pyramid lookup.
                const Eigen::Vector3d X = R * f.bearing + t;
                if (X.z() <= 1e-6) { continue; }
                const double u_lvl = (fx * X.x() / X.z() + cx) * L.scale;
                const double v_lvl = (fy * X.y() / X.z() + cy) * L.scale;
                const double u_tl = u_lvl - (L.patch_size / 2.0);
                const double v_tl = v_lvl - (L.patch_size / 2.0);
                const cv::Mat& cur_img = cur_pyr[level];
                if (u_tl < 0 || v_tl < 0 ||
                    u_tl + L.patch_size > cur_img.cols - 1 ||
                    v_tl + L.patch_size > cur_img.rows - 1) {
                    continue;
                }

                // Bilinear sample current patch and central-difference
                // gradients. We pre-multiply by the projection Jacobian
                // (d uv / d xi) and the patch's intensity gradient.
                const Eigen::Matrix<double, 2, 6>& Juv = f.J_proj;
                for (int y = 0; y < L.patch_size; ++y) {
                    for (int x = 0; x < L.patch_size; ++x) {
                        const double cu = u_tl + x;
                        const double cv = v_tl + y;
                        const float Ic = sampleBilinear(cur_img, cu, cv);
                        const float dx_img = 0.5f * (sampleBilinear(cur_img, cu + 1, cv) -
                                                     sampleBilinear(cur_img, cu - 1, cv));
                        const float dy_img = 0.5f * (sampleBilinear(cur_img, cu, cv + 1) -
                                                     sampleBilinear(cur_img, cu, cv - 1));
                        const double Ir = L.ref_patches(y * L.patch_size + x, i);
                        const double r_pred = Ic * (1.0 + alpha) + beta;
                        const double r = r_pred - Ir;

                        // Build the 8-vector Jacobian: [d r/d xi; d r/d alpha; d r/d beta]
                        // Juv.row(k) is the 1x6 derivative of the k-th image
                        // coordinate (u for k=0, v for k=1) with respect to
                        // the SE(3) pose perturbation, evaluated at the
                        // reference point.
                        Eigen::Matrix<double, kStateDim, 1> Ji;
                        Ji.head<6>() =
                            dx_img * Juv.row(0).transpose() +
                            dy_img * Juv.row(1).transpose();
                        Ji(6) = est_alpha ? -Ic : 0.0;
                        Ji(7) = est_beta ? -1.0 : 0.0;

                        // Reweight residual using the current estimate of r.
                        // We use the unscaled residual to look up the weight;
                        // tukey_b provides the natural scale for residuals in
                        // pixel intensity.
                        const float w = tw.weight(static_cast<float>(r) / opt.tukey_b);
                        const double wr = w * r;
                        chi2 += wr * r;
                        H.noalias() += Ji * Ji.transpose() * w;
                        g.noalias() -= Ji * wr;
                        last_n_meas++;
                    }
                }
            }

            if (last_n_meas == 0) {
                // No overlap; stop iterating at this level.
                break;
            }

            const double mean_chi2 = chi2 / last_n_meas;
            final_chi2 = mean_chi2;

            // Add IMU prior: lambda * I added to the rotational block.
            H(0, 0) += opt.lambda_trans; H(1, 1) += opt.lambda_trans; H(2, 2) += opt.lambda_trans;
            H(3, 3) += opt.lambda_rot; H(4, 4) += opt.lambda_rot; H(5, 5) += opt.lambda_rot;
            if (est_alpha) H(6, 6) += opt.lambda_rot;
            if (est_beta) H(7, 7) += opt.lambda_rot;

            // Solve H * dx = g. If H is not PD, fall back to LM-style damping.
            Eigen::Matrix<double, kStateDim, 1> dx;
            Eigen::LDLT<Eigen::Matrix<double, kStateDim, kStateDim>> ldlt(H);
            if (ldlt.info() != Eigen::Success) {
                H.diagonal().array() += 1e-3;
                ldlt.compute(H);
            }
            dx = ldlt.solve(g);
            if (!dx.allFinite()) {
                break;
            }

            // Convergence check.
            const double x_norm = dx.cwiseAbs().maxCoeff();
            if (x_norm < opt.eps) {
                break;
            }

            // Apply the update: R <- R * exp(-dxi_head6), t <- t - dx[0:3]
            // (we keep the camera-from-reference convention; the negative
            // sign is the inverse compositional step on the left side).
            const Eigen::Matrix3d dR = expMapSO3(-dx.segment<3>(3));
            const Eigen::Vector3d dt = -dx.head<3>();
            R = R * dR;
            t = R * dt + t;  // equivalent to R * (-dx[0:3]) + t under the linearization
            if (est_alpha) alpha = (alpha - dx(6)) / (1.0 + dx(6));
            if (est_beta)  beta  = (beta  - dx(7)) / (1.0 + dx(6));
        }
    }

    out.R_k_kminus1 = R;
    out.t_k_kminus1 = t;
    out.alpha = alpha;
    out.beta = beta;
    out.iterations = total_iter;
    out.n_meas = last_n_meas;
    out.final_chi2 = final_chi2;
    out.success =
        total_iter >= opt.min_iter_for_ok &&
        last_n_meas >= opt.min_features &&
        std::isfinite(final_chi2) &&
        final_chi2 < opt.chi2_thresh;
    return out;
}

}  // namespace vins_sparse

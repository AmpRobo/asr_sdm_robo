#include "td_pre_calibrator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vins_sparse {

namespace {

inline double angleOf(const Eigen::Matrix3d& R) {
    // || log(R) ||_2 in SO(3) is the rotation angle, range [0, π].
    double cos_theta = (R.trace() - 1.0) * 0.5;
    if (cos_theta >  1.0) cos_theta =  1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    return std::acos(cos_theta);
}

}  // namespace

TdPreCalibrator::TdPreCalibrator(const TdPreCalibratorOptions& opt)
    : opt_(opt) {}

bool TdPreCalibrator::beginFrame(double t_prev, double t_cur) {
    resetFrame();
    const double dt = t_cur - t_prev;
    if (dt < opt_.min_dt) return false;

    t_prev_   = t_prev;
    t_cur_    = t_cur;
    have_frame_ = true;
    return true;
}

void TdPreCalibrator::setVisionRotation(const Eigen::Matrix3d& R_vision) {
    if (!have_frame_) return;
    if (angleOf(R_vision) < (opt_.min_angle_deg * M_PI / 180.0)) {
        // Frame has too little rotation to be informative.
        have_frame_ = false;
        return;
    }
    R_vision_ = R_vision;
}

void TdPreCalibrator::addGyro(double t, double wx, double wy, double wz) {
    if (!have_frame_) return;
    if (t < t_prev_ || t > t_cur_) return;   // outside this frame's window
    cur_gyro_.push_back({t, wx, wy, wz});
}

void TdPreCalibrator::resetFrame() {
    cur_gyro_.clear();
    have_frame_ = false;
    R_vision_   = Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d TdPreCalibrator::integrate(double t0, double t1) const {
    // Piecewise-constant gyro:  R = Π_k exp(ω_k · Δt_k)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (cur_gyro_.empty()) return R;

    for (size_t k = 0; k + 1 < cur_gyro_.size(); ++k) {
        const double a = cur_gyro_[k].t;
        const double b = cur_gyro_[k + 1].t;
        if (b <= t0 || a >= t1) continue;
        const double lo = std::max(a, t0);
        const double hi = std::min(b, t1);
        const double dt = hi - lo;
        if (dt <= 0) continue;
        Eigen::Vector3d w(cur_gyro_[k].wx,
                          cur_gyro_[k].wy,
                          cur_gyro_[k].wz);
        Eigen::Matrix3d dR;
        // Axis-angle:  R = I + sin(θ) K + (1-cos(θ)) K^2
        double theta = w.norm() * dt;
        if (theta < 1e-12) {
            dR = Eigen::Matrix3d::Identity();
        } else {
            Eigen::Vector3d axis = w / w.norm();
            Eigen::AngleAxisd aa(theta, axis);
            dR = aa.toRotationMatrix();
        }
        R = dR * R;
    }
    return R;
}

double TdPreCalibrator::solve(double* out_final_angle_rad) {
    if (!have_frame_ || cur_gyro_.size() < 2) {
        if (out_final_angle_rad) *out_final_angle_rad = std::numeric_limits<double>::quiet_NaN();
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 1) Coarse grid search.
    double best_td    = 0.0;
    double best_angle = std::numeric_limits<double>::infinity();
    const double step = (opt_.td_max - opt_.td_min) /
                        std::max(1, opt_.n_samples - 1);
    for (int i = 0; i < opt_.n_samples; ++i) {
        const double td = opt_.td_min + i * step;
        Eigen::Matrix3d R_imu = integrate(t_prev_ + td, t_cur_ + td);
        // Cost: || log( R_imu * R_vision^T ) ||
        Eigen::Matrix3d dR = R_imu * R_vision_.transpose();
        double ang = angleOf(dR);
        if (ang < best_angle) {
            best_angle = ang;
            best_td    = td;
        }
    }

    // 2) Newton / golden-section refinement around the best grid point.
    //    Use a small 1-D ternary search since the cost is well-behaved
    //    (single valley near zero for reasonable motion).
    double lo = std::max(opt_.td_min, best_td - opt_.refine_window);
    double hi = std::min(opt_.td_max, best_td + opt_.refine_window);
    for (int it = 0; it < opt_.max_iter; ++it) {
        const double m1 = lo + (hi - lo) / 3.0;
        const double m2 = hi - (hi - lo) / 3.0;
        Eigen::Matrix3d R1 = integrate(t_prev_ + m1, t_cur_ + m1);
        Eigen::Matrix3d R2 = integrate(t_prev_ + m2, t_cur_ + m2);
        double a1 = angleOf(R1 * R_vision_.transpose());
        double a2 = angleOf(R2 * R_vision_.transpose());
        if (a1 < a2) hi = m2; else lo = m1;
    }
    best_td = 0.5 * (lo + hi);
    // Recompute the angle at the refined td.
    {
        Eigen::Matrix3d R_imu = integrate(t_prev_ + best_td, t_cur_ + best_td);
        best_angle = angleOf(R_imu * R_vision_.transpose());
    }

    samples_.push_back({best_td, best_angle});
    last_td_ = best_td;
    if (out_final_angle_rad) *out_final_angle_rad = best_angle;
    return best_td;
}

double TdPreCalibrator::meanTd() const {
    if (samples_.empty()) return 0.0;
    double s = 0.0;
    for (const auto& x : samples_) s += x.td;
    return s / (double)samples_.size();
}

}  // namespace vins_sparse

#include "imu_preintegrate.h"

#include <algorithm>
#include <cmath>

namespace vins_sparse {

namespace {

// SO(3) exponential map: rotation vector (axis * angle) -> 3x3 rotation.
inline Eigen::Matrix3d expMap(const Eigen::Vector3d& omega) {
    const double theta_sq = omega.squaredNorm();
    const double theta = std::sqrt(theta_sq);
    Eigen::Matrix3d Omega;
    Omega <<     0.0, -omega.z(),  omega.y(),
              omega.z(),     0.0, -omega.x(),
             -omega.y(),  omega.x(),     0.0;
    if (theta < 1e-8) {
        // First-order Taylor: R = I + [omega]_x
        return Eigen::Matrix3d::Identity() + Omega;
    }
    const Eigen::Matrix3d Omega_sq = Omega * Omega;
    const double a = std::sin(theta) / theta;
    const double b = (1.0 - std::cos(theta)) / theta_sq;
    return Eigen::Matrix3d::Identity() + a * Omega + b * Omega_sq;
}

}  // namespace

void ImuPreintegrator::add(const ImuSample& s) {
    // Maintain time order. If a late sample slips in (rare for ROS QoS),
    // find its slot; otherwise append.
    if (!buf_.empty() && s.t < buf_.back().t) {
        auto it = std::upper_bound(buf_.begin(), buf_.end(), s.t,
                                  [](double t, const ImuSample& smp) {
                                      return t < smp.t;
                                  });
        buf_.insert(it, s);
    } else {
        buf_.push_back(s);
    }
}

void ImuPreintegrator::pruneOld(double t_min) {
    while (!buf_.empty() && buf_.front().t < t_min) {
        buf_.pop_front();
    }
}

Eigen::Matrix3d ImuPreintegrator::integrateRotation(
    double t_start,
    double t_end,
    const Eigen::Vector3d& gyro_bias) {
    if (t_end <= t_start || buf_.empty()) {
        return Eigen::Matrix3d::Identity();
    }

    // Binary search for the first sample with t >= t_start.
    auto lower = std::lower_bound(
        buf_.begin(), buf_.end(), t_start,
        [](const ImuSample& smp, double t) { return smp.t < t; });

    // First sample strictly after t_end (exclusive upper bound for the loop).
    auto upper = std::upper_bound(
        buf_.begin(), buf_.end(), t_end,
        [](double t, const ImuSample& smp) { return t < smp.t; });

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (lower == buf_.end()) {
        return R;
    }

    // Handle the half-interval from t_start to lower->t.
    if (lower->t > t_start) {
        if (lower != buf_.begin()) {
            const ImuSample& prev = *(lower - 1);
            const double dt = lower->t - t_start;
            const Eigen::Vector3d w = prev.gyro - gyro_bias;
            R = expMap(w * dt) * R;
        }
        // If lower is buf_.begin() and lower->t > t_start, no prior sample;
        // skip the leading half-interval (we have no extrapolation signal).
    }

    for (auto it = lower; it != upper; ++it) {
        auto next = it + 1;
        double t_next = (next == upper) ? t_end : next->t;
        if (t_next <= it->t) continue;
        const double dt = t_next - it->t;
        const Eigen::Vector3d w = it->gyro - gyro_bias;
        R = expMap(w * dt) * R;
    }

    return R;
}

}  // namespace vins_sparse

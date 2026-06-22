#pragma once

#include <deque>
#include <Eigen/Core>

namespace vins_sparse {

struct ImuSample {
    double t = 0.0;             // seconds
    Eigen::Vector3d gyro;       // angular velocity (rad/s), body frame
    Eigen::Vector3d accel;      // linear acceleration (m/s^2), body frame
};

// Frame-to-frame SO(3) rotation preintegration on an IMU buffer.
// Mirrors the SVO pro pattern in
//   rpg_svo_pro_open/svo/src/frame_handler_base.cpp : getMotionPrior()
// Keeps a time-ordered deque of IMU samples and integrates the angular
// velocity between any two timestamps using a left-product SO(3) recurrence
//   R_{k+1} = R_k * exp((w - b_g) * dt)
class ImuPreintegrator {
public:
    ImuPreintegrator() = default;

    // Insert a new sample at the back. Caller is expected to push in
    // monotonically non-decreasing time order; we keep order in case ROS
    // occasionally delivers slightly out-of-order messages.
    void add(const ImuSample& s);

    // Integrate the rotation from t_start to t_end (seconds). Samples that
    // straddle the boundaries contribute their respective half-interval.
    // Returns Identity if t_end <= t_start or no samples are available.
    Eigen::Matrix3d integrateRotation(
        double t_start,
        double t_end,
        const Eigen::Vector3d& gyro_bias = Eigen::Vector3d::Zero());

    // Drop samples whose timestamp is strictly less than t_min.
    void pruneOld(double t_min);

    // Number of samples currently held.
    size_t size() const { return buf_.size(); }

    // Oldest and newest sample timestamps (or 0 if empty).
    double oldestT() const { return buf_.empty() ? 0.0 : buf_.front().t; }
    double newestT() const { return buf_.empty() ? 0.0 : buf_.back().t; }

private:
    std::deque<ImuSample> buf_;
};

}  // namespace vins_sparse

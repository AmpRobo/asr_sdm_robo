#include <svo/imu_preprocessor.h>
#include <svo/global.h>
#include <vikit/math_utils.h>
#include <Eigen/SVD>
#include <opencv2/core.hpp>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace svo
{

ImuPreprocessor::ImuPreprocessor(const ImuPreprocessorOptions& options)
  : options_(options),
    gyro_bias_(Eigen::Vector3d::Zero()),
    acc_bias_(Eigen::Vector3d::Zero()),
    gravity_direction_(Eigen::Vector3d(0, 0, -1)),
    R_cam_imu_(Eigen::Matrix3d::Identity()),
    t_cam_imu_(Eigen::Vector3d::Zero()),
    scale_(1.0),
    translation_(Eigen::Vector3d::Zero()),
    bias_estimated_(false)
{
  alignment_result_.valid = false;
  alignment_result_.gyro_bias = Eigen::Vector3d::Zero();
  alignment_result_.acc_bias = Eigen::Vector3d::Zero();
  alignment_result_.R_cam_imu = Eigen::Matrix3d::Identity();
  alignment_result_.t_cam_imu = Eigen::Vector3d::Zero();
  alignment_result_.scale = 1.0;
  alignment_result_.gravity_magnitude = options_.gravity_magnitude;
  alignment_result_.residual_rotation_deg = 0.0;
  alignment_result_.residual_position_m = 0.0;
}

void ImuPreprocessor::reset()
{
  raw_imu_.clear();
  visual_poses_.clear();
  aligned_trajectory_.clear();
  alignment_result_.valid = false;
  gyro_bias_.setZero();
  acc_bias_.setZero();
  R_cam_imu_.setIdentity();
  t_cam_imu_.setZero();
  scale_ = 1.0;
  translation_.setZero();
  bias_estimated_ = false;
}

void ImuPreprocessor::addImuData(double timestamp, const Eigen::Vector3d& omega, const Eigen::Vector3d& acc)
{
  raw_imu_.push_back({timestamp, omega, acc});
}

void ImuPreprocessor::addVisualPose(double timestamp, const Eigen::Vector3d& position,
                                    const Eigen::Quaterniond& orientation)
{
  visual_poses_.push_back({timestamp, position, orientation});
}

// ============================================================================
// Step 1: Estimate gyro and accel bias from stationary period
// ============================================================================
void ImuPreprocessor::estimateBias(const std::vector<ImuRawData>& imu_data,
                                   double stationary_start, double stationary_end,
                                   Eigen::Vector3d& gyro_bias_out,
                                   Eigen::Vector3d& acc_bias_out,
                                   Eigen::Vector3d& gravity_dir_out)
{
  std::vector<Eigen::Vector3d> omega_samples;
  std::vector<Eigen::Vector3d> acc_samples;

  for (const auto& m : imu_data)
  {
    if (m.timestamp >= stationary_start && m.timestamp <= stationary_end)
    {
      omega_samples.push_back(m.omega);
      acc_samples.push_back(m.acc);
    }
  }

  if (omega_samples.empty())
  {
    SVO_WARN_STREAM("ImuPreprocessor: No IMU data in stationary period ["
        << stationary_start << ", " << stationary_end << "]");
    gyro_bias_out.setZero();
    acc_bias_out.setZero();
    gravity_dir_out = Eigen::Vector3d(0, 0, -1);
    return;
  }

  // Mean gyro = gyro bias (stationary)
  Eigen::Vector3d mean_omega = Eigen::Vector3d::Zero();
  for (const auto& w : omega_samples) mean_omega += w;
  mean_omega /= static_cast<double>(omega_samples.size());

  // Mean accel ≈ acc_bias + g * R^T * [0,0,1]
  // During stationary: gravity direction is constant in world frame
  // acc = acc_bias + R_world_imu^T * g_world
  // For a general orientation, mean accel direction gives gravity in IMU frame
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
  for (const auto& a : acc_samples) mean_acc += a;
  mean_acc /= static_cast<double>(acc_samples.size());

  // Gravity direction in IMU frame = normalized mean accel
  gravity_dir_out = mean_acc.normalized();

  // For gravity magnitude: ||mean_acc|| ≈ ||g|| + small bias effect
  double g_mag = mean_acc.norm();
  // acc_bias = mean_acc - g_mag * gravity_dir
  acc_bias_out = mean_acc - g_mag * gravity_dir_out;

  gyro_bias_out = mean_omega;

  if (options_.verbose)
  {
    std::cout << "[ImuPreprocessor] Bias estimation (stationary period):" << std::endl;
    std::cout << "  Samples: " << omega_samples.size() << std::endl;
    std::cout << "  Gyro bias:  [" << gyro_bias_out.transpose() << "] rad/s" << std::endl;
    std::cout << "  Acc bias:   [" << acc_bias_out.transpose() << "] m/s^2" << std::endl;
    std::cout << "  Gravity dir: [" << gravity_dir_out.transpose() << "], mag=" << g_mag << std::endl;
  }
}

// ============================================================================
// Step 2: Integrate IMU trajectory with bias correction
// ============================================================================
void ImuPreprocessor::integrateTrajectory(const std::vector<ImuRawData>& raw_imu,
                                          const Eigen::Vector3d& gyro_bias,
                                          const Eigen::Vector3d& acc_bias,
                                          const Eigen::Vector3d& gravity_direction,
                                          const Eigen::Vector3d& init_velocity,
                                          const Eigen::Vector3d& init_position,
                                          const Eigen::Quaterniond& init_orientation,
                                          std::vector<ImuAlignedData>& aligned_trajectory)
{
  aligned_trajectory.clear();
  if (raw_imu.empty()) return;

  Eigen::Quaterniond R_imu_world = init_orientation;
  Eigen::Vector3d v = init_velocity;
  Eigen::Vector3d p = init_position;
  const double g = options_.gravity_magnitude;
  const Eigen::Vector3d g_world(0, 0, -g);

  for (size_t i = 0; i < raw_imu.size(); ++i)
  {
    const auto& m = raw_imu[i];
    double dt = 0.0;
    if (i > 0)
      dt = m.timestamp - raw_imu[i - 1].timestamp;

    if (dt <= 0.0 || dt > 0.5) dt = 0.0;

    // Bias-corrected IMU measurements
    Eigen::Vector3d omega_corr = m.omega - gyro_bias;
    Eigen::Vector3d acc_corr = m.acc - acc_bias;

    // Integrate orientation using mid-point method
    if (dt > 0.0)
    {
      const double wn = omega_corr.norm();
      if (wn > 1e-8)
      {
        Eigen::Quaterniond dq(Eigen::AngleAxisd(wn * dt, omega_corr.normalized()));
        R_imu_world = R_imu_world * dq;
      }

      // Transform accel to world frame: a_world = R_imu_world * acc_corr + g_world
      Eigen::Vector3d a_world = R_imu_world * acc_corr + g_world;

      // Integrate velocity and position (Euler)
      v += a_world * dt;
      p += v * dt;
    }

    ImuAlignedData ad;
    ad.timestamp = m.timestamp;
    ad.omega = omega_corr;
    ad.acc = acc_corr;
    ad.orientation = R_imu_world;
    ad.velocity = v;
    ad.position = p;
    aligned_trajectory.push_back(ad);
  }
}

// ============================================================================
// Step 3: Umeyama alignment (closed-form, 6-DOF + scale)
// ============================================================================
bool ImuPreprocessor::alignUmeyama(const std::vector<Eigen::Vector3d>& P,
                                   const std::vector<Eigen::Vector3d>& Q,
                                   Eigen::Matrix3d& R,
                                   Eigen::Vector3d& t,
                                   double& s)
{
  if (P.size() != Q.size() || P.size() < 3)
    return false;

  const size_t N = P.size();

  // Compute means
  Eigen::Vector3d mean_P = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_Q = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < N; ++i)
  {
    mean_P += P[i];
    mean_Q += Q[i];
  }
  mean_P /= static_cast<double>(N);
  mean_Q /= static_cast<double>(N);

  // Center the points
  std::vector<Eigen::Vector3d> P_centered(N), Q_centered(N);
  for (size_t i = 0; i < N; ++i)
  {
    P_centered[i] = P[i] - mean_P;
    Q_centered[i] = Q[i] - mean_Q;
  }

  // Compute variance of P
  double var_P = 0.0;
  for (size_t i = 0; i < N; ++i)
    var_P += P_centered[i].squaredNorm();
  var_P /= static_cast<double>(N);

  if (var_P < 1e-10)
  {
    R.setIdentity();
    t = mean_Q - mean_P;
    s = 1.0;
    return false;
  }

  // Compute covariance matrix: sigma_PQ = (1/N) * sum(p_i * q_i^T)
  Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < N; ++i)
    sigma += P_centered[i] * Q_centered[i].transpose();
  sigma /= static_cast<double>(N);

  // SVD: sigma = U * S * V^T
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& U = svd.matrixU();
  const Eigen::Matrix3d& V = svd.matrixV();

  // R = V * U^T, with reflection correction
  R = V * U.transpose();
  if (R.determinant() < 0)
  {
    Eigen::Matrix3d V_adj = V;
    V_adj.col(2) *= -1;
    R = V_adj * U.transpose();
  }

  // Scale: s = trace(S * V^T * U) / var_P
  Eigen::Matrix3d S_matrix = Eigen::Matrix3d::Zero();
  S_matrix.diagonal() = svd.singularValues();
  double trace_SR = (S_matrix * V.transpose() * U).trace();
  s = trace_SR / var_P;

  // Translation: t = mean_Q - s * R * mean_P
  t = mean_Q - s * R * mean_P;

  return true;
}

double ImuPreprocessor::huberWeight(double residual, double threshold)
{
  if (options_.use_huber)
  {
    if (std::abs(residual) <= threshold)
      return 1.0;
    else
      return threshold / std::abs(residual);
  }
  return 1.0;
}

double ImuPreprocessor::computeAlignmentResidual(const std::vector<ImuAlignedData>& imu_traj,
                                                 const std::vector<VisualFramePose>& vis_poses,
                                                 const Eigen::Matrix3d& R,
                                                 const Eigen::Vector3d& t,
                                                 double s)
{
  if (imu_traj.empty() || vis_poses.empty()) return 0.0;

  double total_err = 0.0;
  int count = 0;
  for (const auto& vp : vis_poses)
  {
    // Find closest IMU data point
    size_t best_idx = 0;
    double best_dt = 1e9;
    for (size_t i = 0; i < imu_traj.size(); ++i)
    {
      double dt = std::abs(imu_traj[i].timestamp - vp.timestamp);
      if (dt < best_dt)
      {
        best_dt = dt;
        best_idx = i;
      }
    }

    if (best_dt > 0.5) continue;  // Skip if too far

    Eigen::Vector3d imu_pos = imu_traj[best_idx].position;
    Eigen::Vector3d aligned_pos = s * R * imu_pos + t;
    double err = (aligned_pos - vp.position).norm();
    total_err += err;
    ++count;
  }

  return count > 0 ? total_err / count : 0.0;
}

ImuAlignmentResult ImuPreprocessor::alignTrajectories(const std::vector<ImuAlignedData>& imu_trajectory,
                                                      const std::vector<VisualFramePose>& visual_poses)
{
  ImuAlignmentResult result;

  if (imu_trajectory.size() < 10 || visual_poses.size() < 3)
  {
    result.valid = false;
    result.message = "Insufficient data for alignment";
    return result;
  }

  // Build correspondences: for each visual pose, find nearest IMU pose
  std::vector<Eigen::Vector3d> P_imu;  // IMU positions
  std::vector<Eigen::Vector3d> Q_vis;   // Visual positions

  for (const auto& vp : visual_poses)
  {
    size_t best_idx = 0;
    double best_dt = 1e9;
    for (size_t i = 0; i < imu_trajectory.size(); ++i)
    {
      double dt = std::abs(imu_trajectory[i].timestamp - vp.timestamp);
      if (dt < best_dt)
      {
        best_dt = dt;
        best_idx = i;
      }
    }

    if (best_dt < 0.1)  // Only use close matches
    {
      P_imu.push_back(imu_trajectory[best_idx].position);
      Q_vis.push_back(vp.position);
    }
  }

  if (P_imu.size() < 5)
  {
    result.valid = false;
    result.message = "Not enough matched poses for alignment";
    return result;
  }

  // Umeyama alignment: Q_vis = s * R * P_imu + t
  Eigen::Matrix3d R_align;
  Eigen::Vector3d t_align;
  double s_align;
  bool ok = alignUmeyama(P_imu, Q_vis, R_align, t_align, s_align);

  if (!ok)
  {
    result.valid = false;
    result.message = "Umeyama alignment failed";
    return result;
  }

  // Compute residual
  double residual_m = computeAlignmentResidual(imu_trajectory, visual_poses,
                                               R_align, t_align, s_align);

  // Compute rotation angle from identity (how much is the IMU-Camera rotation)
  Eigen::AngleAxisd aa(R_align);
  double angle_deg = std::abs(aa.angle()) * 180.0 / M_PI;
  if (angle_deg > 90.0) angle_deg = 180.0 - angle_deg;

  result.valid = true;
  result.R_cam_imu = R_align;
  result.t_cam_imu = t_align;
  result.scale = s_align;
  result.residual_position_m = residual_m;
  result.residual_rotation_deg = angle_deg;
  result.message = "Alignment successful";

  if (options_.verbose)
  {
    std::cout << "[ImuPreprocessor] Trajectory alignment result:" << std::endl;
    std::cout << "  Matched poses: " << P_imu.size() << std::endl;
    std::cout << "  Scale: " << s_align << std::endl;
    std::cout << "  Rotation angle from identity: " << angle_deg << " deg" << std::endl;
    std::cout << "  Translation: [" << t_align.transpose() << "] m" << std::endl;
    std::cout << "  Position residual: " << residual_m << " m" << std::endl;
    std::cout << "  R_cam_imu (IMU->Visual):" << std::endl;
    std::cout << R_align << std::endl;
  }

  return result;
}

// ============================================================================
// Step 0: Detect stationary period from visual motion
// ============================================================================
bool ImuPreprocessor::detectStationaryPeriod(double& start_time, double& end_time,
                                            const std::vector<VisualFramePose>& poses) const
{
  if (poses.size() < static_cast<size_t>(options_.stationary_window_frames * 2))
    return false;

  // Find the first window with minimum displacement
  const int window = options_.stationary_window_frames;
  double best_start = poses[0].timestamp;
  double best_end = poses[window].timestamp;
  double best_disp = 1e9;

  for (size_t i = 0; i + window < poses.size(); ++i)
  {
    const auto& p_start = poses[i];
    const auto& p_end = poses[i + window];
    double disp = (p_end.position - p_start.position).norm();
    double dt = p_end.timestamp - p_start.timestamp;
    double vel = (dt > 0) ? disp / dt : 1e9;

    if (vel < best_disp && disp < 0.05)  // < 5cm displacement in window
    {
      best_disp = vel;
      best_start = p_start.timestamp;
      best_end = p_end.timestamp;
    }
  }

  if (best_disp > 0.01)  // No stationary period found (> 1cm/s)
    return false;

  start_time = best_start;
  end_time = best_end;
  return true;
}

// ============================================================================
// Full pipeline
// ============================================================================
ImuAlignmentResult ImuPreprocessor::processAll()
{
  ImuAlignmentResult result;
  result.valid = false;

  if (raw_imu_.empty() || visual_poses_.empty())
  {
    result.message = "No data loaded";
    return result;
  }

  // Step 1: Detect stationary period
  double stationary_start, stationary_end;
  bool has_stationary = detectStationaryPeriod(stationary_start, stationary_end, visual_poses_);

  if (!has_stationary)
  {
    // Fallback: use first 1 second
    stationary_start = raw_imu_.front().timestamp;
    stationary_end = stationary_start + 1.0;
    if (options_.verbose)
    {
      std::cout << "[ImuPreprocessor] No stationary period detected, using first 1s for bias estimation" << std::endl;
    }
  }

  // Step 2: Estimate bias
  estimateBias(raw_imu_, stationary_start, stationary_end,
               gyro_bias_, acc_bias_, gravity_direction_);

  // Get initial orientation from gravity alignment
  Eigen::Quaterniond init_orientation;
  {
    const Eigen::Vector3d& g = gravity_direction_;
    const Eigen::Vector3d z = g.normalized();
    Eigen::Vector3d p(1, 0, 0);
    if (std::fabs(z.dot(p)) > 0.9) p = Eigen::Vector3d(0, 1, 0);
    Eigen::Vector3d y = z.cross(p).normalized();
    Eigen::Vector3d x = y.cross(z).normalized();
    Eigen::Matrix3d C;
    C.col(0) = x; C.col(1) = y; C.col(2) = z;
    init_orientation = Eigen::Quaterniond(C);
  }

  // Step 3: Integrate IMU trajectory
  integrateTrajectory(raw_imu_, gyro_bias_, acc_bias_, gravity_direction_,
                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     init_orientation, aligned_trajectory_);

  if (aligned_trajectory_.empty())
  {
    result.message = "IMU integration produced no data";
    return result;
  }

  // Step 4: Align to visual trajectory
  result = alignTrajectories(aligned_trajectory_, visual_poses_);

  if (!result.valid)
    return result;

  // Store alignment parameters
  R_cam_imu_ = result.R_cam_imu;
  t_cam_imu_ = result.t_cam_imu;
  scale_ = result.scale;
  translation_ = result.t_cam_imu;
  bias_estimated_ = true;
  alignment_result_ = result;
  alignment_result_.gyro_bias = gyro_bias_;
  alignment_result_.acc_bias = acc_bias_;
  alignment_result_.gravity_direction = gravity_direction_;
  alignment_result_.gravity_magnitude = options_.gravity_magnitude;

  return result;
}

// ============================================================================
// Apply alignment to get visual-frame pose from IMU pose
// ============================================================================
Eigen::Vector3d ImuPreprocessor::applyAlignmentPosition(const Eigen::Vector3d& imu_pos) const
{
  return scale_ * R_cam_imu_ * imu_pos + translation_;
}

Eigen::Quaterniond ImuPreprocessor::applyAlignmentOrientation(const Eigen::Quaterniond& imu_quat) const
{
  return Eigen::Quaterniond(R_cam_imu_ * imu_quat.toRotationMatrix());
}

// ============================================================================
// Interpolate IMU data to a specific timestamp
// ============================================================================
bool ImuPreprocessor::interpolateImu(double timestamp, Eigen::Vector3d& omega, Eigen::Vector3d& acc) const
{
  if (raw_imu_.empty()) return false;
  if (timestamp <= raw_imu_.front().timestamp)
  {
    omega = raw_imu_.front().omega;
    acc = raw_imu_.front().acc;
    return true;
  }
  if (timestamp >= raw_imu_.back().timestamp)
  {
    omega = raw_imu_.back().omega;
    acc = raw_imu_.back().acc;
    return true;
  }

  // Binary search for bracketing interval
  size_t lo = 0, hi = raw_imu_.size() - 1;
  while (lo + 1 < hi)
  {
    size_t mid = (lo + hi) / 2;
    if (raw_imu_[mid].timestamp < timestamp)
      lo = mid;
    else
      hi = mid;
  }

  const auto& m0 = raw_imu_[lo];
  const auto& m1 = raw_imu_[hi];
  double dt_total = m1.timestamp - m0.timestamp;
  double alpha = (dt_total > 1e-9) ? (timestamp - m0.timestamp) / dt_total : 0.0;
  alpha = std::max(0.0, std::min(1.0, alpha));

  omega = m0.omega + alpha * (m1.omega - m0.omega);
  acc = m0.acc + alpha * (m1.acc - m0.acc);
  return true;
}

// ============================================================================
// Get aligned pose at specific timestamp
// ============================================================================
bool ImuPreprocessor::getAlignedPose(double timestamp,
                                      Eigen::Vector3d& position,
                                      Eigen::Quaterniond& orientation) const
{
  if (!hasAlignment() || aligned_trajectory_.empty())
    return false;

  if (timestamp <= aligned_trajectory_.front().timestamp)
  {
    position = applyAlignmentPosition(aligned_trajectory_.front().position);
    orientation = applyAlignmentOrientation(aligned_trajectory_.front().orientation);
    return true;
  }
  if (timestamp >= aligned_trajectory_.back().timestamp)
  {
    position = applyAlignmentPosition(aligned_trajectory_.back().position);
    orientation = applyAlignmentOrientation(aligned_trajectory_.back().orientation);
    return true;
  }

  // Find bracketing interval
  size_t lo = 0, hi = aligned_trajectory_.size() - 1;
  while (lo + 1 < hi)
  {
    size_t mid = (lo + hi) / 2;
    if (aligned_trajectory_[mid].timestamp < timestamp)
      lo = mid;
    else
      hi = mid;
  }

  const auto& d0 = aligned_trajectory_[lo];
  const auto& d1 = aligned_trajectory_[hi];
  double dt_total = d1.timestamp - d0.timestamp;
  double alpha = (dt_total > 1e-9) ? (timestamp - d0.timestamp) / dt_total : 0.0;
  alpha = std::max(0.0, std::min(1.0, alpha));

  Eigen::Vector3d imu_pos = d0.position + alpha * (d1.position - d0.position);
  Eigen::Quaterniond imu_quat = d0.orientation.slerp(alpha, d1.orientation);

  position = applyAlignmentPosition(imu_pos);
  orientation = applyAlignmentOrientation(imu_quat);
  return true;
}

// ============================================================================
// Predict pose using IMU between two visual frames (for frame interpolation)
// ============================================================================
bool ImuPreprocessor::predictPoseBetweenFrames(double t0, double t1,
                                                const Eigen::Vector3d& pos0,
                                                const Eigen::Quaterniond& quat0,
                                                double t_predict,
                                                Eigen::Vector3d& pos_predict,
                                                Eigen::Quaterniond& quat_predict)
{
  if (!hasAlignment() || aligned_trajectory_.empty())
    return false;

  if (t_predict <= t0)
  {
    pos_predict = pos0;
    quat_predict = quat0;
    return true;
  }
  if (t_predict >= t1)
  {
    // Extrapolate using last known velocity
    Eigen::Vector3d last_pos;
    Eigen::Quaterniond last_quat;
    if (!getAlignedPose(t1, last_pos, last_quat))
      return false;
    Eigen::Vector3d vel = (last_pos - pos0) / (t1 - t0);
    double dt = t_predict - t1;
    pos_predict = last_pos + vel * dt;
    quat_predict = last_quat;
    return true;
  }

  // Find IMU trajectory segment between t0 and t1
  std::vector<ImuAlignedData> segment;
  for (const auto& d : aligned_trajectory_)
  {
    if (d.timestamp >= t0 && d.timestamp <= t1)
      segment.push_back(d);
  }

  if (segment.size() < 2)
    return false;

  // Integrate IMU between t0 and t_predict using the aligned IMU trajectory
  // as displacement delta from the visual anchor
  Eigen::Vector3d imu_delta_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond imu_delta_q = Eigen::Quaterniond::Identity();

  for (size_t i = 0; i < segment.size() - 1; ++i)
  {
    double dt_seg = segment[i + 1].timestamp - segment[i].timestamp;
    if (segment[i + 1].timestamp > t_predict)
    {
      // Interpolate last segment to t_predict
      double alpha = (t_predict - segment[i].timestamp) / dt_seg;
      imu_delta_pos += (segment[i + 1].position - segment[i].position) * alpha;
      imu_delta_q = segment[i].orientation.slerp(alpha, segment[i + 1].orientation);
      break;
    }
    else
    {
      imu_delta_pos += segment[i + 1].position - segment[i].position;
      imu_delta_q = segment[i + 1].orientation;
    }
  }

  // Apply alignment: scaled, rotated delta
  Eigen::Vector3d aligned_delta = scale_ * R_cam_imu_ * imu_delta_pos;
  Eigen::Quaterniond aligned_delta_q(R_cam_imu_ * imu_delta_q.toRotationMatrix());

  pos_predict = pos0 + aligned_delta;
  quat_predict = quat0 * aligned_delta_q;
  return true;
}

bool ImuPreprocessor::saveToYaml(const std::string& filename) const
{
  if (!bias_estimated_)
  {
    SVO_WARN_STREAM("ImuPreprocessor: Cannot save - calibration not completed");
    return false;
  }

  try
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
      SVO_WARN_STREAM("ImuPreprocessor: Cannot open file for writing: " << filename);
      return false;
    }

    cv::Mat_<double> gb_mat(3, 1), ab_mat(3, 1), gd_mat(3, 1), t_mat(3, 1);
    gb_mat << gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z();
    ab_mat << acc_bias_.x(), acc_bias_.y(), acc_bias_.z();
    gd_mat << gravity_direction_.x(), gravity_direction_.y(), gravity_direction_.z();
    t_mat << t_cam_imu_.x(), t_cam_imu_.y(), t_cam_imu_.z();
    cv::Mat R_mat(3, 3, CV_64F);
    memcpy(R_mat.data, R_cam_imu_.data(), 9 * sizeof(double));
    fs << "gyro_bias" << gb_mat;
    fs << "acc_bias" << ab_mat;
    fs << "gravity_direction" << gd_mat;
    fs << "R_cam_imu" << R_mat;
    fs << "t_cam_imu" << t_mat;
    fs << "scale" << scale_;
    fs << "gravity_magnitude" << options_.gravity_magnitude;
    fs << "residual_position_m" << alignment_result_.residual_position_m;
    fs << "residual_rotation_deg" << alignment_result_.residual_rotation_deg;
    fs.release();

    std::cout << "[ImuPreprocessor] Calibration saved to: " << filename << std::endl;
    return true;
  }
  catch (const std::exception& e)
  {
    SVO_WARN_STREAM("ImuPreprocessor: saveToYaml failed: " << e.what());
    return false;
  }
}

bool ImuPreprocessor::loadFromYaml(const std::string& filename)
{
  try
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      SVO_WARN_STREAM("ImuPreprocessor: Cannot open calibration file: " << filename);
      return false;
    }

    cv::Mat gb, ab, gd, Rmat, tmat;
    fs["gyro_bias"] >> gb;
    fs["acc_bias"] >> ab;
    fs["gravity_direction"] >> gd;
    fs["R_cam_imu"] >> Rmat;
    fs["t_cam_imu"] >> tmat;
    fs["scale"] >> scale_;
    double g_mag;
    fs["gravity_magnitude"] >> g_mag;

    gyro_bias_ = Eigen::Map<Eigen::Vector3d>(gb.ptr<double>());
    acc_bias_ = Eigen::Map<Eigen::Vector3d>(ab.ptr<double>());
    gravity_direction_ = Eigen::Map<Eigen::Vector3d>(gd.ptr<double>());
    R_cam_imu_ = Eigen::Map<Eigen::Matrix3d>(Rmat.ptr<double>());
    t_cam_imu_ = Eigen::Map<Eigen::Vector3d>(tmat.ptr<double>());
    translation_ = t_cam_imu_;
    options_.gravity_magnitude = g_mag;
    bias_estimated_ = true;

    // Fill alignment_result_
    alignment_result_.valid = true;
    alignment_result_.gyro_bias = gyro_bias_;
    alignment_result_.acc_bias = acc_bias_;
    alignment_result_.R_cam_imu = R_cam_imu_;
    alignment_result_.t_cam_imu = t_cam_imu_;
    alignment_result_.scale = scale_;
    alignment_result_.gravity_direction = gravity_direction_;
    alignment_result_.gravity_magnitude = g_mag;
    fs["residual_position_m"] >> alignment_result_.residual_position_m;
    fs["residual_rotation_deg"] >> alignment_result_.residual_rotation_deg;

    fs.release();

    if (options_.verbose)
    {
      std::cout << "[ImuPreprocessor] Calibration loaded from: " << filename << std::endl;
      std::cout << "  gyro_bias:   [" << gyro_bias_.transpose() << "] rad/s" << std::endl;
      std::cout << "  acc_bias:    [" << acc_bias_.transpose() << "] m/s^2" << std::endl;
      std::cout << "  R_cam_imu:" << std::endl;
      std::cout << R_cam_imu_ << std::endl;
      std::cout << "  scale: " << scale_ << std::endl;
    }
    return true;
  }
  catch (const std::exception& e)
  {
    SVO_WARN_STREAM("ImuPreprocessor: loadFromYaml failed: " << e.what());
    return false;
  }
}

}  // namespace svo

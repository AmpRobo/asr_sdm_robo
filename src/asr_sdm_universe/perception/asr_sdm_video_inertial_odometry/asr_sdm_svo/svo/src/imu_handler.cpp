#include <iomanip>
#include <numeric>
#include <svo/global.h>
#include <svo/imu_types.h>
#include <vikit/math_utils.h>
#include <Eigen/SVD>

namespace
{

double stdVec(const std::vector<double>& v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / v.size());
}

}  // namespace

namespace svo
{

void PreintegratedImuMeasurement::addMeasurement(const ImuMeasurement& m)
{
  if (last_imu_measurement_set_)
  {
    const double dt = m.timestamp_ - last_imu_measurement.timestamp_;
    if (dt <= 0.0 || dt > 0.5) return;  // Reject non-positive or unreasonably large dt

    const Eigen::Vector3d a = last_imu_measurement.linear_acceleration_ - acc_bias_;
    const Eigen::Vector3d w = last_imu_measurement.angular_velocity_ - omega_bias_;

    // Saturate angular velocity and acceleration to physical limits
    const double omega_norm = w.norm();
    const double acc_norm = a.norm();
    Eigen::Vector3d w_corr = w;
    Eigen::Vector3d a_corr = a;
    if (omega_norm > saturation_omega_max_)
    {
      w_corr = w.normalized() * saturation_omega_max_;
    }
    if (acc_norm > saturation_accel_max_)
    {
      a_corr = a.normalized() * saturation_accel_max_;
    }

    // Second-order integration:
    delta_t_ij_ += delta_v_ij_ * dt + (delta_R_ij_.toRotationMatrix() * a_corr) * dt * dt * 0.5;
    delta_v_ij_ += delta_R_ij_.toRotationMatrix() * a_corr * dt;
    const double wn = w_corr.norm();
    const Eigen::Quaterniond R_incr = Eigen::Quaterniond(
        Eigen::AngleAxisd(wn * dt, wn > 1e-8 ? w_corr.normalized() : Eigen::Vector3d::UnitX()));
    delta_R_ij_ = delta_R_ij_ * R_incr;
    dt_sum_ += dt;
  }
  last_imu_measurement_set_ = true;
  last_imu_measurement = m;
}

void PreintegratedImuMeasurement::addMeasurements(const ImuMeasurements& ms)
{
  for (const ImuMeasurement& m : ms)
    addMeasurement(m);
}

ImuHandler::ImuHandler(
    const ImuCalibration& imu_calib,
    const ImuInitialization& imu_init,
    const IMUHandlerOptions& options)
  : imu_calib_(imu_calib),
    imu_init_(imu_init),
    acc_bias_(imu_init.acc_bias),
    omega_bias_(imu_init.omega_bias),
    calibrator_(std::make_unique<ImuOnlineCalibrator>()),
    options_(options)
{}

ImuHandler::~ImuHandler()
{}

bool ImuHandler::getMeasurements(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    const bool delete_old_measurements,
    ImuMeasurements& measurements)
{
  (void)delete_old_measurements;  // Buffer deletion is disabled; keeping all data is safe and correct.

  if (new_cam_timestamp <= old_cam_timestamp)
    return false;

  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurements available.");
    return false;
  }

  const double t1 = old_cam_timestamp - imu_calib_.delay_imu_cam;
  const double t2 = new_cam_timestamp - imu_calib_.delay_imu_cam;

  // Buffer layout: deque front=oldest, back=newest (chronological order).
  // push_back keeps newest at back; forward iteration (begin->end) = oldest->newest.
  const size_t N = measurements_.size();

  // Find the newest measurement <= t1 (iterating newest -> oldest via reverse indices).
  // Start from back (newest), go toward front (oldest).
  size_t it1_idx = N;  // default: not found
  for (size_t i = N; i-- > 0; )
  {
    if (measurements_[i].timestamp_ <= t1)
    {
      it1_idx = i;
      break;
    }
  }

  // Find the newest measurement < t2 (iterating newest -> oldest via reverse indices).
  // We need the LAST IMU before t2 (i.e., the newest measurement whose timestamp is strictly
  // less than t2). Reverse search ensures we find the newest qualifying element.
  size_t it2_idx = N;  // default: not found
  for (size_t i = N; i-- > 0; )
  {
    if (measurements_[i].timestamp_ < t2)
    {
      it2_idx = i;
      break;  // First match from back = newest one < t2
    }
  }

  // Handle case where t1 is ahead of all IMU in buffer (read-ahead of camera).
  // Fall back to the newest IMU as the "before t1" reference point.
  if (it1_idx == N && it2_idx < N)
  {
    SVO_WARN_STREAM("ImuHandler: t1="
        << std::fixed << std::setprecision(9) << t1
        << " > newest_imu_ts=" << std::setprecision(9) << measurements_.back().timestamp_
        << ". Using newest as reference.");
    it1_idx = it2_idx;  // Start integration from the newest IMU before t2
  }

  // If we have at least 1 IMU measurement (it2_idx >= it1_idx), we can integrate.
  // For single-measurement case (it2_idx == it1_idx), integration range is 0 but
  // we can still use the available data — the getRelativeRotationPrior loop
  // will handle it gracefully. This handles the read-ahead camera case.
  if (it1_idx == N || it2_idx == N)
  {
    const double it1_ts = (it1_idx < N ? measurements_[it1_idx].timestamp_ : -1.0);
    const double it2_ts = (it2_idx < N ? measurements_[it2_idx].timestamp_ : -1.0);
    SVO_WARN_STREAM("ImuHandler: getMeasurements FAIL: "
        << "t1=" << std::fixed << std::setprecision(9) << t1
        << " t2=" << std::setprecision(9) << t2 << " dt=" << (t2-t1)
        << " buf_sz=" << N
        << " oldest_ts=" << std::setprecision(9) << measurements_.front().timestamp_
        << " newest_ts=" << std::setprecision(9) << measurements_.back().timestamp_
        << " it1_idx=" << it1_idx << " it2_idx=" << it2_idx
        << " it1_ts=" << std::setprecision(9) << it1_ts
        << " it2_ts=" << std::setprecision(9) << it2_ts);
    return false;
  }

  // If it2_idx == it1_idx: exactly one IMU measurement covers both t1 and t2.
  // Include it so that getRelativeRotationPrior can integrate from t1 to t2 using this
  // measurement's angular velocity (dt = t2 - delay - imu_ts, then R = exp(omega * dt)).
  if (it2_idx == it1_idx)
  {
    SVO_DEBUG_STREAM("ImuHandler: getMeasurements (single-meas): t1="
        << std::fixed << std::setprecision(9) << t1
        << " t2=" << std::setprecision(9) << t2
        << " n=1 buf_sz=" << N);
  }

  const double gap = t2 - measurements_[it2_idx].timestamp_;
  if (gap > imu_calib_.max_imu_delta_t)
  {
    SVO_WARN_STREAM("ImuHandler: Newest IMU measurement is too old. "
        << "Gap: " << gap * 1000.0 << " ms. "
        << "t2=" << t2 << " it2_ts=" << measurements_[it2_idx].timestamp_
        << " it2_idx=" << it2_idx << " buf_sz=" << N);
    return false;
  }

  // Copy [it1_idx, it2_idx+1] — inclusive of it2_idx.
  // When it2_idx == it1_idx this gives exactly 1 measurement.
  measurements.insert(measurements.begin(), measurements_.begin() + it1_idx, measurements_.begin() + it2_idx + 1);
  SVO_DEBUG_STREAM("ImuHandler: getMeasurements OK t1=" << std::fixed << std::setprecision(9) << t1 << " t2=" << std::setprecision(9) << t2
      << " n_meas=" << measurements.size()
      << " buf_sz=" << N << " oldest_ts=" << std::setprecision(9) << measurements_.front().timestamp_
      << " newest_ts=" << std::setprecision(9) << measurements_.back().timestamp_);
  return true;
}

bool ImuHandler::getClosestMeasurement(
    const double timestamp,
    ImuMeasurement& measurement) const
{
  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurements available.");
    return false;
  }

  double dt_best = std::numeric_limits<double>::max();
  double img_ts_corrected = timestamp - imu_calib_.delay_imu_cam;
  for (const ImuMeasurement& m : measurements_)
  {
    const double dt = std::abs(m.timestamp_ - img_ts_corrected);
    if (dt < dt_best)
    {
      dt_best = dt;
      measurement = m;
    }
  }

  // Use 10x max_imu_delta_t as the initial-attitude lookup threshold
  // (much looser than the preintegration threshold, since we just need one nearby measurement)
  const double max_lookup_dt = std::max(imu_calib_.max_imu_delta_t * 10.0, 1.0);
  if (dt_best > max_lookup_dt)
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurement found within threshold. "
                    "Closest: " << dt_best * 1000.0 << " ms > " << max_lookup_dt * 1000.0 << " ms.");
    return false;
  }
  return true;
}

bool ImuHandler::getRelativeRotationPrior(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    bool delete_old_measurements,
    Eigen::Quaterniond& R_oldimu_newimu)
{
  ImuMeasurements measurements;
  if (!getMeasurements(old_cam_timestamp, new_cam_timestamp,
                       delete_old_measurements, measurements))
    return false;

  // Allow single-measurement case: integrate angular velocity from old_cam_ts to new_cam_ts
  // using the available IMU data. The last-iteration logic in the loop handles dt correctly.
  if (measurements.empty())
    return false;

  // Integrate angular velocity from t1 to t2 (oldest to newest, forward).
  // R_oldimu_newimu represents the rotation from IMU at old timestamp to IMU at new timestamp.
  R_oldimu_newimu.setIdentity();

  // Forward iteration: oldest -> newest
  ImuMeasurements::const_iterator it = measurements.begin();
  ImuMeasurements::const_iterator it_next = measurements.begin();
  ++it_next;

  // Get bias from calibrator if available, otherwise fall back to static bias
  const Eigen::Vector3d omega_bias = currentGyroBias();

  for (; it != measurements.end(); ++it, ++it_next)
  {
    double dt = 0.0;
    if (it_next == measurements.end())
    {
      // Last measurement: integrate to the (corrected) camera timestamp
      dt = new_cam_timestamp - imu_calib_.delay_imu_cam - it->timestamp_;
    }
    else
    {
      dt = it_next->timestamp_ - it->timestamp_;
    }

    if (dt <= 0.0 || dt > 0.5)
    {
      SVO_WARN_STREAM("ImuHandler: Rejected dt=" << dt << " in rotation prior integration.");
      continue;
    }

    Eigen::Vector3d omega_raw = it->angular_velocity_ - omega_bias;
    Eigen::Vector3d omega_corrected = omega_raw;
    const double omega_norm = omega_corrected.norm();
    if (omega_norm > imu_calib_.saturation_omega_max)
    {
      omega_corrected = omega_corrected.normalized() * imu_calib_.saturation_omega_max;
      SVO_WARN_STREAM("ImuHandler: Gyro saturation detected. omega_norm="
                      << omega_norm << " > " << imu_calib_.saturation_omega_max);
    }

    const double theta = omega_corrected.norm() * dt;
    if (theta > 1e-8)
    {
      Eigen::Quaterniond R_incr(Eigen::AngleAxisd(
          theta, omega_corrected.normalized()));
      // Compose: first rotation (oldest) applied, then subsequent ones.
      // R = R * R_incr: current accumulated rotation, then new increment.
      R_oldimu_newimu = R_oldimu_newimu * R_incr;
    }
  }

  return true;
}

bool ImuHandler::getTransformedRotationPrior(
    double old_cam_timestamp, double new_cam_timestamp,
    Eigen::Quaterniond& R_cam_delta)
{
  Eigen::Quaterniond R_imu_delta;
  if (!getRelativeRotationPrior(old_cam_timestamp, new_cam_timestamp, false, R_imu_delta))
    return false;

  // IMU rotation is in body frame; camera rotation is the inverse (conjugate).
  // Note: this method is no longer called by vo_node — rotation prior is permanently
  // disabled in SVO (direct methods cannot accept external rotation priors).
  R_cam_delta = R_imu_delta;
  return true;
}

bool ImuHandler::addImuMeasurement(const ImuMeasurement& m)
{
  ulock_t lock(measurements_mut_);
  static int dbg_count = 0;
  if (dbg_count < 3)
  {
    SVO_WARN_STREAM("ImuHandler: addImuMeasurement #" << dbg_count
        << " ts=" << std::fixed << std::setprecision(9) << m.timestamp_
        << " buf_size=" << measurements_.size());
    ++dbg_count;
  }
  // push_back: front=oldest, back=newest (canonical chronological order for binary search)
  measurements_.push_back(m);
  if (options_.temporal_stationary_check)
    temporal_imu_window_.push_back(m);

  // Feed raw omega to calibrator for inter-frame delta integration
  feedOmegaToCalibrator(m.timestamp_, m.angular_velocity_);

  return true;
}

void ImuHandler::feedOmegaToCalibrator(double timestamp, const Eigen::Vector3d& omega)
{
  if (calibrator_)
    calibrator_->feedOmegaMeasurement(timestamp, omega);
}

Eigen::Vector3d ImuHandler::currentGyroBias() const
{
  if (calibrator_ && calibrator_->isReady())
    return calibrator_->gyroBias();
  return omega_bias_;
}

bool ImuHandler::getInitialAttitude(
    double timestamp,
    Eigen::Quaterniond& R_imu_world) const
{
  ImuMeasurement m;
  if (!getClosestMeasurement(timestamp, m))
  {
    SVO_WARN_STREAM("ImuHandler: Could not get initial attitude. No IMU measurements.");
    return false;
  }

  // Align world Z-axis with measured gravity direction.
  const Eigen::Vector3d& g = m.linear_acceleration_;
  const Eigen::Vector3d z = g.normalized();

  Eigen::Vector3d p(1, 0, 0);
  Eigen::Vector3d p_alt(0, 1, 0);
  if (std::fabs(z.dot(p)) > std::fabs(z.dot(p_alt)))
    p = p_alt;

  Eigen::Vector3d y = z.cross(p);
  y.normalize();
  const Eigen::Vector3d x = y.cross(z);

  Eigen::Matrix3d C_imu_world;
  C_imu_world.col(0) = x;
  C_imu_world.col(1) = y;
  C_imu_world.col(2) = z;

  R_imu_world = Eigen::Quaterniond(C_imu_world);
  SVO_DEBUG_STREAM("ImuHandler: Initial attitude from gravity. g=" << g.transpose());
  return true;
}

void ImuHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
  temporal_imu_window_.clear();
}

IMUTemporalStatus ImuHandler::checkTemporalStatus(const double time_sec)
{
  if (!options_.temporal_stationary_check)
  {
    SVO_WARN_STREAM("ImuHandler: Stationary check is disabled.");
    return IMUTemporalStatus::kMoving;
  }

  ulock_t lock(measurements_mut_);
  if (temporal_imu_window_.empty() ||
      temporal_imu_window_.front().timestamp_ < time_sec)
    return IMUTemporalStatus::kUnkown;

  // Find indices: start = first measurement older than time_sec, end = oldest in window
  ssize_t start_idx = -1;
  ssize_t end_idx = -1;
  for (ssize_t idx = 0; idx < static_cast<ssize_t>(temporal_imu_window_.size()); ++idx)
  {
    if (temporal_imu_window_[static_cast<size_t>(idx)].timestamp_ >= time_sec)
    {
      end_idx = idx;
      break;
    }
  }
  if (end_idx == -1)
    return IMUTemporalStatus::kUnkown;

  // Search for start of a full temporal window
  const double window_end_ts = temporal_imu_window_[static_cast<size_t>(end_idx)].timestamp_;
  for (ssize_t idx = end_idx - 1; idx >= 0; --idx)
  {
    if (window_end_ts - temporal_imu_window_[static_cast<size_t>(idx)].timestamp_ >
        options_.temporal_window_length_sec_)
    {
      start_idx = idx + 1;
      break;
    }
  }
  if (start_idx == -1 || end_idx < start_idx)
    return IMUTemporalStatus::kUnkown;

  const size_t n = static_cast<size_t>(end_idx - start_idx + 1);
  std::vector<double> gyr_x(n), gyr_y(n), gyr_z(n), acc_x(n), acc_y(n), acc_z(n);
  for (ssize_t midx = start_idx; midx <= end_idx; ++midx)
  {
    const ImuMeasurement& m = temporal_imu_window_[static_cast<size_t>(midx)];
    const size_t off = static_cast<size_t>(midx - start_idx);
    gyr_x[off] = m.angular_velocity_.x();
    gyr_y[off] = m.angular_velocity_.y();
    gyr_z[off] = m.angular_velocity_.z();
    acc_x[off] = m.linear_acceleration_.x();
    acc_y[off] = m.linear_acceleration_.y();
    acc_z[off] = m.linear_acceleration_.z();
  }

  const double sqrt_dt = std::sqrt(1.0 / imu_calib_.imu_rate);
  std::array<double, 3> gyr_std = {stdVec(gyr_x) * sqrt_dt,
                                     stdVec(gyr_y) * sqrt_dt,
                                     stdVec(gyr_z) * sqrt_dt};
  std::array<double, 3> acc_std = {stdVec(acc_x) * sqrt_dt,
                                     stdVec(acc_y) * sqrt_dt,
                                     stdVec(acc_z) * sqrt_dt};

  bool stationary = true;
  for (size_t idx = 0; idx < 3; ++idx)
  {
    stationary &= (gyr_std[idx] < options_.stationary_gyr_sigma_thresh_);
    stationary &= (acc_std[idx] < options_.stationary_acc_sigma_thresh_);
  }

  // Prune: keep only measurements newer than the start of the window
  temporal_imu_window_.erase(
      temporal_imu_window_.begin(), temporal_imu_window_.begin() + static_cast<ssize_t>(start_idx));

  return stationary ? IMUTemporalStatus::kStationary : IMUTemporalStatus::kMoving;
}

// =============================================================================
// IMU ONLINE CALIBRATOR
// Uses visual rotation as ground truth to estimate gyro bias and T_cam_imu
// =============================================================================

ImuOnlineCalibrator::ImuOnlineCalibrator()
  : calibration_state_(CalibrationState::kWaitingForSecondFrame)
  , R_cam_imu_(Eigen::Matrix3d::Identity())
  , observations_()
  , R_imu_last_omega_(Eigen::Quaterniond::Identity())
  , last_obs_timestamp_(0.0)
  , first_obs_(true)
  , R_vis_last_(Eigen::Quaterniond::Identity())
  , first_visual_obs_(true)
  , first_omega_obs_(true)
  , last_omega_ts_(0.0)
{
}

void ImuOnlineCalibrator::observeVisualRotation(
    const Eigen::Quaterniond& R_world_last,
    const Eigen::Quaterniond& R_world_cur,
    double timestamp)
{
  // Update last visual reference
  if (first_visual_obs_)
  {
    R_vis_last_ = R_world_last;
    first_visual_obs_ = false;
    last_obs_timestamp_ = timestamp;
    return;
  }

  // Update visual reference
  Eigen::Quaterniond prev_vis = R_vis_last_;  // Save before overwriting
  R_vis_last_ = R_world_cur;

  // Get IMU delta from our rolling integration
  if (!first_obs_)
  {
    Observation obs;
    obs.timestamp = timestamp;
    obs.R_vis_last = prev_vis;
    obs.R_vis_cur = R_world_cur;
    obs.R_imu_delta = R_imu_last_omega_;
    obs.dt = timestamp - last_obs_timestamp_;

    observations_.push_back(obs);
    first_obs_ = false;
  }
  else
  {
    first_obs_ = false;
  }
  R_imu_last_omega_ = Eigen::Quaterniond::Identity();  // Reset for next inter-frame integration
  last_obs_timestamp_ = timestamp;

  // Remove old observations
  const double now = timestamp;
  observations_.erase(
      std::remove_if(observations_.begin(), observations_.end(),
          [now](const Observation& o) { return now - o.timestamp > kMaxObservationAge; }),
      observations_.end());

  // Transition state
  if (calibration_state_ == CalibrationState::kWaitingForSecondFrame)
  {
    if (observations_.size() >= 2)
      calibration_state_ = CalibrationState::kCollecting;
  }

  // Run estimation when we have enough data
  if (calibration_state_ == CalibrationState::kCollecting)
  {
    if (observations_.size() >= kMinObservations)
    {
      runBiasEstimation();
      runTcamImuEstimation();
      calibration_state_ = CalibrationState::kReady;
      SVO_WARN_STREAM("ImuOnlineCalibrator: calibration complete. "
          << "gyro_bias=[" << omega_bias_.transpose() << "] rad/s");
    }
  }
}

Eigen::Vector3d ImuOnlineCalibrator::correctOmega(const Eigen::Vector3d& omega_raw) const
{
  return omega_raw - omega_bias_;
}

void ImuOnlineCalibrator::feedOmegaMeasurement(double timestamp, const Eigen::Vector3d& omega_raw)
{
  if (first_omega_obs_)
  {
    first_omega_obs_ = false;
    last_omega_ts_ = timestamp;
    return;
  }

  double dt = timestamp - last_omega_ts_;
  last_omega_ts_ = timestamp;

  if (dt <= 0.0 || dt > 0.5)
    return;

  // Integrate with current bias estimate into rolling buffer
  Eigen::Vector3d omega = omega_raw - omega_bias_;
  const double wn = omega.norm();
  if (wn > 1e-8)
  {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(wn * dt, omega.normalized()));
    R_imu_last_omega_ = dq * R_imu_last_omega_;
  }
}

void ImuOnlineCalibrator::runBiasEstimation()
{
  // Gyro bias estimation via least-squares on axis-angle residuals.
  //
  // For each observation, the IMU predicts:
  //   R_imu_delta = exp((omega - omega_bias) * dt)
  //
  // The camera predicts:
  //   R_vis_delta = R_cam_imu * R_imu_delta * R_cam_imu^T
  //
  // With T_cam_imu fixed (identity initially), we solve for omega_bias by
  // minimizing the angular residual:
  //   residual = angle(R_vis_delta * R_cam_imu * R_imu_delta^T * R_cam_imu^T)
  //
  // For small biases, the angular error is approximately:
  //   error ≈ || (R_vis_delta * R_cam_imu) - I ||_so3^T * omega_bias * dt
  //
  // We use a simple gradient-descent approach with axis-angle representation.

  if (observations_.empty()) return;

  const Eigen::Matrix3d R_ci = R_cam_imu_;
  const double lr = 0.5;  // Learning rate
  const int max_iter = 50;
  Eigen::Vector3d bias = omega_bias_;

  for (int iter = 0; iter < max_iter; ++iter)
  {
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    double total_weight = 0.0;

    for (const Observation& obs : observations_)
    {
      // Apply current bias to IMU delta
      // We approximate: R_imu_corr = exp(-bias * dt) * R_imu_raw
      // For small bias, use first-order: delta_R_corr = delta_R_raw * (I - [bias*dt]_×)
      const double dt = obs.dt;
      Eigen::Matrix3d R_imu_raw = obs.R_imu_delta.toRotationMatrix();
      Eigen::Matrix3d R_vis_pred = R_ci * R_imu_raw * R_ci.transpose();
      Eigen::Quaterniond q_vis_pred(R_vis_pred);

      // Actual visual rotation
      Eigen::Quaterniond q_vis_actual = obs.R_vis_cur.conjugate() * obs.R_vis_last;
      Eigen::Matrix3d R_vis_actual = q_vis_actual.toRotationMatrix();

      // Residual: R_res = R_vis_actual * R_vis_pred^T
      Eigen::Matrix3d R_res = R_vis_actual * R_vis_pred.transpose();
      Eigen::AngleAxisd aa_res(R_res);
      double err_angle = aa_res.angle();
      if (err_angle > M_PI) err_angle -= 2 * M_PI;
      Eigen::Vector3d err_axis = aa_res.axis();
      if (err_angle < 0) err_axis = -err_axis;

      // Jacobian of angular error w.r.t. bias:
      // For small dt, d(angle)/d(bias) ≈ -dt * axis_component
      // Use absolute angle as scalar cost
      const double weight = 1.0;
      grad += weight * (-err_axis * dt) * err_angle;
      total_weight += weight;
    }

    if (total_weight < 1e-6) break;
    grad /= total_weight;

    // Gradient descent step with step size decay
    const double step_size = lr / (1.0 + iter * 0.1);
    bias -= step_size * grad;

    // Clamp bias to reasonable range (deg/s)
    const double max_bias = 0.1;  // 0.1 rad/s = ~5.7 deg/s
    for (int i = 0; i < 3; ++i)
      bias(i) = std::max(-max_bias, std::min(max_bias, bias(i)));

    omega_bias_ = bias;
  }
}

void ImuOnlineCalibrator::runTcamImuEstimation()
{
  // T_cam_imu estimation: find R_cam_imu such that
  //   R_cam_imu * R_imu_delta * R_cam_imu^T ≈ R_vis_delta
  //
  // This is an orthogonal Procrustes problem.
  // We collect accumulated rotations and find the best rotation matrix.

  if (observations_.size() < 3) return;

  // Build accumulation matrices for Procrustes
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  for (const Observation& obs : observations_)
  {
    Eigen::Matrix3d R_imu_delta = obs.R_imu_delta.toRotationMatrix();
    Eigen::Matrix3d R_vis_delta = (obs.R_vis_cur.conjugate() * obs.R_vis_last).toRotationMatrix();
    M += R_vis_delta * R_imu_delta.transpose();
  }

  // SVD: M = U * S * V^T
  // Optimal R = U * V^T (ensuring det(R) = +1)
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_opt = svd.matrixU() * svd.matrixV().transpose();

  // Ensure proper rotation (det = +1)
  if (R_opt.determinant() < 0)
  {
    Eigen::Matrix3d V_adj = svd.matrixV();
    V_adj.col(2) *= -1;
    R_opt = svd.matrixU() * V_adj.transpose();
  }

  // Only accept if the rotation is reasonable (no more than 90° from identity)
  Eigen::AngleAxisd aa(R_opt);
  double angle_deg = std::abs(aa.angle()) * 180.0 / M_PI;
  if (angle_deg < 90.0)
  {
    R_cam_imu_ = R_opt;
    SVO_WARN_STREAM("ImuOnlineCalibrator: T_cam_imu updated. "
        << "angle_from_identity=" << angle_deg << "deg");
  }
}

}  // namespace svo

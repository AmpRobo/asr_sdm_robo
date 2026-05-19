#ifndef SVO_IMU_TYPES_H_
#define SVO_IMU_TYPES_H_

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <svo/global.h>

namespace svo
{

/// IMU measurement: gyroscope + accelerometer data at a specific timestamp.
struct ImuMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp_ = 0.0;               // Unix timestamp in seconds
  Eigen::Vector3d angular_velocity_;       // Gyroscope (rad/s)
  Eigen::Vector3d linear_acceleration_;   // Accelerometer (m/s^2)

  ImuMeasurement() = default;
  ImuMeasurement(double t, const Eigen::Vector3d& omega, const Eigen::Vector3d& acc)
    : timestamp_(t), angular_velocity_(omega), linear_acceleration_(acc) {}
};

typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>> ImuMeasurements;

/// IMU noise and calibration parameters.
struct ImuCalibration
{
  double delay_imu_cam = 0.0;
  double max_imu_delta_t = 0.1;
  double saturation_accel_max = 200.0;
  double saturation_omega_max = 20.0;
  double gyro_noise_density = 1.0e-5;
  double acc_noise_density = 1.0e-4;
  double gyro_bias_random_walk_sigma = 1.0e-6;
  double acc_bias_random_walk_sigma = 1.0e-5;
  double gravity_magnitude = 9.81;
  double imu_rate = 200.0;
};

/// IMU initial state (biases, velocity).
struct ImuInitialization
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  double velocity_sigma = 0.1;
  double omega_bias_sigma = 0.01;
  double acc_bias_sigma = 0.1;
};

/// Temporal status of IMU measurements.
enum class IMUTemporalStatus { kStationary, kMoving, kUnkown };

/// Options for the IMU handler.
struct IMUHandlerOptions
{
  bool temporal_stationary_check = false;
  double temporal_window_length_sec_ = 0.5;
  double stationary_acc_sigma_thresh_ = 0.0;  //!< m/s^2 std dev; below = stationary
  double stationary_gyr_sigma_thresh_ = 0.0;  //!< rad/s std dev; below = stationary
  double zero_motion_window_sec = 0.3;         //!< window for zero-motion detection
};

/// Preintegrated IMU measurement between two camera timestamps.
class PreintegratedImuMeasurement
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegratedImuMeasurement() = default;
  PreintegratedImuMeasurement(const Eigen::Vector3d& omega_bias,
                             const Eigen::Vector3d& acc_bias,
                             double saturation_omega_max,
                             double saturation_accel_max)
    : omega_bias_(omega_bias)
    , acc_bias_(acc_bias)
    , saturation_omega_max_(saturation_omega_max)
    , saturation_accel_max_(saturation_accel_max)
  {}

  void addMeasurement(const ImuMeasurement& m);
  void addMeasurements(const ImuMeasurements& ms);

  Eigen::Vector3d omega_bias_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d delta_t_ij_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_v_ij_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond delta_R_ij_ = Eigen::Quaterniond::Identity();
  double dt_sum_ = 0.0;
  bool last_imu_measurement_set_ = false;
  ImuMeasurement last_imu_measurement;

private:
  double saturation_omega_max_ = 20.0;
  double saturation_accel_max_ = 200.0;
};

/// Tracks online calibration state for IMU preprocessing.
///
/// Uses visual odometry as ground truth to estimate:
/// 1. Gyroscope bias (drift correction)
/// 2. T_cam_imu rotation (external rotation calibration)
///
/// During the initialization window, we collect (visual_delta_R, imu_delta_R) pairs
/// and solve for bias + T_cam_imu via least squares.
class ImuOnlineCalibrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuOnlineCalibrator();

  /// Call when a new frame is processed with known visual rotation delta.
  /// The calibrator will internally estimate gyro bias and T_cam_imu.
  void observeVisualRotation(const Eigen::Quaterniond& R_world_last,
                              const Eigen::Quaterniond& R_world_cur,
                              double timestamp);

  /// Pre-correct a raw angular velocity measurement using current bias estimate.
  Eigen::Vector3d correctOmega(const Eigen::Vector3d& omega_raw) const;

  /// Feed raw angular velocity measurement to the calibrator for rolling integration.
  /// Integrates omega (with current bias estimate) into a quaternion delta between frames.
  void feedOmegaMeasurement(double timestamp, const Eigen::Vector3d& omega_raw);

  /// Get the current estimated gyro bias (rad/s).
  const Eigen::Vector3d& gyroBias() const { return omega_bias_; }

  /// Get the current T_cam_imu rotation estimate (R_cam_imu: IMU→camera).
  /// Returns identity until enough data is collected.
  const Eigen::Matrix3d& T_cam_imu_R() const { return R_cam_imu_; }

  /// Whether the calibrator has enough data to provide corrected IMU data.
  bool isReady() const { return calibration_state_ >= CalibrationState::kReady; }

  /// Whether we are still collecting initial data.
  bool isCalibrating() const { return calibration_state_ < CalibrationState::kReady; }

  /// Number of observation pairs collected so far.
  size_t numObservations() const { return observations_.size(); }

  enum class CalibrationState
  {
    kWaitingForSecondFrame,  // Need at least 2 visual frames
    kCollecting,             // Actively collecting (visual, imu) pairs
    kReady                   // Enough data, calibration complete
  };

  CalibrationState calibrationState() const { return calibration_state_; }

private:
  /// Run gyro bias estimation from collected (visual_delta, imu_delta) pairs.
  /// omega_corr = omega_raw - omega_bias
  /// R_imu_delta ≈ exp(omega_corr * dt)
  /// We minimize: angle(R_vis_delta * R_cam_imu * R_imu_delta * R_cam_imu^T) over omega_bias
  void runBiasEstimation();

  /// Run T_cam_imu estimation from collected pairs.
  /// We find R_cam_imu such that: R_cam_imu * R_imu_delta * R_cam_imu^T ≈ R_vis_delta
  void runTcamImuEstimation();

  CalibrationState calibration_state_ = CalibrationState::kWaitingForSecondFrame;

  /// Estimated gyro bias (rad/s), updated online.
  Eigen::Vector3d omega_bias_ = Eigen::Vector3d::Zero();

  /// Estimated camera-to-IMU rotation (R_cam_imu: maps IMU frame → camera frame).
  /// For camera-down: Rx(180°). Updated online if identity is wrong.
  Eigen::Matrix3d R_cam_imu_ = Eigen::Matrix3d::Identity();

  /// Minimum observations before we attempt estimation.
  static constexpr size_t kMinObservations = 5;
  /// Maximum age of an observation before it's discarded (s).
  static constexpr double kMaxObservationAge = 3.0;

  struct Observation
  {
    double timestamp;
    Eigen::Quaterniond R_vis_last;   // Visual rotation at last frame (world-from-last-cam)
    Eigen::Quaterniond R_vis_cur;    // Visual rotation at current frame (world-from-cur-cam)
    Eigen::Quaterniond R_imu_delta;  // IMU-integrated rotation from last to cur frame
    double dt;                       // Time span
  };
  std::vector<Observation> observations_;

  /// Rolling IMU buffer for current inter-frame integration (for visual alignment).
  Eigen::Quaterniond R_imu_last_omega_ = Eigen::Quaterniond::Identity();
  double last_obs_timestamp_ = 0.0;
  bool first_obs_ = true;

  /// Previous visual rotation for delta computation.
  Eigen::Quaterniond R_vis_last_ = Eigen::Quaterniond::Identity();
  bool first_visual_obs_ = true;

  /// State for omega measurement feeding (independent of visual pairing).
  bool first_omega_obs_ = true;
  double last_omega_ts_ = 0.0;
};

/// Handles IMU data buffering, bias correction, preintegration, and rotation priors.
class ImuHandler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuHandler(const ImuCalibration& imu_calib,
              const ImuInitialization& imu_init,
              const IMUHandlerOptions& options);
  ~ImuHandler();

  bool addImuMeasurement(const ImuMeasurement& m);
  bool getMeasurements(double old_cam_timestamp, double new_cam_timestamp,
                       bool delete_old_measurements, ImuMeasurements& measurements);
  bool getClosestMeasurement(double timestamp, ImuMeasurement& measurement) const;
  bool getRelativeRotationPrior(double old_cam_timestamp, double new_cam_timestamp,
                                bool delete_old_measurements,
                                Eigen::Quaterniond& R_oldimu_newimu);
  bool getInitialAttitude(double timestamp, Eigen::Quaterniond& R_imu_world) const;
  IMUTemporalStatus checkTemporalStatus(const double time_sec);
  void reset();

  /// Returns mean and standard deviation of linear acceleration over the past window_sec.
  /// Used for zero-motion detection: if accel_std < threshold, the sensor is stationary.
  /// Returns false if the buffer doesn't have enough data for the requested window.
  bool getWindowedAccelerometerStats(
      double window_sec,
      double& accel_mean_norm,    // mean acceleration magnitude (should be ~9.81 at rest)
      double& accel_std_norm,     // std dev of acceleration magnitude
      Eigen::Vector3d& accel_mean_vec) const;  // mean acceleration vector

  // Convenience method: get rotation prior transformed by current R_cam_imu estimate.
  // Returns R_cam_delta = R_cam_imu * R_imu_delta * R_cam_imu^T (camera-frame rotation).
  // Before calibration: returns R_imu_delta unchanged (R_cam_imu = I).
  bool getTransformedRotationPrior(
      double old_cam_timestamp, double new_cam_timestamp,
      Eigen::Quaterniond& R_cam_delta);

  const ImuCalibration& imu_calib_;
  const ImuInitialization& imu_init_;
  mutable Eigen::Vector3d acc_bias_;
  mutable Eigen::Vector3d omega_bias_;
  std::unique_ptr<ImuOnlineCalibrator> calibrator_;
  const IMUHandlerOptions options_;

  /// Integrate biased-corrected gyroscope readings between two timestamps.
  /// Returns R_imu_last_from_imu_cur (quaternion: IMU rotation from cur→last frame).
  /// Returns false if no IMU data is available in the interval.
  /// Uses online or static gyro bias for correction.
  bool getPoseIncrement(double last_timestamp, double cur_timestamp,
                        Eigen::Quaterniond& R_imu_last_from_imu_cur);

  /// Interpolate IMU state (omega + acc) at a specific timestamp.
  /// Returns false if no IMU data brackets the target timestamp.
  /// Uses linear interpolation between the two nearest IMU measurements.
  bool getPoseAt(double timestamp,
                 Eigen::Vector3d& omega_interp,
                 Eigen::Vector3d& acc_interp,
                 double& dt_last,
                 double& dt_cur) const;

  /// Online calibrator: uses visual rotation as reference to estimate gyro bias and T_cam_imu.

  /// Feed raw angular velocity to the calibrator for inter-frame delta integration.
  /// Called from addImuMeasurement.
  void feedOmegaToCalibrator(double timestamp, const Eigen::Vector3d& omega);

  /// Expose calibrator for visual to update with rotation data.
  ImuOnlineCalibrator* calibrator() { return calibrator_.get(); }

  /// Returns true only if the online calibrator has converged and is ready to use.
  /// Until ready, the IMU must NOT be used for pose prediction.
  bool isCalibrated() const { return calibrator_ && calibrator_->isReady(); }

  /// Get current gyro bias: from calibrator if ready, otherwise static bias.
  Eigen::Vector3d currentGyroBias() const;

private:
  mutable std::mutex measurements_mut_;
  ImuMeasurements measurements_;
  ImuMeasurements temporal_imu_window_;
  using ulock_t = std::unique_lock<std::mutex>;
};

typedef std::shared_ptr<ImuHandler> ImuHandlerPtr;

}  // namespace svo

#endif  // SVO_IMU_TYPES_H_

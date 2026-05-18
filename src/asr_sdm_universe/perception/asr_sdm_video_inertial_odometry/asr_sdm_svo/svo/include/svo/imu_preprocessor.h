#pragma once

#include <vector>
#include <Eigen/Dense>

namespace svo
{

struct ImuRawData
{
  double timestamp;
  Eigen::Vector3d omega;  // rad/s
  Eigen::Vector3d acc;    // m/s^2
};

struct ImuAlignedData
{
  double timestamp;
  Eigen::Vector3d omega;      // bias-corrected, rad/s
  Eigen::Vector3d acc;        // bias-corrected, m/s^2
  Eigen::Vector3d velocity;    // integrated velocity (world frame)
  Eigen::Vector3d position;    // integrated position (world frame)
  Eigen::Quaterniond orientation;  // integrated orientation (world frame)
};

struct ImuAlignmentResult
{
  Eigen::Vector3d gyro_bias;       // rad/s
  Eigen::Vector3d acc_bias;         // m/s^2
  Eigen::Matrix3d R_cam_imu;       // rotation from IMU to camera frame
  Eigen::Vector3d t_cam_imu;       // translation (typically zero for rotation-only)
  double scale;                     // scale factor for position alignment
  double gravity_magnitude;         // estimated gravity magnitude
  Eigen::Vector3d gravity_direction; // gravity direction in IMU frame
  bool valid;
  double residual_rotation_deg;     // alignment residual (degrees)
  double residual_position_m;       // alignment residual (meters)
  std::string message;
};

struct VisualFramePose
{
  double timestamp;
  Eigen::Vector3d position;    // camera position in world frame
  Eigen::Quaterniond orientation;  // camera orientation (world frame)
};

struct ImuPreprocessorOptions
{
  double gravity_magnitude = 9.81;
  int stationary_window_frames = 30;   // frames to detect stationary period
  double stationary_gyro_sigma_thresh = 0.005;  // rad/s
  double stationary_acc_sigma_thresh = 0.05;    // m/s^2
  int alignment_min_overlap_frames = 50;
  double max_alignment_time_sec = 30.0;
  int alignment_downsample_factor = 1;
  double robust_kernel_threshold = 0.1;  // Huber kernel threshold for outlier rejection
  bool use_huber = true;
  bool verbose = true;
};

class ImuPreprocessor
{
public:
  explicit ImuPreprocessor(const ImuPreprocessorOptions& options = ImuPreprocessorOptions());
  ~ImuPreprocessor() = default;

  void reset();

  // Feed raw IMU data
  void addImuData(double timestamp, const Eigen::Vector3d& omega, const Eigen::Vector3d& acc);

  // Feed visual pose (this becomes the "ground truth" trajectory)
  void addVisualPose(double timestamp, const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation);

  // Detect stationary period from visual motion (small displacement = stationary)
  bool detectStationaryPeriod(double& start_time, double& end_time,
                              const std::vector<VisualFramePose>& poses) const;

  // Step 1: Estimate gyro and accel bias from stationary period
  void estimateBias(const std::vector<ImuRawData>& imu_data,
                    double stationary_start, double stationary_end,
                    Eigen::Vector3d& gyro_bias, Eigen::Vector3d& acc_bias,
                    Eigen::Vector3d& gravity_direction);

  // Step 2: Integrate IMU trajectory (with bias correction)
  void integrateTrajectory(const std::vector<ImuRawData>& raw_imu,
                           const Eigen::Vector3d& gyro_bias,
                           const Eigen::Vector3d& acc_bias,
                           const Eigen::Vector3d& gravity_direction,
                           const Eigen::Vector3d& init_velocity,
                           const Eigen::Vector3d& init_position,
                           const Eigen::Quaterniond& init_orientation,
                           std::vector<ImuAlignedData>& aligned_trajectory);

  // Step 3: Align IMU trajectory to visual trajectory (6-DOF + scale)
  ImuAlignmentResult alignTrajectories(const std::vector<ImuAlignedData>& imu_trajectory,
                                        const std::vector<VisualFramePose>& visual_poses);

  // Full pipeline: process all data and return alignment result
  ImuAlignmentResult processAll();

  // Apply alignment: transform IMU pose to visual frame
  Eigen::Vector3d applyAlignmentPosition(const Eigen::Vector3d& imu_pos) const;
  Eigen::Quaterniond applyAlignmentOrientation(const Eigen::Quaterniond& imu_quat) const;

  // Interpolate IMU data to a specific timestamp (linear interpolation)
  bool interpolateImu(double timestamp, Eigen::Vector3d& omega, Eigen::Vector3d& acc) const;

  // Get accumulated pose at a specific timestamp (after full alignment)
  bool getAlignedPose(double timestamp,
                      Eigen::Vector3d& position,
                      Eigen::Quaterniond& orientation) const;

  // Predict pose using IMU between two visual frames (for frame interpolation)
  bool predictPoseBetweenFrames(double t0, double t1,
                                 const Eigen::Vector3d& pos0,
                                 const Eigen::Quaterniond& quat0,
                                 double t_predict,
                                 Eigen::Vector3d& pos_predict,
                                 Eigen::Quaterniond& quat_predict);

  // Save/load calibration results to/from YAML file
  bool saveToYaml(const std::string& filename) const;
  bool loadFromYaml(const std::string& filename);

  // Getters
  const std::vector<ImuRawData>& rawImuData() const { return raw_imu_; }
  const std::vector<VisualFramePose>& visualPoses() const { return visual_poses_; }
  const ImuAlignmentResult& alignmentResult() const { return alignment_result_; }
  bool hasAlignment() const { return alignment_result_.valid; }

private:
  ImuPreprocessorOptions options_;

  std::vector<ImuRawData> raw_imu_;
  std::vector<VisualFramePose> visual_poses_;

  ImuAlignmentResult alignment_result_;

  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d gravity_direction_;
  Eigen::Matrix3d R_cam_imu_;
  Eigen::Vector3d t_cam_imu_;
  double scale_;
  Eigen::Vector3d translation_;
  bool bias_estimated_;
  std::vector<ImuAlignedData> aligned_trajectory_;

  // 6-DOF + scale alignment using Umeyama algorithm (closed-form)
  bool alignUmeyama(const std::vector<Eigen::Vector3d>& P,
                    const std::vector<Eigen::Vector3d>& Q,
                    Eigen::Matrix3d& R,
                    Eigen::Vector3d& t,
                    double& s);

  // Huber robust kernel
  double huberWeight(double residual, double threshold);

  // Compute residual between IMU and visual trajectory
  double computeAlignmentResidual(const std::vector<ImuAlignedData>& imu_traj,
                                  const std::vector<VisualFramePose>& vis_poses,
                                  const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t,
                                  double s);
};

}  // namespace svo

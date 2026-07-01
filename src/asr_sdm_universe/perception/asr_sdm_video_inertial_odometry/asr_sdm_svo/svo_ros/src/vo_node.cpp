#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/imu_preprocessor.h>
#include <svo/imu_types.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo_ros/visualizer.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/math_utils.h>
#include <vikit/params_helper.h>
#include <vikit/pinhole_camera.h>
#include <vikit/user_input_thread.h>

#include <svo_vio_backend/vio_backend.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace svo
{

class VoNode : public rclcpp::Node
{
public:
  VoNode();
  ~VoNode();
  void init();
  void run();
  bool shouldQuit() const { return quit_; }

  // Getters needed by main()
  bool& quitFlag() { return quit_; }
  std::condition_variable& imgCv() { return img_cv_; }

private:
  // --- ROS callbacks ---
  void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void remoteKeyCb(const std_msgs::msg::String::ConstSharedPtr& msg);
  void processUserActions();

  // --- VINS-compatible feature cloud ---
  // Pack the SVO Frame's mature (TYPE_GOOD) 3D point observations into a
  // sensor_msgs/PointCloud that stock vins_estimator can consume directly.
  void publishVinsFeatureCloud(
      const svo::Frame& frame,
      const std_msgs::msg::Header& image_header);

  // --- Core ---
  svo::FrameHandlerMono* vo_ = nullptr;
  vk::AbstractCamera* cam_ = nullptr;
  std::unique_ptr<svo::Visualizer> visualizer_;
  std::shared_ptr<vk::UserInputThread> user_input_thread_;
  bool quit_ = false;

  // --- Image pipeline (producer-consumer, single-frame) ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_img_;
  double latest_img_rel_ts_ = 0.0;  // relative timestamp (bag_start_time_ = t=0)
  std::mutex img_mutex_;
  std::condition_variable img_cv_;
  bool new_img_ready_ = false;

  // --- Remote key ---
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::mutex remote_mutex_;
  std::string remote_input_;

  // --- IMU ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  std::shared_ptr<svo::ImuHandler> imu_handler_;
  bool use_imu_ = false;
  bool received_first_img_ = false;
  bool received_first_imu_ = false;
  svo::ImuCalibration imu_calib_;
  svo::ImuInitialization imu_init_;
  svo::IMUHandlerOptions imu_options_;

  // --- IMU yaw alignment (unify IMU world to SVO visual world) ---
  // NOTE: The final implementation uses gravity-aligned Roll/Pitch + yaw=0.
  // IMU quaternion yaw is NOT used — it drifts continuously (sensor fusion drift),
  // not a fixed offset. The visual front-end establishes yaw from image features.

  // IMU preprocessor (visual-guided offline calibration + data collection):
  //   0=none: no IMU preprocessing, use raw ImuHandler
  //   1=collect: collect visual poses + raw IMU for offline calibration
  int imu_preprocessing_mode_ = 0;
  std::shared_ptr<svo::ImuPreprocessor> imu_preprocessor_;

  // Use independent per-stream time origins instead of a shared bag origin.
  // Some recorded bags interleave IMU/image streams non-monotonically when replayed,
  // even though each individual stream is monotonic. Keeping separate relative
  // timelines avoids false "out-of-order" rejection while preserving per-stream
  // monotonicity required by SVO.
  double imu_start_time_ = -1.0;
  double img_start_time_ = -1.0;

  // Per-stream monotonic timestamp guards (raw absolute time, seconds).
  double last_imu_abs_ts_ = -1.0;
  double last_img_abs_ts_ = -1.0;

  // --- IMU health monitoring ---
  int imu_meas_count_ = 0;
  double last_imu_msg_ts_ = 0.0;
  bool imu_timeout_warned_ = false;
  static constexpr double IMU_TIMEOUT_SEC_ = 2.0;

  // --- Publish control ---
  bool publish_markers_ = true;
  bool publish_dense_input_ = false;
  bool enable_visualization_ = false;
  double last_marker_publish_ts_ = 0.0;

  // --- Frame throttling ---
  bool enable_frame_throttle_ = true;
  double target_fps_ = 15.0;
  double last_accepted_ts_ = 0.0;

  // --- Tight-coupled VIO backend (Phase 1 integration) ---
  // Runs in addition to the SVO frontend; receives every keyframe pose
  // and runs a Ceres-based IMU optimization on a 10-frame sliding window.
  std::unique_ptr<svo_vio_backend::VioBackend> vio_backend_;
  bool enable_vio_backend_ = false;          // global on/off
  bool vio_backend_publish_odometry_ = true; // publish /svo_vio/odometry
  double vio_backend_last_optimize_ts_ = 0.0;
  double vio_backend_optimize_period_ = 0.1; // throttle optimization to 10 Hz
  Eigen::Matrix3d vio_ric_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d vio_tic_ = Eigen::Vector3d::Zero();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  double last_vio_pose_ts_ = 0.0;

  // --- VINS-compatible feature publisher (Phase 2: replace svo_vio_backend) ---
  // Emits sensor_msgs/PointCloud on `/feature_tracker/feature` so that
  // a stock `vins_estimator` node can consume SVO-tracked features and
  // run the well-tested VINS sliding-window Ceres back-end.
  // Channel layout (matching VINS `feature_tracker_node`):
  //   points[i]      = (x, y, 1)   normalized camera plane coords (un_pts)
  //   channels[0][i] = id * NUM_OF_CAM + camera_id    (matches VINS encoding)
  //   channels[1][i] = u (pixel x)
  //   channels[2][i] = v (pixel y)
  //   channels[3][i] = velocity_x (we send 0.0; only used by td estimator)
  //   channels[4][i] = velocity_y (we send 0.0; only used by td estimator)
  bool publish_vins_features_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature_cloud_;
  static constexpr int NUM_OF_CAM_FOR_FEATURE_ = 1;  // mono
  uint64_t vins_feature_publish_count_ = 0;
};

// =============================================================================
// Constructor
// =============================================================================
VoNode::VoNode()
  : Node("asr_sdm_video_inertial_odometry")
{
  publish_markers_      = vk::getParam<bool>(this, "publish_markers", true);
  publish_dense_input_  = vk::getParam<bool>(this, "publish_dense_input", false);
  enable_visualization_  = vk::getParam<bool>(this, "enable_visualization", false);

  if (vk::getParam<bool>(this, "accept_console_user_input", true))
    user_input_thread_ = std::make_shared<vk::UserInputThread>();

  const std::string dataset_name = vk::getParam<std::string>(this, "dataset_name", "");

  if (dataset_name == "euroc")
  {
    RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC camera parameters");
    cam_ = new vk::PinholeCamera(752, 480, 458.654, 457.296, 367.215, 248.375);
  }
  else
  {
    const std::string cam_model = vk::getParam<std::string>(this, "cam_model", "Pinhole");
    bool ok = vk::camera_loader::loadFromRosNode(this, "", cam_);
    if (!ok || cam_ == nullptr)
      throw std::runtime_error("Failed to load camera. cam_model=" + cam_model);
    RCLCPP_INFO(this->get_logger(), "Created '%s' camera model", cam_model.c_str());
  }

  // --- Algorithm parameters ---
  if (dataset_name == "euroc")
  {
    RCLCPP_INFO(this->get_logger(), "Using hardcoded EuRoC algorithm parameters");
    svo::Config::gridSize() = 18;
    svo::Config::maxFts() = 400;
    svo::Config::triangMinCornerScore() = 3.0;
    svo::Config::nPyrLevels() = 4;
    svo::Config::fastType() = 12;
    svo::Config::kltMaxLevel() = 4;
    svo::Config::kltMinLevel() = 0;
    svo::Config::reprojThresh() = 4.0;
    svo::Config::poseOptimThresh() = 4.0;
    svo::Config::poseOptimNumIter() = 10;
    svo::Config::qualityMinFts() = 30;
    svo::Config::qualityMaxFtsDrop() = 80;
    svo::Config::initMinDisparity() = 50.0;
    svo::Config::initMinTracked() = 50;
    svo::Config::initMinInliers() = 40;
    svo::Config::kfSelectMinDist() = 0.001;
    svo::Config::maxNKfs() = 180;
    svo::Config::mapScale() = 5.0;
    svo::Config::useImu() = true;
    svo::Config::useThreadedDepthfilter() = false;
    svo::Config::patchMatchThresholdFactor() = 1.5;
    svo::Config::subpixNIter() = 20;
  }
  else
  {
    svo::Config::gridSize() = static_cast<size_t>(
        vk::getParam<int>(this, "grid_size", static_cast<int>(svo::Config::gridSize())));
    svo::Config::maxFts() = static_cast<size_t>(
        vk::getParam<int>(this, "max_fts", static_cast<int>(svo::Config::maxFts())));
    svo::Config::triangMinCornerScore() = vk::getParam<double>(
        this, "triang_min_corner_score", svo::Config::triangMinCornerScore());
    svo::Config::nPyrLevels() = static_cast<size_t>(
        vk::getParam<int>(this, "n_pyr_levels", static_cast<int>(svo::Config::nPyrLevels())));
    svo::Config::fastType() = static_cast<size_t>(
        vk::getParam<int>(this, "fast_type", static_cast<int>(svo::Config::fastType())));
    svo::Config::reprojThresh() = vk::getParam<double>(
        this, "reproj_thresh", svo::Config::reprojThresh());
    svo::Config::poseOptimThresh() = vk::getParam<double>(
        this, "poseoptim_thresh", svo::Config::poseOptimThresh());
    svo::Config::qualityMinFts() = static_cast<size_t>(vk::getParam<int>(
        this, "quality_min_fts", static_cast<int>(svo::Config::qualityMinFts())));
    svo::Config::qualityMaxFtsDrop() = vk::getParam<int>(
        this, "quality_max_drop_fts", svo::Config::qualityMaxFtsDrop());
    svo::Config::kfSelectMinDist() = vk::getParam<double>(
        this, "kfselect_mindist", svo::Config::kfSelectMinDist());
    svo::Config::maxNKfs() = static_cast<size_t>(vk::getParam<int>(
        this, "max_n_kfs", static_cast<int>(svo::Config::maxNKfs())));
    svo::Config::mapScale() = vk::getParam<double>(
        this, "map_scale", svo::Config::mapScale());
    svo::Config::initMinDisparity() = vk::getParam<double>(
        this, "init_min_disparity", svo::Config::initMinDisparity());
    svo::Config::initMinTracked() = static_cast<size_t>(vk::getParam<int>(
        this, "init_min_tracked", static_cast<int>(svo::Config::initMinTracked())));
    svo::Config::initMinInliers() = static_cast<size_t>(vk::getParam<int>(
        this, "init_min_inliers", static_cast<int>(svo::Config::initMinInliers())));
    svo::Config::patchMatchThresholdFactor() = vk::getParam<double>(
        this, "patch_match_thresh_factor", svo::Config::patchMatchThresholdFactor());
    svo::Config::kltMinLevel() = static_cast<size_t>(vk::getParam<int>(
        this, "klt_min_level", static_cast<int>(svo::Config::kltMinLevel())));
    svo::Config::kltMaxLevel() = static_cast<size_t>(vk::getParam<int>(
        this, "klt_max_level", static_cast<int>(svo::Config::kltMaxLevel())));
    svo::Config::subpixNIter() = static_cast<size_t>(vk::getParam<int>(
        this, "subpix_n_iter", static_cast<int>(svo::Config::subpixNIter())));
    svo::Config::maxEpiSearchSteps() = static_cast<size_t>(vk::getParam<int>(
        this, "max_epi_search_steps", static_cast<int>(svo::Config::maxEpiSearchSteps())));

    // === VIO Backend: Pose Optimization ===
    svo::Config::poseOptimNumIter() = static_cast<size_t>(vk::getParam<int>(
        this, "pose_optim_num_iter", static_cast<int>(svo::Config::poseOptimNumIter())));

    // === VIO Backend: Local Bundle Adjustment ===
    svo::Config::lobaNumIter() = static_cast<size_t>(vk::getParam<int>(
        this, "loba_num_iter", static_cast<int>(svo::Config::lobaNumIter())));
    svo::Config::coreNKfs() = static_cast<size_t>(vk::getParam<int>(
        this, "core_n_kfs", static_cast<int>(svo::Config::coreNKfs())));
    // Reproj threshold for local BA (higher than frontend for robustness)
    if (vk::getParam<bool>(this, "loba_override_thresh", false))
      svo::Config::lobaThresh() = vk::getParam<double>(this, "loba_thresh", svo::Config::lobaThresh());

    // === VIO Backend: Structure Optimization ===
    svo::Config::structureOptimNumIter() = static_cast<size_t>(vk::getParam<int>(
        this, "structure_optim_num_iter", static_cast<int>(svo::Config::structureOptimNumIter())));
    svo::Config::structureOptimMaxPts() = static_cast<size_t>(vk::getParam<int>(
        this, "structure_optim_max_pts", static_cast<int>(svo::Config::structureOptimMaxPts())));
  }

  // --- Frame throttling ---
  enable_frame_throttle_ = vk::getParam<bool>(this, "enable_frame_throttle", true);
  target_fps_ = vk::getParam<double>(this, "target_fps", 15.0);

  // --- IMU ---
  use_imu_ = vk::getParam<bool>(this, "use_imu", false);
  if (use_imu_)
  {
    RCLCPP_INFO(this->get_logger(), "IMU: ENABLED with IMU-prior VIO (SparseImgAlign weighted prior)");


    imu_calib_.delay_imu_cam = vk::getParam<double>(this, "imu_delay_imu_cam", 0.0);
    imu_calib_.max_imu_delta_t = vk::getParam<double>(this, "imu_max_imu_delta_t", 0.1);
    imu_calib_.saturation_accel_max = vk::getParam<double>(this, "imu_acc_max", 200.0);
    imu_calib_.saturation_omega_max = vk::getParam<double>(this, "imu_omega_max", 20.0);
    imu_calib_.gyro_noise_density = vk::getParam<double>(this, "imu_gyro_noise_density", 1.0e-4);
    imu_calib_.acc_noise_density = vk::getParam<double>(this, "imu_acc_noise_density", 1.0e-3);
    imu_calib_.gyro_bias_random_walk_sigma = vk::getParam<double>(this, "imu_gyro_bias_rw", 1.0e-6);
    imu_calib_.acc_bias_random_walk_sigma = vk::getParam<double>(this, "imu_acc_bias_rw", 1.0e-5);
    imu_calib_.gravity_magnitude = vk::getParam<double>(this, "imu_gravity_magnitude", 9.81);
    imu_calib_.imu_rate = vk::getParam<double>(this, "imu_rate", 200.0);

    imu_init_.velocity.setZero();
    imu_init_.omega_bias.setZero();
    imu_init_.acc_bias.setZero();
    imu_init_.velocity_sigma = vk::getParam<double>(this, "imu_velocity_sigma", 0.1);
    imu_init_.omega_bias_sigma = vk::getParam<double>(this, "imu_omega_bias_sigma", 0.01);
    imu_init_.acc_bias_sigma = vk::getParam<double>(this, "imu_acc_bias_sigma", 0.1);

    imu_options_.temporal_stationary_check = vk::getParam<bool>(
        this, "imu_temporal_stationary_check", false);
    imu_options_.temporal_window_length_sec_ = vk::getParam<double>(
        this, "imu_temporal_window_length_sec", 0.5);
    imu_options_.stationary_acc_sigma_thresh_ = vk::getParam<double>(
        this, "stationary_acc_sigma_thresh", 0.0);
    imu_options_.stationary_gyr_sigma_thresh_ = vk::getParam<double>(
        this, "stationary_gyr_sigma_thresh", 0.0);

    // NOTE: The final implementation uses gravity-aligned Roll/Pitch + yaw=0.
    // IMU quaternion yaw is NOT used — it drifts continuously (sensor fusion drift).
    // The visual front-end establishes yaw from image features.

    // IMU preprocessing mode:
    // 0 = no preprocessing (use raw ImuHandler directly)
    // 1 = collect (gather visual poses + IMU for offline calibration)
    // 2 = aligned (load calibration params, use aligned IMU for interpolation)
    imu_preprocessing_mode_ = vk::getParam<int>(this, "imu_preprocessing_mode", 0);

    if (imu_preprocessing_mode_ > 0)
    {
      svo::ImuPreprocessorOptions preproc_opts;
      preproc_opts.gravity_magnitude = imu_calib_.gravity_magnitude;
      preproc_opts.verbose = true;
      imu_preprocessor_ = std::make_shared<svo::ImuPreprocessor>(preproc_opts);
      RCLCPP_INFO(this->get_logger(), "IMU preprocessing mode=%d", imu_preprocessing_mode_);
    }

    imu_handler_ = std::make_shared<svo::ImuHandler>(imu_calib_, imu_init_, imu_options_);

    RCLCPP_INFO(this->get_logger(),
                "IMU: delay=%.3fs rate=%.0fHz gravity=%.2f",
                imu_calib_.delay_imu_cam, imu_calib_.imu_rate,
                imu_calib_.gravity_magnitude);
  }

  vo_ = new svo::FrameHandlerMono(cam_, use_imu_);
  if (use_imu_ && imu_handler_)
  {
    vo_->setImuHandler(imu_handler_.get());
    vo_->setImuCalibrator(imu_handler_->calibrator());
    vo_->setImgAlignPriorLambda(
        vk::getParam<double>(this, "img_align_prior_lambda_rot", 0.5),
        vk::getParam<double>(this, "img_align_prior_lambda_trans", 0.0));
    vo_->setZeroMotionParams(
        vk::getParam<double>(this, "zero_motion_accel_std_thresh", 0.05),
        vk::getParam<double>(this, "zero_motion_window_sec", 0.3));
    vo_->setPoseOptimPriorLambda(
        vk::getParam<double>(this, "pose_optim_prior_lambda_rot", 0.0),
        vk::getParam<double>(this, "pose_optim_prior_lambda_trans", 0.0));
    RCLCPP_INFO(this->get_logger(),
                "IMU prior λ_img_align=%.2f λ_pose_optim=%.2f zero_motion_accel_std=%.3f",
                vk::getParam<double>(this, "img_align_prior_lambda_rot", 0.5),
                vk::getParam<double>(this, "pose_optim_prior_lambda_rot", 0.0),
                vk::getParam<double>(this, "zero_motion_accel_std_thresh", 0.05));
  }

  // ---------------------------------------------------------------------
  // Tight-coupled VIO backend (Phase 1: VINS-style sliding-window Ceres
  // optimization layered on top of the SVO frontend).
  // ---------------------------------------------------------------------
  enable_vio_backend_ = vk::getParam<bool>(this, "enable_vio_backend", false);
  if (enable_vio_backend_)
  {
    vio_backend_ = std::make_unique<svo_vio_backend::VioBackend>();
    // IMU noise — defaults match the D435i BMI055 if not overridden.
    vio_backend_->setIMUNoise(
        vk::getParam<double>(this, "vio_acc_n",       imu_calib_.acc_noise_density),
        vk::getParam<double>(this, "vio_acc_w",       imu_calib_.acc_bias_random_walk_sigma),
        vk::getParam<double>(this, "vio_gyr_n",       imu_calib_.gyro_noise_density),
        vk::getParam<double>(this, "vio_gyr_w",       imu_calib_.gyro_bias_random_walk_sigma));
    // Gravity (VINS convention: world Z up, gravity points down +Z).
    vio_backend_->setGravity(Eigen::Vector3d(0.0, 0.0, imu_calib_.gravity_magnitude));
    // IMU-Camera extrinsics: identity by default (matches SVO's T_cam_imu=0).
    // vk::getParam doesn't support std::vector<double> well, so we use
    // node->declare_parameter + get_parameter directly.
    {
      const std::vector<double> ric_default = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      if (!this->has_parameter("vio_ric")) this->declare_parameter("vio_ric", ric_default);
      std::vector<double> ric_flat;
      if (this->get_parameter("vio_ric", ric_flat) && ric_flat.size() == 9) {
        vio_ric_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            ric_flat.data());
      } else {
        vio_ric_ = Eigen::Matrix3d::Identity();
      }
    }
    {
      const std::vector<double> tic_default = {0, 0, 0};
      if (!this->has_parameter("vio_tic")) this->declare_parameter("vio_tic", tic_default);
      std::vector<double> tic_flat;
      if (this->get_parameter("vio_tic", tic_flat) && tic_flat.size() == 3) {
        vio_tic_ = Eigen::Vector3d(tic_flat[0], tic_flat[1], tic_flat[2]);
      } else {
        vio_tic_ = Eigen::Vector3d::Zero();
      }
    }
    vio_backend_->setIMUExtrinsics(vio_ric_, vio_tic_);
    vio_backend_->setMaxIterations(vk::getParam<int>(this, "vio_max_iterations", 8));
    vio_backend_->setSolverTimeLimit(vk::getParam<double>(this, "vio_solver_time_limit", 0.04));
    vio_backend_optimize_period_ = vk::getParam<double>(this, "vio_optimize_period", 0.1);
    vio_backend_publish_odometry_ = vk::getParam<bool>(this, "vio_publish_odometry", true);
    RCLCPP_INFO(this->get_logger(),
                "VIO backend ENABLED: acc_n=%.4f gyr_n=%.4f acc_w=%.6f gyr_w=%.6f "
                "gravity=%.3f opt_period=%.3fs",
                imu_calib_.acc_noise_density, imu_calib_.gyro_noise_density,
                imu_calib_.acc_bias_random_walk_sigma, imu_calib_.gyro_bias_random_walk_sigma,
                imu_calib_.gravity_magnitude, vio_backend_optimize_period_);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "VIO backend DISABLED (set enable_vio_backend:=true to enable)");
  }
}

// =============================================================================
void VoNode::init()
{
  visualizer_ = std::make_unique<svo::Visualizer>(this->shared_from_this());

  visualizer_->T_world_from_vision_ = Sophus::SE3d(
      vk::rpy2dcm(Eigen::Vector3d(
          vk::getParam<double>(this, "init_rx", 0.0),
          vk::getParam<double>(this, "init_ry", 0.0),
          vk::getParam<double>(this, "init_rz", 0.0))),
      Eigen::Vector3d(
          vk::getParam<double>(this, "init_tx", 0.0),
          vk::getParam<double>(this, "init_ty", 0.0),
          vk::getParam<double>(this, "init_tz", 0.0)));

  sub_remote_key_ = create_subscription<std_msgs::msg::String>(
      "remote_key", 5,
      std::bind(&VoNode::remoteKeyCb, this, std::placeholders::_1));

  const std::string cam_topic = vk::getParam<std::string>(this, "cam_topic", "camera/image_raw");
  sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      cam_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VoNode::imgCb, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribing to camera: %s", cam_topic.c_str());

  if (use_imu_)
  {
    const std::string imu_topic = vk::getParam<std::string>(this, "imu_topic", "/imu/data");
    rclcpp::QoS qos_imu(100);
    qos_imu.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_imu.durability(rclcpp::DurabilityPolicy::Volatile);
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_imu,
        std::bind(&VoNode::imuCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());
  }

  // VIO backend tight-coupled odometry publisher
  if (enable_vio_backend_ && vio_backend_publish_odometry_)
  {
    const std::string odom_topic = vk::getParam<std::string>(
        this, "vio_odom_topic", "/svo_vio/odometry");
    pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing tight-coupled VIO odometry to: %s",
                odom_topic.c_str());
  }

  // --- VINS-compatible feature cloud publisher (Phase 2) ---
  // When enabled, the node will emit sensor_msgs/PointCloud on
  // `vins_feature_topic` for stock `vins_estimator` to consume. The
  // channel layout matches `feature_tracker_node` so the estimator
  // can drop-in use it.
  //
  // We read the bool via `has_parameter` + `get_parameter` directly
  // (NOT `vk::getParam<bool>`) because the param may have been
  // pre-declared by the YAML loader with a different type, and
  // LaunchConfiguration string→bool conversion can be brittle.
  if (this->has_parameter("publish_vins_features"))
  {
    this->get_parameter("publish_vins_features", publish_vins_features_);
  }
  else
  {
    publish_vins_features_ = vk::getParam<bool>(this, "publish_vins_features", false);
  }
  if (publish_vins_features_)
  {
    const std::string feature_topic = vk::getParam<std::string>(
        this, "vins_feature_topic", "/feature_tracker/feature");
    // VINS estimator subscribes with RELIABLE + KeepLast(2000). Match
    // exactly or messages will be dropped silently. SensorDataQoS is
    // best-effort — would trigger the QoS warning we just saw.
    rclcpp::QoS qos_feature(rclcpp::KeepLast(2000));
    qos_feature.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_feature.durability(rclcpp::DurabilityPolicy::Volatile);
    pub_feature_cloud_ = create_publisher<sensor_msgs::msg::PointCloud>(
        feature_topic, qos_feature);
    RCLCPP_INFO(this->get_logger(),
                "VINS-compatible feature cloud ENABLED on: %s (QoS: RELIABLE/KeepLast(2000))",
                feature_topic.c_str());
  }
}

// =============================================================================
// VINS-compatible feature cloud publisher
//
// Emits sensor_msgs/PointCloud with the exact channel layout that stock
// `vins_estimator` (`estimator_node.cpp::feature_callback`) expects:
//
//   points[i]      = (x, y, 1)   normalized camera plane coords (un_pts)
//   channels[0][i] = id * NUM_OF_CAM + camera_id  (VINS ID encoding)
//   channels[1][i] = u (pixel x)
//   channels[2][i] = v (pixel y)
//   channels[3][i] = velocity_x   (we send 0.0; only used by td estimator)
//   channels[4][i] = velocity_y   (we send 0.0; only used by td estimator)
//
// SVO `Feature::f` is the unit-bearing vector (f / f.z() gives normalized
// plane coords), and `Feature::point->id_` is a stable global ID across
// the whole SVO session. We only emit features whose 3D point has been
// successfully triangulated (Point::TYPE_GOOD) — these are the ones with
// reliable reprojection geometry that VINS sliding-window can triangulate
// stably from a second view.
//
// We also publish the FIRST frame with an empty cloud (VINS estimator
// skips the first message: it expects "no optical flow speed" on it).
// =============================================================================
void VoNode::publishVinsFeatureCloud(
    const svo::Frame& frame,
    const std_msgs::msg::Header& image_header)
{
  sensor_msgs::msg::PointCloud cloud;
  cloud.header = image_header;
  cloud.header.frame_id = "world";

  sensor_msgs::msg::ChannelFloat32 id_channel;
  sensor_msgs::msg::ChannelFloat32 u_channel;
  sensor_msgs::msg::ChannelFloat32 v_channel;
  sensor_msgs::msg::ChannelFloat32 vx_channel;
  sensor_msgs::msg::ChannelFloat32 vy_channel;
  id_channel.name  = "id";
  u_channel.name   = "u";
  v_channel.name   = "v";
  vx_channel.name  = "velocity_x";
  vy_channel.name  = "velocity_y";

  size_t emitted = 0;
  for (const svo::Feature* ftr : frame.fts_)
  {
    if (ftr == nullptr) continue;
    const svo::Point* pt = ftr->point;
    if (pt == nullptr) continue;             // not triangulated yet (seed)
    if (pt->type_ != svo::Point::TYPE_GOOD) continue;  // skip CANDIDATE/UNKNOWN

    // Normalized camera-plane coordinates: (x, y, 1) for VINS estimator.
    // SVO `f` is the unit-bearing vector: divide by f.z() to get (x, y, 1).
    const Eigen::Vector3d& f = ftr->f;
    const double fz = f.z();
    if (std::abs(fz) < 1e-6) continue;        // degenerate bearing vector

    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(f.x() / fz);
    p.y = static_cast<float>(f.y() / fz);
    p.z = 1.0f;
    cloud.points.push_back(p);

    const int id_with_cam = pt->id_ * NUM_OF_CAM_FOR_FEATURE_ + 0; // mono
    id_channel.values.push_back(static_cast<float>(id_with_cam));
    u_channel.values.push_back(static_cast<float>(ftr->px.x()));
    v_channel.values.push_back(static_cast<float>(ftr->px.y()));
    vx_channel.values.push_back(0.0f);   // SVO does not expose LK velocity
    vy_channel.values.push_back(0.0f);
    ++emitted;
  }

  // VINS estimator skips its first message via the `init_feature` flag,
  // so we MUST publish something every frame — even an empty cloud on
  // the first frame is enough to advance that flag.
  if (cloud.points.empty() && vins_feature_publish_count_ == 0)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[VINS features] first frame: emitting empty cloud to advance "
        "vins_estimator `init_feature` flag");
  }

  cloud.channels.push_back(id_channel);
  cloud.channels.push_back(u_channel);
  cloud.channels.push_back(v_channel);
  cloud.channels.push_back(vx_channel);
  cloud.channels.push_back(vy_channel);

  pub_feature_cloud_->publish(cloud);
  ++vins_feature_publish_count_;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[VINS features] frame_id=%d n_fts=%zu (mature=%zu) cloud=%zux5",
      frame.id_, frame.fts_.size(), emitted, cloud.points.size());
}

// =============================================================================
VoNode::~VoNode()
{
  quit_ = true;
  img_cv_.notify_all();
  sub_img_.reset();
  sub_imu_.reset();
  sub_remote_key_.reset();
  visualizer_.reset();
  delete vo_;
  delete cam_;
  if (user_input_thread_ != nullptr)
    user_input_thread_->stop();
}

// =============================================================================
// IMU callback (producer thread — ROS callback executor)
// =============================================================================
void VoNode::imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  // Convert raw IMU timestamp to a per-stream relative timeline.
  double t_abs = rclcpp::Time(msg->header.stamp).seconds();
  if (imu_start_time_ < 0.0)
  {
    imu_start_time_ = t_abs;
    RCLCPP_INFO(this->get_logger(), "IMU baseline set: first IMU ts=%.3f (wall=%.3f)", t_abs, this->get_clock()->now().seconds());
  }

  // Drop out-of-order IMU messages before they poison interpolation/integration.
  if (last_imu_abs_ts_ >= 0.0 && t_abs <= last_imu_abs_ts_)
  {
    const double dt_back_ms = (last_imu_abs_ts_ - t_abs) * 1000.0;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dropping out-of-order IMU: raw ts moved backward by %.1f ms", dt_back_ms);
    return;
  }
  last_imu_abs_ts_ = t_abs;

  const double t = t_abs - imu_start_time_;

  ++imu_meas_count_;

  if (imu_meas_count_ <= 5)
  {
    if (!received_first_imu_)
    {
      RCLCPP_INFO(this->get_logger(), "First IMU received; VIO input stream is now live");
      received_first_imu_ = true;
    }
    RCLCPP_INFO(this->get_logger(),
        "IMU callback #%d: ts=%.6f (raw: %u.%u) wclk=%.3f",
        imu_meas_count_, t,
        msg->header.stamp.sec, msg->header.stamp.nanosec,
        this->get_clock()->now().seconds());
  }

  if (last_imu_msg_ts_ > 0.0)
  {
    const double dt = t - last_imu_msg_ts_;
    if (dt > 1.0 / imu_calib_.imu_rate * 100.0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "IMU gap: %.1f ms", dt * 1000.0);
    }
  }
  last_imu_msg_ts_ = t;

  if (imu_timeout_warned_)
  {
    RCLCPP_INFO(this->get_logger(), "IMU data resumed");
    imu_timeout_warned_ = false;
  }

  const double omega_norm = std::sqrt(
      msg->angular_velocity.x * msg->angular_velocity.x +
      msg->angular_velocity.y * msg->angular_velocity.y +
      msg->angular_velocity.z * msg->angular_velocity.z);
  const double acc_norm = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z);

  if (omega_norm > imu_calib_.saturation_omega_max ||
      acc_norm > imu_calib_.saturation_accel_max)
    return;

  imu_handler_->addImuMeasurement(
      svo::ImuMeasurement(t,
          Eigen::Vector3d(msg->angular_velocity.x,
                           msg->angular_velocity.y,
                           msg->angular_velocity.z),
          Eigen::Vector3d(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z)));

  // Mirror the same IMU sample into the VIO backend's pre-integrator
  // (tight-coupled Ceres sliding window). The VioBackend keeps its own
  // independent buffer + preintegration; it does not block the SVO
  // frontend's IMU prior path.
  if (vio_backend_)
  {
    vio_backend_->addIMUMeasurement(
        t,
        Eigen::Vector3d(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z),
        Eigen::Vector3d(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z));
  }

  // Feed raw IMU to preprocessor for data collection (mode 1)
  if (imu_preprocessing_mode_ == 1 && imu_preprocessor_)
  {
    imu_preprocessor_->addImuData(
        t,
        Eigen::Vector3d(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z),
        Eigen::Vector3d(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z));
  }
}

// =============================================================================
// Image callback (producer — throttles, then signals consumer)
// =============================================================================
void VoNode::imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // Convert image timestamp to a per-stream relative timeline.
  double ts_abs = rclcpp::Time(msg->header.stamp).seconds();
  if (img_start_time_ < 0.0)
  {
    img_start_time_ = ts_abs;
    RCLCPP_INFO(this->get_logger(), "IMG baseline set: first IMG ts=%.3f (wall=%.3f)", ts_abs, this->get_clock()->now().seconds());
  }
  if (!received_first_img_)
  {
    RCLCPP_INFO(this->get_logger(), "First image received; starting SVO processing");
    received_first_img_ = true;
    if (vo_)
      vo_->start();
  }

  // Drop out-of-order images for the same reason as IMU: SVO assumes monotonic input.
  if (last_img_abs_ts_ >= 0.0 && ts_abs <= last_img_abs_ts_)
  {
    const double dt_back_ms = (last_img_abs_ts_ - ts_abs) * 1000.0;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dropping out-of-order image: raw ts moved backward by %.1f ms", dt_back_ms);
    return;
  }
  last_img_abs_ts_ = ts_abs;

  const double timestamp = ts_abs - img_start_time_;

  if (enable_frame_throttle_ && target_fps_ > 0.0)
  {
    const double min_dt = 1.0 / target_fps_;
    if (timestamp - last_accepted_ts_ < min_dt)
      return;
  }
  last_accepted_ts_ = timestamp;

  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    latest_img_ = msg;
    latest_img_rel_ts_ = timestamp;
    new_img_ready_ = true;
  }
  img_cv_.notify_one();
}

// =============================================================================
// Main processing loop (consumer — waits on condition variable)
// =============================================================================
void VoNode::run()
{
  RCLCPP_INFO(this->get_logger(), "SVO processing loop started");

  if (use_imu_ && imu_handler_)
  {
    RCLCPP_INFO(this->get_logger(), "IMU fusion enabled; waiting for bag topics to appear...");
  }

  double last_imu_status_time = 0.0;
  while (rclcpp::ok() && !quit_)
  {
    sensor_msgs::msg::Image::ConstSharedPtr img;
    double timestamp = 0.0;

    // Wait for next image
    {
      std::unique_lock<std::mutex> lock(img_mutex_);
      img_cv_.wait_for(lock, std::chrono::milliseconds(100),
                       [this]() { return new_img_ready_ || quit_; });
      if (quit_ || !new_img_ready_)
        continue;
      img = latest_img_;
      // Use the relative timestamp computed in imgCb (bag absolute → relative)
      timestamp = latest_img_rel_ts_;
      new_img_ready_ = false;
    }

    processUserActions();
    if (quit_)
      break;

    // Heartbeat: every 5 seconds, print IMU data status + prior state
    double now = this->get_clock()->now().seconds();
    if (now - last_imu_status_time > 5.0)
    {
      last_imu_status_time = now;
      bool imu_calib_ready = (imu_handler_ && imu_handler_->isCalibrated());
      size_t n_obs = 0;
      int calib_state = -1;
      Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
      if (imu_handler_ && imu_handler_->calibrator())
      {
        n_obs = imu_handler_->calibrator()->numObservations();
        calib_state = static_cast<int>(imu_handler_->calibrator()->calibrationState());
        gyro_bias = imu_handler_->calibrator()->gyroBias();
      }
      RCLCPP_INFO(this->get_logger(),
          "[HEARTBEAT] wall=%.3f imu_ts=%.3f imu_cnt=%d "
          "calib_ready=%d calib_state=%d n_obs=%zu gyro_bias=[%.4f %.4f %.4f] stationary=%d",
          now, last_imu_msg_ts_, imu_meas_count_,
          imu_calib_ready, calib_state, n_obs,
          gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
          vo_->isStationary());
    }

    // --- Image conversion ---
    cv::Mat img_cv;
    try
    {
      img_cv = cv_bridge::toCvShare(img, "mono8")->image;
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      continue;
    }

    // --- SVO ---
    vo_->addImage(img_cv, timestamp);

    // --- IMU preprocessor: mode 1 = collect visual poses for calibration ---
    if (imu_preprocessing_mode_ == 1 && imu_preprocessor_ && vo_->hasStarted())
    {
      auto last = vo_->lastFrame();
      if (last && last->isKeyframe())
      {
        imu_preprocessor_->addVisualPose(
            last->timestamp_,
            last->T_f_w_.translation(),
            Eigen::Quaterniond(last->T_f_w_.rotationMatrix()));
      }
    }

    // --- VINS-compatible feature cloud (Phase 2) ---
    // Publish SVO-tracked features to a stock vins_estimator.
    if (publish_vins_features_ && pub_feature_cloud_)
    {
      auto last = vo_->lastFrame();
      if (last)
      {
        try
        {
          publishVinsFeatureCloud(*last, img->header);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "[VINS features] EXCEPTION: %s", e.what());
        }
      }
    }

    // --- Tight-coupled VIO backend (Phase 1) ---
    // Feed the most recent frame pose to the sliding-window optimizer
    // whenever SVO marks a new keyframe, then run the IMU Ceres solve
    // (throttled) and republish the body-in-world pose.
    if (vio_backend_)
    {
      auto last = vo_->lastFrame();
      if (last && (vo_->stage() == svo::FrameHandlerBase::STAGE_DEFAULT_FRAME ||
                   vo_->stage() == svo::FrameHandlerBase::STAGE_SECOND_FRAME))
      {
        try
        {
        // T_f_w_ = transform from world to frame (= inverse of body-in-world)
        // Pass R, p of T_w_b to the backend.
        Eigen::Matrix3d R_w_b = last->T_f_w_.rotationMatrix().transpose();
        Eigen::Vector3d p_w_b = -R_w_b * last->T_f_w_.translation();

        if (last->isKeyframe())
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "[VIO backend] addKeyFrame ts=%.3f window=%d", timestamp,
              vio_backend_->getWindowFrameCount());
          // NOTE: We do NOT pass the raw Frame pointer to the VioBackend
          // because `last` is a temporary boost::shared_ptr<Frame> alias
          // whose target is owned by the SVO map. The VioBackend does not
          // dereference the pointer (only R, p, timestamp are used), so
          // passing nullptr is safe.
          vio_backend_->addKeyFrame(nullptr,
                                    last->timestamp_, R_w_b, p_w_b);

          // Throttle optimization to keep frontend responsive
          if (timestamp - vio_backend_last_optimize_ts_ > vio_backend_optimize_period_)
          {
            const auto t0 = std::chrono::steady_clock::now();
            const bool ok = vio_backend_->optimize();
            const auto t1 = std::chrono::steady_clock::now();
            const double dt_ms =
                std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (ok)
            {
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                  "[VIO backend] optimize OK | window=%d | dt=%.1fms",
                  vio_backend_->getWindowFrameCount(), dt_ms);
            }
            else
            {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                  "[VIO backend] optimize returned false | window=%d",
                  vio_backend_->getWindowFrameCount());
            }
            vio_backend_last_optimize_ts_ = timestamp;
          }
        }

        // Always publish current body-in-world pose (use the SVO pose
        // directly; once the window is full the optimized pose converges
        // to the same trajectory but is more drift-free).
        Eigen::Matrix4d T_w_b;
        if (vio_backend_->getCurrentPose(T_w_b) && pub_odometry_)
        {
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = rclcpp::Time(timestamp);
          odom.header.frame_id = "world";
          odom.child_frame_id = "body";
          odom.pose.pose.position.x = T_w_b(0, 3);
          odom.pose.pose.position.y = T_w_b(1, 3);
          odom.pose.pose.position.z = T_w_b(2, 3);
          Eigen::Quaterniond q(T_w_b.block<3, 3>(0, 0));
          q.normalize();
          odom.pose.pose.orientation.x = q.x();
          odom.pose.pose.orientation.y = q.y();
          odom.pose.pose.orientation.z = q.z();
          odom.pose.pose.orientation.w = q.w();
          // Identity covariance: backend does not yet estimate this
          for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
          odom.pose.covariance[0]  = 0.01;
          odom.pose.covariance[7]  = 0.01;
          odom.pose.covariance[14] = 0.01;
          odom.pose.covariance[21] = 0.05;
          odom.pose.covariance[28] = 0.05;
          odom.pose.covariance[35] = 0.05;
          pub_odometry_->publish(odom);
          last_vio_pose_ts_ = timestamp;
        }
        }  // try
        catch (const std::exception& e)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "[VIO backend] EXCEPTION: %s", e.what());
        }
        catch (...)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "[VIO backend] UNKNOWN EXCEPTION");
        }
      }
    }

    // --- Visualization ---
    if (enable_visualization_)
    {
      visualizer_->publishMinimal(img_cv, vo_->lastFrame(), *vo_, timestamp);

      if (publish_markers_ &&
          vo_->stage() != svo::FrameHandlerBase::STAGE_PAUSED &&
          timestamp - last_marker_publish_ts_ > 0.2)
      {
        visualizer_->visualizeMarkers(
            vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
        last_marker_publish_ts_ = timestamp;
      }

      if (publish_dense_input_)
        visualizer_->exportToDense(vo_->lastFrame());
    }

    if (vo_->stage() == svo::FrameHandlerMono::STAGE_PAUSED)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "SVO processing loop finished");
}

// =============================================================================
void VoNode::processUserActions()
{
  char input = 0;
  {
    std::lock_guard<std::mutex> lock(remote_mutex_);
    if (!remote_input_.empty())
    {
      input = remote_input_[0];
      remote_input_.clear();
    }
  }

  if (user_input_thread_ != nullptr)
  {
    const char c = user_input_thread_->getInput();
    if (c != 0)
      input = c;
  }

  switch (input)
  {
    case 'q':
      quit_ = true;
      break;
    case 'r':
      vo_->reset();
      visualizer_->resetTrajectory();
      if (imu_handler_)
      {
        imu_handler_->reset();
      }
      if (imu_preprocessor_)
      {
        imu_preprocessor_->reset();
      }
      break;
    case 's':
      vo_->start();
      break;
    case 'c':
      // Run offline calibration from collected data (mode 1)
      if (imu_preprocessing_mode_ == 1 && imu_preprocessor_)
      {
        RCLCPP_INFO(this->get_logger(), "Running offline IMU calibration...");
        auto result = imu_preprocessor_->processAll();
        if (result.valid)
        {
          RCLCPP_INFO(this->get_logger(),
              "Calibration SUCCESS: gyro_bias=[%.4f,%.4f,%.4f] "
              "acc_bias=[%.4f,%.4f,%.4f] scale=%.4f "
              "pos_residual=%.4fm rot_angle=%.2fdeg",
              result.gyro_bias.x(), result.gyro_bias.y(), result.gyro_bias.z(),
              result.acc_bias.x(), result.acc_bias.y(), result.acc_bias.z(),
              result.scale, result.residual_position_m, result.residual_rotation_deg);
          // Auto-save to default calibration file
          const std::string save_path = "imu_calibration.yaml";
          if (imu_preprocessor_->saveToYaml(save_path))
          {
            RCLCPP_INFO(this->get_logger(),
                "Calibration auto-saved to: %s (press 'l' to reload)", save_path.c_str());
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Calibration FAILED: %s", result.message.c_str());
        }
      }
      else if (imu_preprocessing_mode_ != 1)
      {
        RCLCPP_WARN(this->get_logger(),
            "Calibration ('c') only works in mode 1 (imu_preprocessing_mode:=1)");
      }
      break;
    case 'l':
      // Load calibration from file
      {
        const std::string calib_file = vk::getParam<std::string>(
            this, "imu_calib_file", "");
        if (!calib_file.empty() && imu_preprocessor_)
        {
          RCLCPP_INFO(this->get_logger(), "Loading IMU calibration from: %s", calib_file.c_str());
          if (imu_preprocessor_->loadFromYaml(calib_file))
          {
            RCLCPP_INFO(this->get_logger(), "Calibration loaded successfully");
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration file: %s", calib_file.c_str());
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "No calibration file specified (imu_calib_file)");
        }
      }
      break;
  }
}

void VoNode::remoteKeyCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(remote_mutex_);
  remote_input_ = msg->data;
}

}  // namespace svo

// =============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<svo::VoNode>();
  node->init();

  RCLCPP_INFO(node->get_logger(), "SVO node ready");

  // IMU and remote-key callbacks run in the ROS executor (MultiThreadedExecutor).
  // The SVO processing loop runs in its own thread and waits on a condition
  // variable for each new image. This keeps IMU callbacks non-blocking.
  std::thread svo_thread([&node]() { node->run(); });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  node->quitFlag() = true;
  node->imgCv().notify_all();

  if (svo_thread.joinable())
    svo_thread.join();

  RCLCPP_INFO(node->get_logger(), "SVO shut down");
  rclcpp::shutdown();
  return 0;
}

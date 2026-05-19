#ifndef SVO_FRAME_HANDLER_H_
#define SVO_FRAME_HANDLER_H_

#include <svo/frame_handler_base.h>
#include <svo/imu_types.h>
#include <svo/initialization.h>
#include <svo/reprojector.h>
#include <vikit/abstract_camera.h>

#include <set>

namespace svo
{


class FrameHandlerMono : public FrameHandlerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameHandlerMono(vk::AbstractCamera * cam, bool use_imu = false);
  virtual ~FrameHandlerMono();

  /// Set whether IMU rotation prior is enabled.
  void setUseImu(bool use_imu) { use_imu_ = use_imu; }

  /// Provide an image.
  void addImage(const cv::Mat & img, double timestamp);

  /// Set the first frame (used for synthetic datasets in benchmark node)
  void setFirstFrame(const FramePtr & first_frame);

  /// Get the last frame that has been processed.
  FramePtr lastFrame() { return last_frame_; }

  /// Get the set of spatially closest keyframes of the last frame.
  const std::set<FramePtr> & coreKeyframes() { return core_kfs_; }

  /// Return the feature track to visualize the KLT tracking during initialization.
  const std::vector<cv::Point2f> & initFeatureTrackRefPx() const
  {
    return klt_homography_init_.px_ref_;
  }
  const std::vector<cv::Point2f> & initFeatureTrackCurPx() const
  {
    return klt_homography_init_.px_cur_;
  }

  /// Access the depth filter.
  DepthFilter * depthFilter() const { return depth_filter_; }

  /// An external place recognition module may know where to relocalize.
  bool relocalizeFrameAtPose(
    const int keyframe_id, const Sophus::SE3d & T_kf_f, const cv::Mat & img,
    const double timestamp);

  // -------------------------------------------------------------------------
  // IMU Support: inter-frame pose prediction + timestamp interpolation
  // (NO rotation prior — rotation prior is permanently disabled in SVO)
  // -------------------------------------------------------------------------
  /// Set IMU handler for inter-frame pose prediction.
  /// Must be called before addImage if use_imu_ is true.
  void setImuHandler(ImuHandler* handler) { imu_handler_ = handler; }

  /// Set IMU online calibrator for visual-guided IMU preprocessing.
  void setImuCalibrator(ImuOnlineCalibrator* calib) { imu_calibrator_ = calib; }

  /// Set IMU prior weights for SparseImgAlign.
  /// Follows rpg/vio_mono.yaml convention: lambda_rot=0.5, lambda_trans=0.0.
  void setImgAlignPriorLambda(double lambda_rot, double lambda_trans)
  {
    img_align_prior_lambda_rot_ = lambda_rot;
    img_align_prior_lambda_trans_ = lambda_trans;
  }

  /// Set IMU prior weights for pose optimizer (bundle adjustment stage).
  /// Follows rpg/imufusion convention: lambda > 0 constrains rotation toward IMU.
  void setPoseOptimPriorLambda(double lambda_rot, double lambda_trans = 0.0)
  {
    pose_optim_prior_lambda_rot_ = lambda_rot;
    pose_optim_prior_lambda_trans_ = lambda_trans;
  }

  /// Set zero-motion detection thresholds.
  /// accel_std_thresh: m/s²; if accel std < this, robot is stationary.
  /// window_sec: look-back window for accelerometer statistics.
  void setZeroMotionParams(double accel_std_thresh, double window_sec)
  {
    zero_motion_accel_std_thresh_ = accel_std_thresh;
    zero_motion_window_sec_ = window_sec;
  }

  /// Returns true if the robot is currently detected as stationary.
  bool isStationary() const { return is_stationary_; }

protected:
  vk::AbstractCamera * cam_;     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).
  Reprojector reprojector_;      //!< Projects points from other keyframes into the current frame
  FramePtr new_frame_;           //!< Current frame.
  FramePtr last_frame_;          //!< Last frame, not necessarily a keyframe.
  std::set<FramePtr> core_kfs_;  //!< Keyframes in the closer neighbourhood.
  std::vector<std::pair<FramePtr, size_t> >
    overlap_kfs_;  //!< All keyframes with overlapping field of view. the paired number specifies
                   //!< how many common mappoints are observed TODO: why vector!?
  initialization::KltHomographyInit
    klt_homography_init_;  //!< Used to estimate pose of the first two keyframes by estimating a
                           //!< homography.
  DepthFilter * depth_filter_;  //!< Depth estimation algorithm runs in a parallel thread and is
                               //!< used to initialize new 3D points.

  // IMU: inter-frame pose prediction + IMU prior for SparseImgAlign
  bool use_imu_ = false;              //!< Whether IMU data is available
  ImuHandler* imu_handler_ = nullptr;  //!< IMU handler for inter-frame integration
  ImuOnlineCalibrator* imu_calibrator_ = nullptr;  //!< IMU online calibrator
  double last_imu_timestamp_ = 0.0;     //!< Last IMU timestamp for inter-frame integration
  Quaterniond last_optimized_quat_;     //!< Visual-optimized rotation from last frame
  bool is_stationary_ = false;        //!< Zero-motion detection via accel variance
  double zero_motion_accel_std_thresh_ = 0.05;  //!< m/s²; accel std < this = stationary
  double zero_motion_window_sec_ = 0.3;  //!< Window for zero-motion accel stats

  /// IMU prior weights for SparseImgAlign (pyramid direct alignment).
  double img_align_prior_lambda_rot_ = 0.5;
  double img_align_prior_lambda_trans_ = 0.0;

  /// IMU prior weight for pose optimizer (bundle adjustment over reprojected points).
  /// Follows rpg/imufusion convention: lambda > 0 → constrain rotation toward IMU.
  double pose_optim_prior_lambda_rot_ = 0.0;
  double pose_optim_prior_lambda_trans_ = 0.0;

  /// Notify that a frame was successfully optimized (called from pose optimizer).
  void setLastOptimizedRotation(const Quaterniond& q) { last_optimized_quat_ = q; }

  /// Initialize the visual odometry algorithm.
  virtual void initialize();

  /// Processes the first frame and sets it as a keyframe.
  virtual UpdateResult processFirstFrame();

  /// Processes all frames after the first frame until a keyframe is selected.
  virtual UpdateResult processSecondFrame();

  /// Processes all frames after the first two keyframes.
  virtual UpdateResult processFrame();

  /// Try relocalizing the frame at relative position to provided keyframe.
  virtual UpdateResult relocalizeFrame(const Sophus::SE3d & T_cur_ref, FramePtr ref_keyframe);

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll();

  /// Keyframe selection criterion.
  virtual bool needNewKf(double scene_depth_mean);

  void setCoreKfs(size_t n_closest);

  /// Compute IMU motion prior via gyro integration (overrides base class).
  virtual void getMotionPrior() override;
};

}  

#endif  

// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/**
 * @file frame_handler_mono.cpp
 * @brief Monocular visual odometry frame handler.
 * 
 * This is the main entry point for monocular SVO. It orchestrates the
 * complete visual odometry pipeline:
 * 
 * 1. FIRST_FRAME: Initialize with first keyframe
 * 2. SECOND_FRAME: KLT-based initialization, triangulate initial map
 * 3. DEFAULT_FRAME: Normal tracking mode
 *    a. Sparse image alignment (direct method)
 *    b. Map reprojection and feature alignment
 *    c. Pose optimization
 *    d. Structure optimization
 *    e. Keyframe selection
 *    f. Optional local bundle adjustment
 * 4. RELOCALIZING: Recovery from tracking failure
 * 
 * The handler also manages:
 * - Depth filter for probabilistic depth estimation
 * - Keyframe selection and removal
 * - Core keyframe set for local BA
 */

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <svo/depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

/**
 * @brief Constructs monocular frame handler with camera model.
 *
 * @param cam Camera model (pinhole, atan, etc.)
 */
FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam, bool use_imu) :
  FrameHandlerBase(),
  cam_(cam),
  reprojector_(cam_, map_),
  depth_filter_(NULL),
  use_imu_(use_imu),
  imu_handler_(nullptr),
  imu_calibrator_(nullptr),
  last_imu_timestamp_(0.0),
  last_optimized_quat_(Quaterniond::Identity())
{
  initialize();
}

/**
 * @brief Initializes depth filter and feature detector.
 * 
 * Sets up the FAST detector and depth filter with callback to add
 * converged seeds to the map as candidate points.
 */
void FrameHandlerMono::initialize()
{
  // Create FAST corner detector
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
  
  // Depth filter callback: add converged seeds as candidate map points
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->applyConfigOptions();
  if(svo::Config::useThreadedDepthfilter())
    depth_filter_->startThread();  // Run depth filter in background thread
  else
    SVO_INFO_STREAM("DepthFilter: running in synchronous mode (no background thread)");
}

/**
 * @brief Destructor - stops depth filter thread.
 */
FrameHandlerMono::~FrameHandlerMono()
{
  delete depth_filter_;
}

/**
 * @brief Main entry point: processes a new image.
 * 
 * This is the main function called for each incoming camera image.
 * It dispatches to the appropriate processing function based on
 * the current pipeline stage.
 * 
 * @param img Grayscale image (CV_8UC1)
 * @param timestamp Image timestamp
 */
void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
  if(!startFrameProcessingCommon(timestamp))
    return;  // Paused or error

  // Clear keyframe sets from previous iteration
  core_kfs_.clear();
  overlap_kfs_.clear();

  // Create new frame with image pyramid
  SVO_START_TIMER("pyramid_creation");
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  SVO_STOP_TIMER("pyramid_creation");

  // Dispatch based on current stage
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();             // Normal tracking
  else if(stage_ == STAGE_SECOND_FRAME)
    res = processSecondFrame();       // Initialization phase 2
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstFrame();        // Initialization phase 1
  else if(stage_ == STAGE_RELOCALIZING)
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // Update frame references
  last_frame_ = new_frame_;
  new_frame_.reset();
  
  // Finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

/**
 * @brief Processes the first frame (initialization phase 1).
 * 
 * Sets the first frame as a keyframe with identity pose.
 * Detects features for tracking in the next frame.
 * 
 * @return RESULT_IS_KEYFRAME on success, RESULT_NO_KEYFRAME on failure
 */
FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
  // Pure visual: world origin at first frame, no IMU rotation prior.
  // SVO is a direct method — any external rotation distorts photometric alignment.
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());
  
  // Initialize KLT tracker with first frame features
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;
  
  new_frame_->setKeyframe();
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_SECOND_FRAME;
  SVO_INFO_STREAM("Init: Selected first frame.");
  return RESULT_IS_KEYFRAME;
}

/**
 * @brief Processes the second frame (initialization phase 2).
 * 
 * Tracks features from first frame using KLT, estimates relative pose
 * using homography decomposition, and triangulates initial 3D points.
 * 
 * @return RESULT_IS_KEYFRAME on success, RESULT_FAILURE on tracking failure,
 *         RESULT_NO_KEYFRAME if not enough disparity yet
 */
FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
  // Track features and check for sufficient disparity
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
  if(res == initialization::FAILURE)
    return RESULT_FAILURE;
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME;  // Not enough disparity yet

  // Refine initial map with two-view bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  // Set as keyframe and initialize depth filter
  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // Add to map and transition to normal tracking
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;
}

/**
 * @brief Processes a normal tracking frame.
 *
 * Main tracking pipeline:
 * 1. Initialize pose from last frame (constant velocity)
 * 2. Sparse image alignment (coarse pose estimation)
 * 3. Reproject map points and align features (fine pose estimation)
 * 4. Pose optimization (Gauss-Newton refinement) — pure visual only
 * 5. Structure optimization (refine 3D points)
 * 6. Keyframe decision
 * 7. Optional local bundle adjustment
 * 8. Update depth filter
 *
 * @return RESULT_IS_KEYFRAME if new keyframe, RESULT_NO_KEYFRAME otherwise,
 *         RESULT_FAILURE on tracking failure
 */
FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
  // === Pose Initialization: IMU inter-frame prediction ===
  // IMU gyro integration provides an initial rotation estimate between frames.
  // This is a PURE initial guess — it never enters the optimizer or corrupts the map.
  // Fallback to constant velocity (translation only) if IMU is unavailable.
  if (use_imu_ && imu_handler_ != nullptr && last_frame_ != nullptr &&
      last_imu_timestamp_ > 0.0)
  {
    Eigen::Quaterniond R_imu_delta;
    if (imu_handler_->getPoseIncrement(last_imu_timestamp_, new_frame_->timestamp_, R_imu_delta))
    {
      // Predict rotation: R_cam_world(new) = R_imu_delta * R_cam_world(last)
      // Camera rotation is the inverse of IMU rotation in world frame.
      const Eigen::Matrix3d R_last = last_frame_->T_f_w_.rotationMatrix();
      const Eigen::Matrix3d R_new = R_last * R_imu_delta.conjugate().toRotationMatrix();
      new_frame_->T_f_w_ = SE3(R_new, last_frame_->T_f_w_.translation());
    }
    else
    {
      new_frame_->T_f_w_ = last_frame_->T_f_w_;
    }
  }
  else
  {
    new_frame_->T_f_w_ = last_frame_ ? last_frame_->T_f_w_ : SE3(Matrix3d::Identity(), Vector3d::Zero());
  }

  // === Stage 1: Sparse Image Alignment ===
  // Direct method: minimize photometric error between frames.
  // Optionally inject IMU rotation as a weighted information-matrix prior.
  SVO_START_TIMER("sparse_img_align");
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);

  // Compute IMU motion prior and inject as weighted prior into SparseImgAlign.
  // On each Gauss-Newton iteration, applyPrior() adds:
  //   H += λ_rot * H_max_rot * I_3  (rotation block)
  //   g += λ_rot * H_max_rot * log(R_prior^-1 * R_current)
  // λ_rot = 0.5 follows rpg/vio_mono.yaml; translation prior stays 0 (pure visual).
  // The scale-adaptive weighting releases control when visual features are strong.
  if (use_imu_ && imu_handler_ != nullptr && last_frame_ != nullptr &&
      last_imu_timestamp_ > 0.0)
  {
    getMotionPrior();
    if (have_motion_prior_)
    {
      img_align.setWeightedPrior(T_newimu_lastimu_prior_,
                                 img_align_prior_lambda_rot_,
                                 img_align_prior_lambda_trans_);
    }
  }

  size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // === Stage 2: Map Reprojection & Feature Alignment ===
  // Project map points to frame and align features to find correspondences
  SVO_START_TIMER("reproject");
  reprojector_.reprojectMap(new_frame_, overlap_kfs_);
  SVO_STOP_TIMER("reproject");
  const size_t repr_n_new_references = reprojector_.n_matches_;
  const size_t repr_n_mps = reprojector_.n_trials_;
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);

  SVO_WARN_STREAM("processFrame: img_align=" << img_align_n_tracked
      << "  T=" << new_frame_->T_f_w_.translation().transpose()
      << "  repr_trials=" << repr_n_mps << "  repr_matches=" << repr_n_new_references
      << "  reproj_ok=" << (repr_n_new_references >= Config::qualityMinFts()));
  
  // Check for tracking failure
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
    new_frame_->T_f_w_ = last_frame_->T_f_w_;  // Reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;
  }

  // === Stage 3: Pose Optimization ===
  // Gauss-Newton refinement of reprojected 3D points.
  // Optionally inject IMU rotation prior (rpg/imufusion style) when
  // pose_optim_prior_lambda_rot_ > 0: the optimizer pulls the rotation
  // toward the IMU-derived prediction based on the last visual result.
  SVO_START_TIMER("pose_optimizer");
  size_t sfba_n_edges_final;
  double sfba_thresh, sfba_error_init, sfba_error_final;
  if (pose_optim_prior_lambda_rot_ > 0.0 && use_imu_ && imu_handler_ != nullptr &&
      last_imu_timestamp_ > 0.0)
  {
    // Get IMU delta rotation from last to current frame
    Eigen::Quaterniond R_imu_delta;
    if (imu_handler_->getPoseIncrement(last_imu_timestamp_,
                                      new_frame_->timestamp_, R_imu_delta))
    {
      // Use last visual rotation as the IMU world reference.
      // This constrains the optimizer to stay close to the IMU-derived
      // rotation while still being driven by visual reprojection errors.
      pose_optimizer::optimizeGaussNewtonWithImuPrior(
          Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
          new_frame_,
          last_optimized_quat_,          // R_world_from_imu (≈ visual rotation)
          R_imu_delta,                  // R_imu_last_from_imu_cur
          pose_optim_prior_lambda_rot_,
          sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
    }
    else
    {
      pose_optimizer::optimizeGaussNewton(
          Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
          new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
    }
  }
  else
  {
    pose_optimizer::optimizeGaussNewton(
        Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
        new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  }
  SVO_STOP_TIMER("pose_optimizer");

  // Save optimized rotation for next frame
  last_optimized_quat_ = Quaterniond(new_frame_->T_f_w_.rotationMatrix());
  // Update IMU timestamp for next inter-frame integration
  if (use_imu_ && imu_handler_ != nullptr)
    last_imu_timestamp_ = new_frame_->timestamp_;

  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  
  // Require at least qualityMinFts() inliers. If the reprojector found a lot of matches
  // but the optimizer rejected most (e.g., during reloc recovery or motion blur), still
  // allow tracking to continue with a relaxed threshold so we don't wipe the map.
  const bool sfba_ok = (sfba_n_edges_final >= Config::qualityMinFts()) ||
      (sfba_n_edges_final >= Config::qualityMinFts() / 2 &&
       repr_n_new_references >= Config::qualityMinFts() * 2);
  if (!sfba_ok)
    return RESULT_FAILURE;

  // === Stage 4: Structure Optimization ===
  // Refine 3D point positions
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  // === Stage 5: Keyframe Decision ===
  core_kfs_.insert(new_frame_);
  setTrackingQuality(sfba_n_edges_final);
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_;  // Reset pose
    return RESULT_FAILURE;
  }
  
  // Check if we need a new keyframe
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    depth_filter_->addFrame(new_frame_);  // Still use for depth estimation
    return RESULT_NO_KEYFRAME;
  }
  
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // === New Keyframe Processing ===
  // Add frame references to observed points
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  
  // Promote candidate points to regular map points
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // === Stage 6: Optional Local Bundle Adjustment ===
#ifdef USE_BUNDLE_ADJUSTMENT
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
    setCoreKfs(Config::coreNKfs());  // Select keyframes for local BA
    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // Initialize depth filter seeds for new keyframe
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // === Stage 7: Keyframe Management ===
  // Remove oldest keyframe if map is too large
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame);
    map_.safeDeleteFrame(furthest_frame);
  }

  // Add new keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}

/**
 * @brief Attempts to relocalize after tracking failure.
 * 
 * Uses sparse image alignment against the closest keyframe to
 * recover camera pose.
 * 
 * @param T_cur_ref Initial relative pose estimate
 * @param ref_keyframe Reference keyframe for relocalization
 * @return RESULT_NO_KEYFRAME on success, RESULT_FAILURE otherwise
 */
FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
    const SE3& T_cur_ref,
    FramePtr ref_keyframe)
{
  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
  if(ref_keyframe == nullptr)
  {
    SVO_INFO_STREAM("No reference keyframe.");
    return RESULT_FAILURE;
  }
  
  // Try sparse image alignment against reference keyframe
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
  
  if(img_align_n_tracked > 30)
  {
    // Good alignment - try normal processing
    SE3 T_f_w_last = last_frame_->T_f_w_;
    last_frame_ = ref_keyframe;
    FrameHandlerMono::UpdateResult res = processFrame();
    if(res != RESULT_FAILURE)
    {
      stage_ = STAGE_DEFAULT_FRAME;
      SVO_INFO_STREAM("Relocalization successful.");
    }
    else
      new_frame_->T_f_w_ = T_f_w_last;  // Reset to last good pose
    return res;
  }
  return RESULT_FAILURE;
}

/**
 * @brief External relocalization at a specific pose.
 * 
 * Allows external systems to initialize tracking at a known pose
 * relative to an existing keyframe.
 * 
 * @param keyframe_id ID of reference keyframe
 * @param T_f_kf Pose relative to keyframe
 * @param img New image
 * @param timestamp Image timestamp
 * @return true on success
 */
bool FrameHandlerMono::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  FramePtr ref_keyframe;
  if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
    return false;
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
  if(res != RESULT_FAILURE) {
    last_frame_ = new_frame_;
    return true;
  }
  return false;
}

/**
 * @brief Resets all state for a fresh start.
 */
void FrameHandlerMono::resetAll()
{
  resetCommon();
  last_frame_.reset();
  new_frame_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  depth_filter_->reset();
  if (use_imu_)
  {
    last_optimized_quat_ = Quaterniond::Identity();
    last_imu_timestamp_ = 0.0;
  }
}

/**
 * @brief Sets an external first frame (for testing/external init).
 * 
 * @param first_frame Pre-initialized first keyframe
 */
void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
  resetAll();
  last_frame_ = first_frame;
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
}

/**
 * @brief Determines if a new keyframe is needed.
 * 
 * Uses distance-based criterion: new keyframe needed if camera moved
 * far enough from all existing keyframes (relative to scene depth).
 * 
 * @param scene_depth_mean Mean scene depth
 * @return true if new keyframe should be selected
 */
bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
    // Compute relative position in current frame coordinates
    Vector3d relpos = new_frame_->w2f(it->first->pos());
    
    // Check if too close to existing keyframe (relative to depth)
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;
}

/**
 * @brief Selects core keyframes for local bundle adjustment.
 * 
 * Chooses the n_closest keyframes with most feature overlap
 * with the current frame.
 * 
 * @param n_closest Number of keyframes to select
 */
void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  // Sort by overlap count (descending)
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  
  // Add sorted keyframes to core set
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

/**
 * @brief Computes IMU motion prior between last and new frame.
 *
 * Three-tier fallback:
 * 1. IMU gyro integration → integrate biased gyroscope data between frames
 *    Gets R_imu_delta = exp(ω_corrected * dt) for each IMU measurement interval.
 *    Skipped if zero-motion detected (accel_std > thresh → stationary).
 * 2. Constant velocity (when lambda > 0) → T = Identity, have_motion_prior_ = true
 * 3. No prior available → have_motion_prior_ = false
 *
 * Populates T_newimu_lastimu_prior_ (SE3: rotation from last IMU to new IMU frame).
 * Translation is set to zero since monocular IMU has no metric scale.
 */
void FrameHandlerMono::getMotionPrior()
{
  // Update zero-motion detection using accelerometer variance
  if (use_imu_ && imu_handler_ != nullptr)
  {
    double accel_mean_norm, accel_std_norm;
    Eigen::Vector3d accel_mean_vec;
    if (imu_handler_->getWindowedAccelerometerStats(
            zero_motion_window_sec_, accel_mean_norm, accel_std_norm, accel_mean_vec))
    {
      is_stationary_ = (accel_std_norm < zero_motion_accel_std_thresh_);
    }
  }

  if (use_imu_ && imu_handler_ != nullptr && last_frame_ != nullptr &&
      last_imu_timestamp_ > 0.0 && !is_stationary_)
  {
    Eigen::Quaterniond R_imu_delta;
    if (imu_handler_->getPoseIncrement(last_imu_timestamp_,
                                      new_frame_->timestamp_, R_imu_delta))
    {
      // R_imu_delta: rotation from last IMU to new IMU frame.
      // For SparseImgAlign prior: T_newimu_lastimu_prior_ = exp(R_imu_delta), t=0
      // (inverse of what we need for initial pose, but correct for prior).
      T_newimu_lastimu_prior_ = Sophus::SE3d(
          Sophus::SO3d(R_imu_delta.conjugate().toRotationMatrix()),
          Eigen::Vector3d::Zero());
      have_motion_prior_ = true;
      return;
    }
  }

  // Tier 2: Constant velocity assumption (triggered when lambda > 0)
  if (img_align_prior_lambda_rot_ > 0.0 || img_align_prior_lambda_trans_ > 0.0)
  {
    T_newimu_lastimu_prior_ = Sophus::SE3d();  // Identity
    have_motion_prior_ = true;
    return;
  }

  // Tier 3: No prior
  have_motion_prior_ = false;
}

}  // namespace svo

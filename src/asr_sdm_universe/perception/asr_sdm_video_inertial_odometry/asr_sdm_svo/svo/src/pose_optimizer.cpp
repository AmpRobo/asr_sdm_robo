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
 * @file pose_optimizer.cpp
 * @brief Camera pose optimization using Gauss-Newton with optional IMU rotation prior.
 *
 * This module refines the camera pose by minimizing reprojection errors
 * of 3D points observed in the frame. Optionally, an IMU rotation prior
 * can be applied to constrain the camera orientation using gyroscope data.
 *
 * The pose is represented in SE(3) (6 DOF: rotation + translation).
 * The optimization solves for incremental updates in the tangent space (se3).
 */

#include <stdexcept>
#include <svo/pose_optimizer.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace svo {
namespace pose_optimizer {

namespace {

/// Compute skew-symmetric matrix from 3-vector (for cross product).
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<  0.0, -v(2),  v(1),
        v(2),    0.0, -v(0),
       -v(1),   v(0),    0.0;
  return S;
}

/// IMU rotation prior regularization with camera-down installation correction.
///
/// R_cam_imu_T = R_cam_imu^T (Rx(180°) for camera-down rig).
///
/// IMU frame: z ≈ world +z (upward). Camera frame: optical axis = world -z.
///
/// Camera rotation in world: R_cam_world = R_cam_imu^T * R_imu_world^T
///
/// IMU-predicted camera rotation at current frame:
///   R_cam_world_imu = R_cam_imu^T * (R_imu_world * R_imu_last_from_imu_cur^T)^T
///
/// Residual: theta = log(R_cam_world_imu * R_cam_world_prior^{-1})
/// Cost: J = theta^T * theta
///
/// Outlier detection: if ||theta|| > outlier_threshold_deg, reduce effective lambda
/// to prevent IMU from pulling the pose away from visual.
inline void computeImuPriorTerm(
    const Eigen::Matrix3d& R_cam_world_prior,
    const Eigen::Matrix3d& R_imu_world_mat,
    const Eigen::Matrix3d& R_imu_last_from_imu_cur,
    const Eigen::Matrix3d& R_cam_imu_T,
    double lambda,
    Vector6d* g_prior,
    Matrix6d* H_prior,
    double* outlier_theta_deg = nullptr)
{
  // IMU predicts: R_imu_world(new) = R_imu_delta * R_imu_world(old)
  // Camera world: R_cam_world = R_cam_imu^T * R_imu_world^T
  // So IMU-predicted camera world rotation:
  // R_cam_world_imu = R_cam_imu^T * (R_imu_last_mat * R_imu_world_mat)^T
  //                 = R_cam_imu^T * R_imu_world_mat^T * R_imu_last_mat^T
  const Eigen::Matrix3d R_cam_world_imu = R_cam_imu_T * R_imu_world_mat.transpose() * R_imu_last_from_imu_cur.transpose();

  // Error rotation: R_err = R_cam_world_imu * R_cam_world_prior^{-1}
  const Eigen::Matrix3d R_err = R_cam_world_imu * R_cam_world_prior.transpose();

  // Axis-angle from SO(3) logarithm
  Eigen::AngleAxisd aa(R_err);
  double angle = aa.angle();
  if (angle > M_PI)  // Unwrap to [-pi, pi]
    angle -= 2 * M_PI;
  const Eigen::Vector3d theta = aa.axis() * angle;
  const double theta_deg = std::abs(angle) * 180.0 / M_PI;

  if (outlier_theta_deg != nullptr)
    *outlier_theta_deg = theta_deg;

  // Outlier suppression: if IMU prediction differs by > outlier_threshold_deg
  // from the visual estimate, reduce effective lambda dramatically.
  // This prevents the IMU from overriding visual when it diverges.
  static constexpr double OUTLIER_THRESH_DEG = 30.0;
  static constexpr double OUTLIER_LAMBDA_FACTOR = 0.01;  // 100x reduction
  double effective_lambda = lambda;
  if (theta_deg > OUTLIER_THRESH_DEG)
  {
    effective_lambda *= OUTLIER_LAMBDA_FACTOR;
  }

  const double w_sq = effective_lambda * effective_lambda;
  g_prior->setZero();
  g_prior->tail<3>() = w_sq * theta;

  H_prior->setZero();
  H_prior->topLeftCorner<3, 3>() = w_sq * Eigen::Matrix3d::Identity();
}

}  // anonymous namespace

// =============================================================================
// POSE OPTIMIZER CLASS (rpg/imufusion mirror)
// =============================================================================

PoseOptimizer::PoseOptimizer(SolverOptions options) : options_(options) {}

void PoseOptimizer::setRotationPrior(const Quaterniond& R_frame_world, double lambda)
{
  have_prior_ = true;
  prior_lambda_ = lambda;
  prior_R_world_ = R_frame_world;
}

size_t PoseOptimizer::run(
    FramePtr& frame,
    const double reproj_thresh,
    const Quaterniond& R_world_from_imu,
    const Quaterniond& R_imu_last_from_imu_cur,
    double lambda,
    double& estimated_scale,
    double& error_init,
    double& error_final)
{
  if (lambda <= 0.0)
  {
    double dummy;
    size_t num_obs;
    optimizeGaussNewton(reproj_thresh, options_.max_iter, false, frame,
                        estimated_scale, error_init, dummy, num_obs);
    return num_obs;
  }
  imu_delta_ = R_imu_last_from_imu_cur;
  size_t num_obs;
  optimizeGaussNewtonWithImuPrior(
      reproj_thresh, options_.max_iter, false, frame,
      R_world_from_imu, R_imu_last_from_imu_cur, lambda,
      estimated_scale, error_init, error_final, num_obs);
  return num_obs;
}

/**
 * @brief Optimizes camera pose using Gauss-Newton with robust weighting.
 *
 * Algorithm:
 * 1. Compute initial error scale using MAD estimator
 * 2. Iterate Gauss-Newton updates with Tukey robust weights
 * 3. After iteration 5, switch to tighter threshold
 * 4. Remove outliers exceeding reprojection threshold
 * 5. Compute pose covariance
 */
void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs)
{
  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  std::vector<float> errors;
  errors.reserve(frame->fts_.size());
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1 << (*it)->level);
    errors.push_back(e.norm());
  }
  if (errors.empty())
    return;

  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);
  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;

  for (size_t iter = 0; iter < n_iter; ++iter)
  {
    if (iter == 5)
      scale = 0.85 / frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
    {
      if ((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
      e *= sqrt_inv_cov;
      if (iter == 0)
        chi2_vec_init.push_back(e.squaredNorm());
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm() / scale);
      A.noalias() += J.transpose() * J * weight;
      b.noalias() -= J.transpose() * e * weight;
      new_chi2 += e.squaredNorm() * weight;
    }

    const Vector6d dT(A.ldlt().solve(b));

    if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
    {
      if (verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old;
      break;
    }

    SE3 T_new = SE3::exp(dT) * frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;

    if (verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    if (vk::norm_max(dT) <= EPS)
      break;
  }

  const double pixel_variance = 1.0;
  frame->Cov_ = pixel_variance * (A * std::pow(frame->cam_->errorMultiplier2(), 2)).inverse();

  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if (e.norm() > reproj_thresh_scaled)
    {
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  error_init = 0.0;
  error_final = 0.0;
  if (!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init)) * frame->cam_->errorMultiplier2();
  if (!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final)) * frame->cam_->errorMultiplier2();
  estimated_scale *= frame->cam_->errorMultiplier2();

  if (verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;

  num_obs -= n_deleted_refs;
}

// =============================================================================
// POSE OPTIMIZER WITH IMU ROTATION PRIOR
// =============================================================================

void optimizeGaussNewtonWithImuPrior(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    const Quaterniond& R_world_from_imu,
    const Quaterniond& R_imu_last_from_imu_cur,
    double lambda,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs,
    const Eigen::Matrix3d& R_cam_imu_T)
{
  if (lambda <= 0.0)
  {
    optimizeGaussNewton(reproj_thresh, n_iter, verbose, frame,
                        estimated_scale, error_init, error_final, num_obs);
    return;
  }

  double chi2(0.0);
  vector<double> chi2_vec_init, chi2_vec_final;
  vk::robust_cost::TukeyWeightFunction weight_function;
  SE3 T_old(frame->T_f_w_);
  Matrix6d A;
  Vector6d b;

  std::vector<float> errors;
  errors.reserve(frame->fts_.size());
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f)
               - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    e *= 1.0 / (1 << (*it)->level);
    errors.push_back(e.norm());
  }
  if (errors.empty())
    return;
  vk::robust_cost::MADScaleEstimator scale_estimator;
  estimated_scale = scale_estimator.compute(errors);
  num_obs = errors.size();
  chi2_vec_init.reserve(num_obs);
  chi2_vec_final.reserve(num_obs);
  double scale = estimated_scale;

  // R_imu_world_mat: IMU orientation in world frame (rotation_prior_ = R_imu_delta * R_imu_world)
  const Eigen::Matrix3d R_imu_world_mat = R_world_from_imu.toRotationMatrix();
  const Eigen::Matrix3d R_imu_last_mat = R_imu_last_from_imu_cur.toRotationMatrix();
  // R_cam_world_prior (initial estimate from processFrame init):
  // R_cam_world = R_cam_imu^T * R_imu_world^T
  const Eigen::Matrix3d R_cam_world_prior = R_cam_imu_T * R_imu_world_mat.transpose();

  for (size_t iter = 0; iter < n_iter; ++iter)
  {
    if (iter == 5)
      scale = 0.85 / frame->cam_->errorMultiplier2();

    b.setZero();
    A.setZero();
    double new_chi2(0.0);

    for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
    {
      if ((*it)->point == NULL)
        continue;
      Matrix26d J;
      Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
      Frame::jacobian_xyz2uv(xyz_f, J);
      Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
      double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
      e *= sqrt_inv_cov;
      if (iter == 0)
        chi2_vec_init.push_back(e.squaredNorm());
      J *= sqrt_inv_cov;
      double weight = weight_function.value(e.norm() / scale);
      A.noalias() += J.transpose() * J * weight;
      b.noalias() -= J.transpose() * e * weight;
      new_chi2 += e.squaredNorm() * weight;
    }

    // === Add IMU Rotation Prior (with R_cam_imu correction + outlier suppression) ===
    if (iter == 0)
    {
      Vector6d g_prior;
      Matrix6d H_prior;
      double theta_deg = 0.0;
      computeImuPriorTerm(
          R_cam_world_prior, R_imu_world_mat, R_imu_last_mat,
          R_cam_imu_T, lambda, &g_prior, &H_prior, &theta_deg);
      A += H_prior;
      b += g_prior;
      if (verbose && theta_deg > 5.0)
        std::cout << "  IMU prior theta=" << theta_deg << "deg (effective lambda reduced)" << std::endl;
    }

    const Vector6d dT(A.ldlt().solve(b));

    if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
    {
      if (verbose)
        std::cout << "it " << iter
                  << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
      frame->T_f_w_ = T_old;
      break;
    }

    SE3 T_new = SE3::exp(dT) * frame->T_f_w_;
    T_old = frame->T_f_w_;
    frame->T_f_w_ = T_new;
    chi2 = new_chi2;

    if (verbose)
      std::cout << "it " << iter
                << "\t Success \t new_chi2 = " << new_chi2
                << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

    if (vk::norm_max(dT) <= EPS)
      break;
  }

  const double pixel_variance = 1.0;
  frame->Cov_ = pixel_variance * (A * std::pow(frame->cam_->errorMultiplier2(), 2)).inverse();

  double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2();
  size_t n_deleted_refs = 0;
  for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
  {
    if ((*it)->point == NULL)
      continue;
    Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_);
    double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
    e *= sqrt_inv_cov;
    chi2_vec_final.push_back(e.squaredNorm());
    if (e.norm() > reproj_thresh_scaled)
    {
      (*it)->point = NULL;
      ++n_deleted_refs;
    }
  }

  error_init = 0.0;
  error_final = 0.0;
  if (!chi2_vec_init.empty())
    error_init = sqrt(vk::getMedian(chi2_vec_init)) * frame->cam_->errorMultiplier2();
  if (!chi2_vec_final.empty())
    error_final = sqrt(vk::getMedian(chi2_vec_final)) * frame->cam_->errorMultiplier2();
  estimated_scale *= frame->cam_->errorMultiplier2();

  if (verbose)
    std::cout << "n deleted obs = " << n_deleted_refs
              << "\t scale = " << estimated_scale
              << "\t error init = " << error_init
              << "\t error end = " << error_final << std::endl;

  num_obs -= n_deleted_refs;
}

}  // namespace pose_optimizer
}  // namespace svo

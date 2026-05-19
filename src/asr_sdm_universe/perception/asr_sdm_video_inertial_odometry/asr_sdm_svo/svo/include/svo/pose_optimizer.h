#ifndef SVO_POSE_OPTIMIZER_H_
#define SVO_POSE_OPTIMIZER_H_

#include <svo/global.h>

namespace svo
{

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Point;

namespace pose_optimizer
{

/// Lightweight pose optimizer class mirroring rpg/imufusion PoseOptimizer.
class PoseOptimizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct SolverOptions
  {
    int max_iter;
    double eps;
    SolverOptions() : max_iter(10), eps(1e-6) {}
  };

  explicit PoseOptimizer(SolverOptions options = SolverOptions());
  virtual ~PoseOptimizer() = default;

  /// Inject an IMU-derived rotation prior on subsequent run() calls.
  /// Follows rpg/imufusion pattern: sets zero translation, identity info on rotation block.
  void setRotationPrior(const Quaterniond& R_frame_world, double lambda);

  /// Entry point: optimize pose of a single frame with optional IMU rotation prior.
  size_t run(
      FramePtr& frame,
      const double reproj_thresh,
      const Quaterniond& R_world_from_imu,
      const Quaterniond& R_imu_last_from_imu_cur,
      double lambda,
      double& estimated_scale,
      double& error_init,
      double& error_final);

protected:
  SolverOptions options_;
  bool have_prior_ = false;
  double prior_lambda_ = 0.0;
  int iter_ = 0;
  Quaterniond prior_R_world_ = Quaterniond::Identity();
  Quaterniond imu_delta_ = Quaterniond::Identity();
  Eigen::Matrix<double, 6, 6> prior_I_;
};

/// Standard version without IMU prior (backward compatible).
void optimizeGaussNewton(
  const double reproj_thresh, const size_t n_iter, const bool verbose, FramePtr & frame,
  double & estimated_scale, double & error_init, double & error_final, size_t & num_obs);

/// Version with IMU rotation prior + camera-down installation correction.
///   - R_world_from_imu: gravity-aligned orientation of IMU w.r.t. world
///   - R_imu_last_from_imu_cur: relative rotation from IMU gyroscope
///   - lambda: regularization strength (0 = no prior, >0 = stronger prior)
///   - R_cam_imu_T: camera-down installation correction matrix (R_cam_imu^T)
void optimizeGaussNewtonWithImuPrior(
  const double reproj_thresh, const size_t n_iter, const bool verbose, FramePtr & frame,
  const Quaterniond& R_world_from_imu, const Quaterniond& R_imu_last_from_imu_cur,
  double lambda,
  double & estimated_scale, double & error_init, double & error_final, size_t & num_obs,
  const Eigen::Matrix3d& R_cam_imu_T = Eigen::Matrix3d::Identity());

}  // namespace pose_optimizer
}  // namespace svo

#endif

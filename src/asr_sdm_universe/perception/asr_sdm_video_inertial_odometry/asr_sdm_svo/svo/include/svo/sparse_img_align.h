#ifndef SVO_SPARSE_IMG_ALIGN_H_
#define SVO_SPARSE_IMG_ALIGN_H_

#include <svo/global.h>
#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>

namespace vk
{
class AbstractCamera;
}

namespace svo
{

class Feature;

/// Optimize the pose of the frame by minimizing the photometric error of feature patches.
class SparseImgAlign : public vk::NLLSSolver<6, Sophus::SE3d>
{
  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2 * patch_halfsize_;
  static const int patch_area_ = patch_size_ * patch_size_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat resimg_;

  SparseImgAlign(
    int n_levels, int min_level, int n_iter, Method method, bool display, bool verbose);

  size_t run(FramePtr ref_frame, FramePtr cur_frame);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  Eigen::Matrix<double, 6, 6> getFisherInformation();

  /// Inject an IMU-derived rotation prior into the Gauss-Newton optimization.
  ///
  /// On each iteration, applyPrior() adds a weighted information-matrix regularizer:
  ///   H += lambda_trans * H_max_trans * I_3  (translation block)
  ///   H += lambda_rot   * H_max_rot   * I_3  (rotation block)
  ///   g += lambda_trans * H_max_trans * (t_cur - t_prior)
  ///   g += lambda_rot   * H_max_rot   * log(R_prior^-1 * R_cur)
  ///
  /// lambda values follow the rpg/vio_mono.yaml convention:
  ///   lambda_rot = 0.5  → moderate IMU pull on rotation
  ///   lambda_trans = 0.0 → pure visual translation (recommended)
  ///
  /// Must be called before run().
  void setWeightedPrior(
      const Sophus::SE3d& T_cur_ref_prior,
      double lambda_rot,
      double lambda_trans);

protected:
  FramePtr ref_frame_;  //!< reference frame, has depth for gradient pixels.
  FramePtr cur_frame_;  //!< only the image is known!
  int level_;           //!< current pyramid level on which the optimization runs.
  bool display_;        //!< display residual image.
  int max_level_;       //!< coarsest pyramid level for the alignment.
  int min_level_;       //!< finest pyramid level for the alignment.

  // cache:
  Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache_;
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_;

  // IMU prior state (populated by setWeightedPrior)
  bool have_imu_prior_ = false;
  Sophus::SE3d T_cur_ref_prior_;     //!< IMU-derived prior pose
  double prior_lambda_rot_ = 0.0;    //!< rotation prior weight
  double prior_lambda_trans_ = 0.0;  //!< translation prior weight
  Eigen::Matrix<double, 6, 6> prior_I_;  //!< accumulated info matrix (recomputed on iter 0)

  void precomputeReferencePatches();
  virtual double computeResiduals(
    const Sophus::SE3d & model, bool linearize_system, bool compute_weight_scale = false);
  virtual int solve();
  virtual void update(const ModelType & old_model, ModelType & new_model);
  virtual void startIteration();
  virtual void finishIteration();

  /// Apply IMU information-matrix prior on each Gauss-Newton iteration.
  virtual void applyPrior(const Sophus::SE3d& T_cur_from_ref);
};

}  

#endif 

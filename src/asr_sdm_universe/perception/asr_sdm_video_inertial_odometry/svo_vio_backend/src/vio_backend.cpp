#include "svo_vio_backend/vio_backend.h"
#include "svo_vio_backend/vins_factors.h"

#include <algorithm>
#include <iomanip>

namespace svo_vio_backend {

VioBackend::VioBackend()
{
  ric_.setIdentity();
  tic_.setZero();
  g_.setZero();
  g_(2) = 9.805;

  noise_params_.acc_n = 0.1;
  noise_params_.gyr_n = 0.01;
  noise_params_.acc_w = 0.002;
  noise_params_.gyr_w = 2e-5;

  imu_preintegrator_ = std::make_unique<IMUPreintegrator>(noise_params_);
  optimizer_ = std::make_unique<VinsOptimizer>();
  optimizer_->setMaxIterations(max_iterations_);
  optimizer_->setSolverTimeLimit(solver_time_limit_);
  optimizer_->setNumThreads(2);

  // The pose manifold wraps PoseLocalParameterization (6-DOF: 3 pos + 3 rot).
  pose_manifold_ = std::unique_ptr<ceres::Manifold>(
      new ceres::AutoDiffManifold<PoseLocalParameterization, 7, 6>(
          new PoseLocalParameterization()));
}

VioBackend::~VioBackend()
{
  for (auto* blk : param_pose_)      delete[] blk;
  for (auto* blk : param_speedbias_) delete[] blk;
  param_pose_.clear();
  param_speedbias_.clear();
}

void VioBackend::setIMUExtrinsics(const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic)
{
  ric_ = ric;
  tic_ = tic;
}

void VioBackend::setIMUNoise(double acc_n, double acc_w, double gyr_n, double gyr_w)
{
  noise_params_.acc_n = acc_n;
  noise_params_.acc_w = acc_w;
  noise_params_.gyr_n = gyr_n;
  noise_params_.gyr_w = gyr_w;

  imu_preintegrator_ = std::make_unique<IMUPreintegrator>(noise_params_);
}

void VioBackend::setMaxIterations(int iter) { max_iterations_ = iter; if (optimizer_) optimizer_->setMaxIterations(iter); }
void VioBackend::setSolverTimeLimit(double time_limit) { solver_time_limit_ = time_limit; if (optimizer_) optimizer_->setSolverTimeLimit(time_limit); }

void VioBackend::addIMUMeasurement(double timestamp,
                                   const Eigen::Vector3d& acc,
                                   const Eigen::Vector3d& gyro)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  IMUMeasurement measurement;
  measurement.timestamp = timestamp;
  measurement.acc = acc;
  measurement.gyro = gyro;

  imu_buffer_.push_back(measurement);

  if (imu_preintegrator_->isInitialized()) {
    imu_preintegrator_->addMeasurement(measurement);
  }

  last_imu_timestamp_ = timestamp;
}

void VioBackend::addKeyFrame(void* frame_ptr, double timestamp,
                             const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
{
  (void)frame_ptr;

  if (!initialized_) {
    sliding_window_.addState(R, p);
    imu_preintegrator_->initialize(timestamp,
                                    Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero());
    initialized_ = true;
    return;
  }

  processIMUForNewFrame(timestamp);

  auto preint = imu_preintegrator_->getPreintegration();
  if (preint) {
    preintegrations_.push_back(preint);
  }

  sliding_window_.addState(R, p);
  imu_preintegrator_->initialize(timestamp,
                                  Eigen::Vector3d::Zero(),
                                  Eigen::Vector3d::Zero());

  if (sliding_window_.isFull()) {
    marginalize();
    sliding_window_.slideWindow();
  }
}

void VioBackend::processIMUForNewFrame(double timestamp)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  for (auto it = imu_buffer_.begin(); it != imu_buffer_.end(); ) {
    if (it->timestamp <= timestamp) {
      it = imu_buffer_.erase(it);
    } else {
      ++it;
    }
  }
}

bool VioBackend::getCurrentPose(Eigen::Matrix4d& T_w_b) const
{
  if (!initialized_ || sliding_window_.getWindowSize() == 0) {
    return false;
  }

  int idx = sliding_window_.getWindowSize() - 1;
  T_w_b.setIdentity();
  T_w_b.block<3,3>(0,0) = sliding_window_.getRotation(idx);
  T_w_b.block<3,1>(0,3) = sliding_window_.getPosition(idx);

  return true;
}

bool VioBackend::getCurrentVelocity(Eigen::Vector3d& v_w) const
{
  if (!initialized_ || sliding_window_.getWindowSize() == 0) {
    return false;
  }

  int idx = sliding_window_.getWindowSize() - 1;
  v_w = sliding_window_.getVelocity(idx);

  return true;
}

bool VioBackend::optimize()
{
  if (!initialized_ || sliding_window_.getWindowSize() < 2) {
    return false;
  }

  buildOptimizationProblem();

  if (!problem_ || problem_->NumParameterBlocks() == 0) {
    return false;
  }

  if (optimizer_->solve(*problem_)) {
    updateStatesFromOptimization(*problem_);
    return true;
  }
  return false;
}

void VioBackend::buildOptimizationProblem()
{
  // ------------------------------------------------------------------
  // 1. Sync in-memory parameter blocks to current sliding-window state.
  //    2 blocks per frame: pose (7) + speed_bias (9).
  // ------------------------------------------------------------------
  const int N = sliding_window_.getWindowSize();

  while (static_cast<int>(param_pose_.size()) < N) {
    param_pose_.push_back(new double[7]);
    param_speedbias_.push_back(new double[9]);
  }
  while (static_cast<int>(param_pose_.size()) > N) {
    delete[] param_pose_.back();      param_pose_.pop_back();
    delete[] param_speedbias_.back(); param_speedbias_.pop_back();
  }

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d Pi = sliding_window_.getPosition(i);
    Eigen::Matrix3d  Ri = sliding_window_.getRotation(i);
    Eigen::Quaterniond Qi(Ri);
    Qi.normalize();

    param_pose_[i][0] = Pi.x();
    param_pose_[i][1] = Pi.y();
    param_pose_[i][2] = Pi.z();
    param_pose_[i][3] = Qi.x();
    param_pose_[i][4] = Qi.y();
    param_pose_[i][5] = Qi.z();
    param_pose_[i][6] = Qi.w();

    Eigen::Vector3d Vi  = sliding_window_.getVelocity(i);
    Eigen::Vector3d Bai = sliding_window_.getAccBias(i);
    Eigen::Vector3d Bgi = sliding_window_.getGyroBias(i);

    param_speedbias_[i][0] = Vi.x();
    param_speedbias_[i][1] = Vi.y();
    param_speedbias_[i][2] = Vi.z();
    param_speedbias_[i][3] = Bai.x();
    param_speedbias_[i][4] = Bai.y();
    param_speedbias_[i][5] = Bai.z();
    param_speedbias_[i][6] = Bgi.x();
    param_speedbias_[i][7] = Bgi.y();
    param_speedbias_[i][8] = Bgi.z();
  }

  // ------------------------------------------------------------------
  // 2. Build a fresh Ceres problem.
  // ------------------------------------------------------------------
  problem_ = std::make_unique<ceres::Problem>();

  for (int i = 0; i < N; ++i) {
    problem_->AddParameterBlock(param_pose_[i],      7, pose_manifold_.get());
    problem_->AddParameterBlock(param_speedbias_[i], 9);
  }

  // Fix the first frame to remove gauge freedom (4 unobservable DOF: yaw + translation).
  if (N >= 1) {
    problem_->SetParameterBlockConstant(param_pose_[0]);
    problem_->SetParameterBlockConstant(param_speedbias_[0]);
  }

  // ------------------------------------------------------------------
  // 3. Add IMU factors between consecutive frames.
  //    preintegrations_[i] corresponds to the pair (frame i, frame i+1).
  // ------------------------------------------------------------------
  const int num_imu_factors =
      std::min<int>(N - 1, static_cast<int>(preintegrations_.size()));
  for (int i = 0; i < num_imu_factors; ++i) {
    const auto& preint = preintegrations_[i];
    if (!preint) continue;
    if (preint->getDeltaTime() < 1e-6) continue;  // skip empty preints

    auto* imu_factor = new IMUFactor(preint, g_);
    problem_->AddResidualBlock(
        imu_factor, nullptr,
        param_pose_[i],     param_speedbias_[i],
        param_pose_[i + 1], param_speedbias_[i + 1]);
  }
}

void VioBackend::updateStatesFromOptimization(const ceres::Problem& problem)
{
  (void)problem;

  const int N = sliding_window_.getWindowSize();
  const int num_blocks = static_cast<int>(param_pose_.size());
  if (num_blocks < N) return;

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d Pi(param_pose_[i][0], param_pose_[i][1], param_pose_[i][2]);
    Eigen::Quaterniond Qi(param_pose_[i][6],
                          param_pose_[i][3], param_pose_[i][4], param_pose_[i][5]);
    Qi.normalize();
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();

    Eigen::Vector3d Vi(param_speedbias_[i][0], param_speedbias_[i][1], param_speedbias_[i][2]);
    Eigen::Vector3d Bai(param_speedbias_[i][3], param_speedbias_[i][4], param_speedbias_[i][5]);
    Eigen::Vector3d Bgi(param_speedbias_[i][6], param_speedbias_[i][7], param_speedbias_[i][8]);

    sliding_window_.setPosition(i, Pi);
    sliding_window_.setRotation(i, Ri);
    sliding_window_.setVelocity(i, Vi);
    sliding_window_.setAccBias(i, Bai);
    sliding_window_.setGyroBias(i, Bgi);
  }
}

void VioBackend::marginalize()
{
  // ------------------------------------------------------------------
  // Phase-1 simplified marginalization:
  //
  // Drop the preintegration aligned to the oldest frame. Full Schur
  // complement is left for Phase 2 (we keep sliding window semantics
  // by removing the corresponding IMU factor from the next problem
  // build).
  // ------------------------------------------------------------------
  if (!preintegrations_.empty()) {
    preintegrations_.erase(preintegrations_.begin());
  }
}

int VioBackend::getWindowFrameCount() const
{
  return sliding_window_.getWindowSize();
}

}  // namespace svo_vio_backend

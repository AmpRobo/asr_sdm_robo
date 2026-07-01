#pragma once

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>

namespace svo_vio_backend {

/**
 * @brief Sliding window state for tight-coupled VIO
 */
class VinsSlidingWindow
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int WINDOW_SIZE = 10;
  static constexpr int NUM_OF_CAM = 1;

  VinsSlidingWindow();
  ~VinsSlidingWindow();

  void setWindowSize(int size);
  void clear();
  
  int getWindowSize() const { return window_size_; }
  bool isFull() const { return window_size_ >= WINDOW_SIZE + 1; }

  const Eigen::Vector3d& getPosition(int i) const { return Ps_[i]; }
  const Eigen::Matrix3d& getRotation(int i) const { return Rs_[i]; }
  const Eigen::Vector3d& getVelocity(int i) const { return Vs_[i]; }
  const Eigen::Vector3d& getAccBias(int i) const { return Bas_[i]; }
  const Eigen::Vector3d& getGyroBias(int i) const { return Bgs_[i]; }

  void setPosition(int i, const Eigen::Vector3d& p) { Ps_[i] = p; }
  void setRotation(int i, const Eigen::Matrix3d& R) { Rs_[i] = R; }
  void setVelocity(int i, const Eigen::Vector3d& v) { Vs_[i] = v; }
  void setAccBias(int i, const Eigen::Vector3d& ba) { Bas_[i] = ba; }
  void setGyroBias(int i, const Eigen::Vector3d& bg) { Bgs_[i] = bg; }

  void addState(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
  void slideWindow();

  enum class MarginFlag { OLD = 0, SECOND_NEW = 1 };
  MarginFlag marginalization_flag_ = MarginFlag::OLD;

private:
  int window_size_ = 0;

  Eigen::Vector3d Ps_[WINDOW_SIZE + 1];
  Eigen::Matrix3d Rs_[WINDOW_SIZE + 1];
  Eigen::Vector3d Vs_[WINDOW_SIZE + 1];
  Eigen::Vector3d Bas_[WINDOW_SIZE + 1];
  Eigen::Vector3d Bgs_[WINDOW_SIZE + 1];

  Eigen::Matrix3d ric_[NUM_OF_CAM];
  Eigen::Vector3d tic_[NUM_OF_CAM];
};

}  // namespace svo_vio_backend

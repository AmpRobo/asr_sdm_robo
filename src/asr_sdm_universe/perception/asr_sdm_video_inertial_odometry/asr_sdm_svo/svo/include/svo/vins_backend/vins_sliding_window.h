#pragma once

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>

namespace svo {

// Forward declarations
class Frame;
class Point;

/**
 * @brief Sliding window state for tight-coupled VIO
 * 
 * Manages the sliding window of frames with their IMU preintegration data
 * for tight-coupled visual-inertial odometry optimization.
 */
class VinsSlidingWindow
{
public:
  static constexpr int WINDOW_SIZE = 10;  // Maximum window size
  static constexpr int NUM_OF_CAM = 1;    // Number of cameras

  VinsSlidingWindow();
  ~VinsSlidingWindow();

  // Frame management
  void setWindowSize(int size);
  void clear();
  
  // State access
  int getWindowSize() const { return static_cast<int>(frames_.size()); }
  bool isFull() const { return getWindowSize() >= WINDOW_SIZE + 1; }

  // Get frame at index
  Frame* getFrame(int i) const { return frames_[i].frame; }
  
  // Get state at index
  const Eigen::Vector3d& getPosition(int i) const { return Ps_[i]; }
  const Eigen::Matrix3d& getRotation(int i) const { return Rs_[i]; }
  const Eigen::Vector3d& getVelocity(int i) const { return Vs_[i]; }
  const Eigen::Vector3d& getAccBias(int i) const { return Bas_[i]; }
  const Eigen::Vector3d& getGyroBias(int i) const { return Bgs_[i]; }

  // Set state at index
  void setPosition(int i, const Eigen::Vector3d& p) { Ps_[i] = p; }
  void setRotation(int i, const Eigen::Matrix3d& R) { Rs_[i] = R; }
  void setVelocity(int i, const Eigen::Vector3d& v) { Vs_[i] = v; }
  void setAccBias(int i, const Eigen::Vector3d& ba) { Bas_[i] = ba; }
  void setGyroBias(int i, const Eigen::Vector3d& bg) { Bgs_[i] = bg; }

  // Frame to index mapping
  int getFrameId(Frame* frame) const;
  bool hasFrame(Frame* frame) const;

  // Initialize states from first frame
  void initializeFromFirstFrame(Frame* frame);

  // Slide the window - marginalize oldest frame
  void slideWindow();

  // Add frame to window
  void addFrame(Frame* frame);

  // Marginalization flag
  enum class MarginFlag { OLD = 0, SECOND_NEW = 1 };
  MarginFlag marginalization_flag_ = MarginFlag::OLD;

private:
  // Frame pointers in sliding window
  struct FrameWithMeta {
    Frame* frame = nullptr;
    bool is_keyframe = false;
  };
  std::vector<FrameWithMeta> frames_;

  // Sliding window states
  Eigen::Vector3d Ps_[WINDOW_SIZE + 1];      // Positions
  Eigen::Matrix3d Rs_[WINDOW_SIZE + 1];      // Rotations (R_w_b)
  Eigen::Vector3d Vs_[WINDOW_SIZE + 1];      // Velocities in world frame
  Eigen::Vector3d Bas_[WINDOW_SIZE + 1];     // Accelerometer biases
  Eigen::Vector3d Bgs_[WINDOW_SIZE + 1];     // Gyroscope biases

  // Camera-IMU extrinsic parameters
  Eigen::Matrix3d ric_[NUM_OF_CAM];  // Rotation: camera to IMU
  Eigen::Vector3d tic_[NUM_OF_CAM];  // Translation: camera to IMU

  // Map from frame pointer to window index
  std::map<Frame*, int> frame_to_index_;
};

}  // namespace svo

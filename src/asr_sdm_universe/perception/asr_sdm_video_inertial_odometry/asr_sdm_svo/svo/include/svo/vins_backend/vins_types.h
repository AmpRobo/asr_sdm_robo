#pragma once

#include <Eigen/Dense>
#include <vector>

namespace svo {

// State ordering indices for optimization
enum StateOrder {
  O_P = 0,   // Position: indices [0-2]
  O_R = 3,   // Rotation (quaternion): indices [3-6]
  O_V = 6,   // Velocity: indices [6-8]
  O_BA = 9,  // Accelerometer bias: indices [9-11]
  O_BG = 12  // Gyroscope bias: indices [12-14]
};

// IMU measurement structure
struct IMUMeasurement {
  double timestamp;
  Eigen::Vector3d acc;   // Acceleration (m/s^2)
  Eigen::Vector3d gyro;   // Angular velocity (rad/s)
};

// IMU noise parameters
struct IMUNoiseParams {
  double acc_n = 0.1;   // Accelerometer noise density (m/s^2/sqrt(Hz))
  double gyr_n = 0.01;  // Gyroscope noise density (rad/s/sqrt(Hz))
  double acc_w = 0.002; // Accelerometer random walk (m/s^3/sqrt(Hz))
  double gyr_w = 2e-5;  // Gyroscope random walk (rad/s^2/sqrt(Hz))

  // Information matrix (inverse covariance)
  Eigen::Matrix<double, 18, 18> getInformationMatrix() const;
};

// Camera intrinsic parameters
struct CameraCalibration {
  double fx, fy, cx, cy;  // Focal lengths and principal point
  int width, height;       // Image dimensions
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0;  // Distortion coefficients

  Eigen::Matrix3d getK() const;
};

// Utility functions
class VinsUtility {
public:
  // Quaternion operations
  static Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta);
  static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
  
  // Convert between representations
  static Eigen::Matrix3d quatToRotation(const Eigen::Quaterniond& q);
  static Eigen::Quaterniond rotationToQuat(const Eigen::Matrix3d& R);
  
  // Normalize quaternion
  static Eigen::Quaterniond normalizeQuat(const Eigen::Quaterniond& q);

  // SE3 operations
  static Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T);
  
  // Yaw-pitch-roll to rotation
  static Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr);
  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R);

  // Angle to rotation
  static Eigen::Matrix3d angle2R(const Eigen::Vector3d& rpy);
  
  // Compute delta between two quaternions
  static Eigen::Quaterniond qLeft(const Eigen::Quaterniond& q);
  static Eigen::Quaterniond qRight(const Eigen::Quaterniond& q);
};

}  // namespace svo

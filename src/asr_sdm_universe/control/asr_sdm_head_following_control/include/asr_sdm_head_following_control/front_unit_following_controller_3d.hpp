#ifndef FRONT_UNIT_FOLLOWING_CONTROLLER_3D_HPP_
#define FRONT_UNIT_FOLLOWING_CONTROLLER_3D_HPP_

#include "asr_sdm_control_msgs/msg/robot_command.hpp"

#include <array>
#include <cstddef>

namespace asr
{

constexpr size_t kNum3dJoints = 3;
constexpr size_t kNum3dLinks = 4;
constexpr size_t kNum3dPoints = kNum3dLinks + 1;
constexpr size_t kNum3dJointDofs = 2 * kNum3dJoints;

// Each physical serial joint pair is yaw first, then pitch.
constexpr size_t yawIndex(size_t joint)
{
  return 2 * joint;
}

constexpr size_t pitchIndex(size_t joint)
{
  return 2 * joint + 1;
}

struct Vec3
{
  double x;
  double y;
  double z;
};

struct Vec2
{
  double x;
  double y;
};

struct Mat3
{
  double v[3][3];
};

struct JointState3D
{
  // theta[2*j] = yaw, theta[2*j + 1] = pitch
  std::array<double, kNum3dJointDofs> theta{};
};

struct JointVelocity3D
{
  // theta_dot[2*j] = yaw rate, theta_dot[2*j + 1] = pitch rate
  std::array<double, kNum3dJointDofs> theta_dot{};
};

struct FrontUnitController3DParameters
{
  double link_length;
  double joint_rate_limit;
  double joint_limit;
  double max_curvature;
  double curvature_velocity_epsilon;
  double damping = 0.02;         // 阻尼最小二乘的 λ (8.40)
};

struct SimulationState3D
{
  double time{0.0};
  Vec3 head_position{0.0, 0.0, 0.0};
  Mat3 head_frame{};
  JointState3D joints{};
  std::array<Mat3, kNum3dLinks> link_frames{};
  std::array<Vec3, kNum3dLinks> link_axes{};
  std::array<Vec3, kNum3dPoints> body_points{};
};

Vec3 operator+(const Vec3 & a, const Vec3 & b);
Vec3 operator-(const Vec3 & a, const Vec3 & b);
Vec3 operator*(double s, const Vec3 & a);
Vec3 operator*(const Vec3 & a, double s);
Vec3 operator/(const Vec3 & a, double s);

double dot(const Vec3 & a, const Vec3 & b);
Vec3 cross(const Vec3 & a, const Vec3 & b);
double norm(const Vec3 & a);
Vec3 normalize(const Vec3 & a);
Vec3 projectPerpendicular(const Vec3 & value, const Vec3 & axis);
Vec3 column(const Mat3 & m, size_t j);
Mat3 fromColumns(const Vec3 & c0, const Vec3 & c1, const Vec3 & c2);
Mat3 identityFrame();
Mat3 multiply(const Mat3 & a, const Mat3 & b);
Vec3 multiply(const Mat3 & m, const Vec3 & a);
Mat3 rotationY(double angle);
Mat3 rotationZ(double angle);
Mat3 frameFromAxis(const Vec3 & axis);
Mat3 orthonormalize(const Mat3 & frame);

std::array<Mat3, kNum3dLinks> linkFrames(
  const Mat3 & head_frame, const std::array<double, kNum3dJointDofs> & theta);
std::array<Vec3, kNum3dLinks> linkAxes(const std::array<Mat3, kNum3dLinks> & frames);
std::array<Vec3, kNum3dPoints> bodyPoints(
  const Vec3 & head_point, const std::array<Vec3, kNum3dLinks> & axes, double link_length);

double saturate(double value, double limit);

class FrontUnitFollowingController3D
{
public:
  explicit FrontUnitFollowingController3D(const FrontUnitController3DParameters & params);

  SimulationState3D makeInitialState() const;
  asr_sdm_control_msgs::msg::RobotCommand limitCommand(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd) const;
  JointVelocity3D computeJointVelocity(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, const SimulationState3D & state) const;
  JointVelocity3D step(
    const asr_sdm_control_msgs::msg::RobotCommand & cmd, double dt, SimulationState3D & state) const;

private:
  static void pitchYawJacobianColumns(
    const Mat3 & frame, double upstreamPitch, Vec3 & jp, Vec3 & jy);
  Vec2 computePitchYawRateReference(
    const Mat3 & downstreamFrame, double upstreamPitch,
    const Vec3 & downstreamAxis, const Vec3 & frontJointVel,
    const Vec3 & upstreamOmega, double lambda) const;

  FrontUnitController3DParameters params_;
};

}  // namespace asr

#endif  // FRONT_UNIT_FOLLOWING_CONTROLLER_3D_HPP_

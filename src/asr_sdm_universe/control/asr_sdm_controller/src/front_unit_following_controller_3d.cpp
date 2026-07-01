#include "asr_sdm_controller/front_unit_following_controller_3d.hpp"

#include <algorithm>
#include <cmath>

namespace asr
{
namespace
{

size_t pitchIndex(size_t joint)
{
  return 2 * joint;
}

size_t yawIndex(size_t joint)
{
  return 2 * joint + 1;
}

}  // namespace

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(double s, const Vec3 & a)
{
  return {s * a.x, s * a.y, s * a.z};
}

Vec3 operator*(const Vec3 & a, double s)
{
  return s * a;
}

Vec3 operator/(const Vec3 & a, double s)
{
  return {a.x / s, a.y / s, a.z / s};
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x};
}

double norm(const Vec3 & a)
{
  return std::sqrt(dot(a, a));
}

Vec3 normalize(const Vec3 & a)
{
  const double n = norm(a);
  if (n < 1.0e-12) {
    return {1.0, 0.0, 0.0};
  }
  return a / n;
}

Vec3 projectPerpendicular(const Vec3 & value, const Vec3 & axis)
{
  return value - axis * dot(axis, value);
}

Vec3 column(const Mat3 & m, size_t j)
{
  return {m.v[0][j], m.v[1][j], m.v[2][j]};
}

Mat3 fromColumns(const Vec3 & c0, const Vec3 & c1, const Vec3 & c2)
{
  return {{{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}}};
}

Mat3 identityFrame()
{
  return fromColumns({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
}

Mat3 multiply(const Mat3 & a, const Mat3 & b)
{
  Mat3 out{};
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        out.v[i][j] += a.v[i][k] * b.v[k][j];
      }
    }
  }
  return out;
}

Vec3 multiply(const Mat3 & m, const Vec3 & a)
{
  return {
    m.v[0][0] * a.x + m.v[0][1] * a.y + m.v[0][2] * a.z,
    m.v[1][0] * a.x + m.v[1][1] * a.y + m.v[1][2] * a.z,
    m.v[2][0] * a.x + m.v[2][1] * a.y + m.v[2][2] * a.z};
}

Mat3 rotationY(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Mat3 rotationZ(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 frameFromAxis(const Vec3 & axis)
{
  const Vec3 ex = normalize(axis);
  const Vec3 reference = std::abs(dot(ex, {0.0, 0.0, 1.0})) > 0.92 ? Vec3{0.0, 1.0, 0.0} :
                                                                   Vec3{0.0, 0.0, 1.0};
  const Vec3 ey = normalize(cross(reference, ex));
  const Vec3 ez = cross(ex, ey);
  return fromColumns(ex, ey, ez);
}

Mat3 orthonormalize(const Mat3 & frame)
{
  const Vec3 ex = normalize(column(frame, 0));
  Vec3 ey = projectPerpendicular(column(frame, 1), ex);
  if (norm(ey) < 1.0e-9) {
    return frameFromAxis(ex);
  }

  ey = normalize(ey);
  const Vec3 ez = cross(ex, ey);
  return fromColumns(ex, ey, ez);
}

std::array<Mat3, kNum3dLinks> linkFrames(
  const Mat3 & head_frame, const std::array<double, kNum3dJointDofs> & theta)
{
  std::array<Mat3, kNum3dLinks> frames{};
  frames[0] = head_frame;
  for (size_t i = 0; i < kNum3dJoints; ++i) {
    frames[i + 1] =
      multiply(multiply(frames[i], rotationZ(-theta[yawIndex(i)])), rotationY(-theta[pitchIndex(i)]));
  }
  return frames;
}

std::array<Vec3, kNum3dLinks> linkAxes(const std::array<Mat3, kNum3dLinks> & frames)
{
  std::array<Vec3, kNum3dLinks> axes{};
  for (size_t i = 0; i < kNum3dLinks; ++i) {
    axes[i] = column(frames[i], 0);
  }
  return axes;
}

std::array<Vec3, kNum3dPoints> bodyPoints(
  const Vec3 & head_point, const std::array<Vec3, kNum3dLinks> & axes, double link_length)
{
  std::array<Vec3, kNum3dPoints> points{};
  points[0] = head_point;
  for (size_t i = 0; i < kNum3dLinks; ++i) {
    points[i + 1] = points[i] - link_length * axes[i];
  }
  return points;
}

double saturate(double value, double limit)
{
  return std::max(-limit, std::min(value, limit));
}

FrontUnitFollowingController3D::FrontUnitFollowingController3D(
  const FrontUnitController3DParameters & params)
: params_(params)
{
}

SimulationState3D FrontUnitFollowingController3D::makeInitialState() const
{
  SimulationState3D state;
  state.time = 0.0;
  state.head_position = {0.0, 0.0, 0.0};
  state.head_frame = identityFrame();
  state.joints.theta.fill(0.0);
  state.link_frames = linkFrames(state.head_frame, state.joints.theta);
  state.link_axes = linkAxes(state.link_frames);
  state.body_points = bodyPoints(state.head_position, state.link_axes, params_.link_length);
  return state;
}

HeadCommand3D FrontUnitFollowingController3D::limitCommand(const HeadCommand3D & cmd) const
{
  HeadCommand3D limited_cmd = cmd;
  const double omega = std::hypot(cmd.pitch_rate, cmd.yaw_rate);
  if (omega < 1.0e-12) {
    return limited_cmd;
  }

  const double max_curvature = std::max(0.0, params_.max_curvature);
  const double velocity_epsilon = std::max(std::abs(params_.curvature_velocity_epsilon), 1.0e-12);
  const double omega_limit = max_curvature * std::max(std::abs(cmd.linear_velocity), velocity_epsilon);
  if (omega <= omega_limit) {
    return limited_cmd;
  }

  const double scale = omega_limit / omega;
  limited_cmd.pitch_rate *= scale;
  limited_cmd.yaw_rate *= scale;
  return limited_cmd;
}

void FrontUnitFollowingController3D::pitchYawJacobianColumns(
  const Mat3 & frame, double upstreamPitch, Vec3 & jp, Vec3 & jy)
{
  jp = column(frame, 1);
  jy = std::sin(upstreamPitch) * column(frame, 0) + std::cos(upstreamPitch) * column(frame, 2);
}

Vec2 FrontUnitFollowingController3D::computePitchYawRateReference(
  const Mat3 & downstreamFrame, double upstreamPitch,
  const Vec3 & downstreamAxis, const Vec3 & frontJointVel,
  const Vec3 & upstreamOmega, double lambda) const
{
  // (8.37) 俯仰/偏航瞬时关节轴
  Vec3 jp, jy;
  pitchYawJacobianColumns(downstreamFrame, upstreamPitch, jp, jy);

  // (8.38) P⊥ = I − b bᵀ 投影算子
  auto Pperp = [&](const Vec3 & v) { return v - downstreamAxis * dot(downstreamAxis, v); };

  // (8.36) 期望垂直角速度 ω⊥_d = (2/L) b × ṗ_{j(i-1)}
  const Vec3 omegaPerpDes = (2.0 / params_.link_length) * cross(downstreamAxis, frontJointVel);

  // (8.39) RHS = ω⊥_d − P⊥·ω_{i-1}  无轴对齐反馈项
  const Vec3 rhs = omegaPerpDes - Pperp(upstreamOmega);

  // A = [P⊥·jp  P⊥·jy] (3×2)
  const Vec3 a0 = Pperp(jp);
  const Vec3 a1 = Pperp(jy);

  // (8.40) 阻尼最小二乘 q̇ = (AᵀA + λ²I)⁻¹ Aᵀ·RHS
  const double a00 = dot(a0, a0) + lambda * lambda;
  const double a01 = dot(a0, a1);
  const double a11 = dot(a1, a1) + lambda * lambda;
  const double b0 = dot(a0, rhs);
  const double b1 = dot(a1, rhs);
  const double det = a00 * a11 - a01 * a01;
  if (std::abs(det) < 1e-12) {
    return {0.0, 0.0};
  }

  const double pitchRate = (a11 * b0 - a01 * b1) / det;
  const double yawRate = (-a01 * b0 + a00 * b1) / det;
  return {pitchRate, yawRate};
}

JointVelocity3D FrontUnitFollowingController3D::step(
  const HeadCommand3D & cmd, double dt, SimulationState3D & state) const
{
  const HeadCommand3D limited_cmd = limitCommand(cmd);
  JointVelocity3D output;

  // 1. 用当前构型计算头部速度和链节速度传播
  const Mat3 head_frame = state.head_frame;
  state.link_frames = linkFrames(head_frame, state.joints.theta);
  state.link_axes = linkAxes(state.link_frames);
  const Vec3 head_axis = column(head_frame, 0);
  const Vec3 head_velocity = limited_cmd.linear_velocity * head_axis;
  const Vec3 headOmega = limited_cmd.yaw_rate * column(head_frame, 2) +
                         limited_cmd.pitch_rate * column(head_frame, 1);

  // 2. 速度传播初始化
  std::array<Vec3, kNum3dLinks + 1> jointPointVel{};  // ṗ_{j(i)}: kNum3dLinks+1 = 5 个点（含超出）
  std::array<Vec3, kNum3dLinks> linkOmega{};            // ω_i

  jointPointVel[0] = head_velocity;
  linkOmega[0] = headOmega;
  jointPointVel[1] = jointPointVel[0] - params_.link_length * cross(linkOmega[0], state.link_axes[0]);

  // 4. 关节循环：computePitchYawRateReference → saturate → 传播 → 积分
  for (size_t j = 0; j < kNum3dJoints; ++j) {
    const size_t ds = j + 1;  // 下游 link 索引

    const Vec2 rate = computePitchYawRateReference(
      state.link_frames[ds], state.joints.theta[pitchIndex(j)],
      state.link_axes[ds],
      jointPointVel[j + 1],
      linkOmega[j],
      params_.damping);

    const double pitchRate = saturate(rate.x, params_.joint_rate_limit);
    const double yawRate = saturate(rate.y, params_.joint_rate_limit);
    output.theta_dot[pitchIndex(j)] = -pitchRate;
    output.theta_dot[yawIndex(j)] = -yawRate;

    // 角速度传播 ω_{i} = ω_{i-1} + pitchRate*jp + yawRate*jy
    Vec3 jp, jy;
    pitchYawJacobianColumns(state.link_frames[ds], state.joints.theta[pitchIndex(j)], jp, jy);
    linkOmega[ds] = linkOmega[j] + pitchRate * jp + yawRate * jy;

    // 关节速度传播 ṗ_{j(i+1)} = ṗ_{j(i)} − L·(ω_i × b_i)
    if (ds < kNum3dLinks) {
      jointPointVel[ds + 1] = jointPointVel[j + 1] - params_.link_length * cross(linkOmega[ds], state.link_axes[ds]);
    }

    // 关节积分 theta -= rate*dt（与 Rz(-θ_yaw)Ry(-θ_pitch) 同号）
    const double jointLimit = std::abs(params_.joint_limit);
    state.joints.theta[pitchIndex(j)] = saturate(
      state.joints.theta[pitchIndex(j)] - pitchRate * dt, jointLimit);
    state.joints.theta[yawIndex(j)] = saturate(
      state.joints.theta[yawIndex(j)] - yawRate * dt, jointLimit);
  }

  // 5. 头部积分并刷新几何
  state.head_position = state.head_position + head_velocity * dt;
  state.head_frame = orthonormalize(
    multiply(
      multiply(state.head_frame, rotationZ(limited_cmd.yaw_rate * dt)),
      rotationY(limited_cmd.pitch_rate * dt)));
  state.time += dt;
  state.link_frames = linkFrames(state.head_frame, state.joints.theta);
  state.link_axes = linkAxes(state.link_frames);
  state.body_points = bodyPoints(state.head_position, state.link_axes, params_.link_length);

  return output;
}

}  // namespace asr

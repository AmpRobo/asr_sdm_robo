#include "asr_sdm_controller/front_unit_following_controller_3d.hpp"

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include "asr_sdm_control_msgs/msg/unit_cmd.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

using namespace std::chrono_literals;

namespace
{

constexpr double pi_value = 3.14159265358979323846;
constexpr size_t kStateSize3D = 43;
constexpr double kUrdfJointLimit = 1.5707963267948966;

const std::array<std::string, 6> kArticulationJointNames = {
  "joint_joint_unit_a__joint_unit_cross__0",
  "joint_joint_unit_cross__joint_unit_b__0",
  "joint_joint_unit_a__joint_unit_cross__1",
  "joint_joint_unit_cross__joint_unit_b__1",
  "joint_joint_unit_a__joint_unit_cross__2",
  "joint_joint_unit_cross__joint_unit_b__2",
};

const std::array<std::string, 8> kRotorJointNames = {
  "joint_screwdrive_segment_0__screw_rotor_left_0__0",
  "joint_screwdrive_segment_0__screw_rotor_right_0__0",
  "joint_screwdrive_segment_0__screw_rotor_left_0__1",
  "joint_screwdrive_segment_0__screw_rotor_right_0__1",
  "joint_screwdrive_segment_0__screw_rotor_left_0__2",
  "joint_screwdrive_segment_0__screw_rotor_right_0__2",
  "joint_screwdrive_segment_0__screw_rotor_left_0__3",
  "joint_screwdrive_segment_0__screw_rotor_right_0__3",
};

int32_t scale_to_int32(double value, double scale, int32_t min_value, int32_t max_value)
{
  const double scaled = value * scale;
  const double clamped = std::clamp(
    scaled, static_cast<double>(min_value), static_cast<double>(max_value));
  return static_cast<int32_t>(std::lround(clamped));
}

geometry_msgs::msg::Quaternion quaternionFromFrame(const asr::Mat3 & frame)
{
  const double trace = frame.v[0][0] + frame.v[1][1] + frame.v[2][2];
  geometry_msgs::msg::Quaternion quaternion;
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    quaternion.w = 0.25 * scale;
    quaternion.x = (frame.v[2][1] - frame.v[1][2]) / scale;
    quaternion.y = (frame.v[0][2] - frame.v[2][0]) / scale;
    quaternion.z = (frame.v[1][0] - frame.v[0][1]) / scale;
  } else if (frame.v[0][0] > frame.v[1][1] && frame.v[0][0] > frame.v[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[0][0] - frame.v[1][1] - frame.v[2][2]);
    quaternion.w = (frame.v[2][1] - frame.v[1][2]) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (frame.v[0][1] + frame.v[1][0]) / scale;
    quaternion.z = (frame.v[0][2] + frame.v[2][0]) / scale;
  } else if (frame.v[1][1] > frame.v[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[1][1] - frame.v[0][0] - frame.v[2][2]);
    quaternion.w = (frame.v[0][2] - frame.v[2][0]) / scale;
    quaternion.x = (frame.v[0][1] + frame.v[1][0]) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (frame.v[1][2] + frame.v[2][1]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + frame.v[2][2] - frame.v[0][0] - frame.v[1][1]);
    quaternion.w = (frame.v[1][0] - frame.v[0][1]) / scale;
    quaternion.x = (frame.v[0][2] + frame.v[2][0]) / scale;
    quaternion.y = (frame.v[1][2] + frame.v[2][1]) / scale;
    quaternion.z = 0.25 * scale;
  }
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

}  // namespace

class RealtimeFrontUnitController3DNode : public rclcpp::Node
{
public:
  RealtimeFrontUnitController3DNode()
  : Node("realtime_front_unit_controller_3d_" + std::to_string(getpid())),
    controller_(makeControllerParameters())
  {
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/asr_sdm/cmd_vel");
    controller_state_topic_ = this->declare_parameter<std::string>(
      "controller_state_topic", "/asr_sdm/controller_state_3d");
    control_cmd_topic_ = this->declare_parameter<std::string>(
      "control_cmd_topic", "/asr_sdm/control_cmd_3d");
    initialpose_topic_ = this->declare_parameter<std::string>(
      "initialpose_topic", "/asr_sdm/initialpose");
    initial_x_ = this->declare_parameter<double>("initial_x", -5.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_z_ = this->declare_parameter<double>("initial_z", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/asr_sdm/odom");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    controller_base_frame_ = this->declare_parameter<std::string>(
      "controller_base_frame", "asr_sdm_controller_base");
    root_frame_ = this->declare_parameter<std::string>("root_frame", "screwdrive_segment_0");
    joint_source_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "joint_source_indices", {0, 1, 2, 3, 4, 5});
    joint_signs_ = this->declare_parameter<std::vector<double>>(
      "joint_signs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    joint_offsets_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_offsets_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    clip_joint_positions_ = this->declare_parameter<bool>("clip_joint_positions", true);
    joint_position_limit_rad_ = this->declare_parameter<double>(
      "joint_position_limit_rad", kUrdfJointLimit);
    model_yaw_offset_rad_ = this->declare_parameter<double>("model_yaw_offset_rad", pi_value);
    model_pitch_offset_rad_ = this->declare_parameter<double>(
      "model_pitch_offset_rad", pi_value / 2.0);
    control_period_ms_ = this->declare_parameter<int>("control_period_ms", 20);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.3);

    max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.12);
    max_pitch_rate_ = this->declare_parameter<double>("max_pitch_rate", 0.35);
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 0.35);
    publish_control_cmd_ = this->declare_parameter<bool>("publish_control_cmd", false);
    joint_angle_scale_ = this->declare_parameter<double>("joint_angle_scale", 1.0);
    joint_angle_limit_ = this->declare_parameter<int>("joint_angle_limit", 2147483647);

    if (
      joint_source_indices_.size() != kArticulationJointNames.size() ||
      joint_signs_.size() != kArticulationJointNames.size() ||
      joint_offsets_rad_.size() != kArticulationJointNames.size()) {
      throw std::runtime_error(
              "joint_source_indices, joint_signs, and joint_offsets_rad must each contain six values");
    }
    for (const auto index : joint_source_indices_) {
      if (index < 0 || index >= static_cast<int64_t>(asr::kNum3dJointDofs)) {
        throw std::runtime_error("joint_source_indices values must be in [0, 5]");
      }
    }

    state_ = controller_.makeInitialState();
    link_length_ = get_parameter("link_length").as_double();
    resetState(initial_x_, initial_y_, initial_z_, initial_yaw_);
    last_cmd_time_ = this->now();
    last_control_time_ = this->now();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&RealtimeFrontUnitController3DNode::onTwist, this, std::placeholders::_1));
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10),
      std::bind(&RealtimeFrontUnitController3DNode::onInitialPose, this, std::placeholders::_1));
    pub_controller_state_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      controller_state_topic_, rclcpp::QoS(10));
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(10));
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishModelCalibrationTransform();
    if (publish_control_cmd_) {
      pub_control_cmd_ = this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>(
        control_cmd_topic_, rclcpp::QoS(1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&RealtimeFrontUnitController3DNode::onControlTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Realtime 3D controller started: cmd_vel=%s, state=%s, odom=%s, dynamic TF %s->%s, static calibration %s->%s (yaw=%.3f pitch=%.3f)",
      cmd_vel_topic_.c_str(), controller_state_topic_.c_str(), odom_topic_.c_str(), world_frame_.c_str(),
      controller_base_frame_.c_str(), controller_base_frame_.c_str(), root_frame_.c_str(),
      model_yaw_offset_rad_, model_pitch_offset_rad_);
  }

private:
  asr::FrontUnitController3DParameters makeControllerParameters()
  {
    asr::FrontUnitController3DParameters params;
    params.link_length = this->declare_parameter<double>("link_length", 0.25);
    params.joint_rate_limit = this->declare_parameter<double>("joint_rate_limit", 2.0);
    params.joint_limit = this->declare_parameter<double>("joint_limit", 0.85 * pi_value);
    params.max_curvature = this->declare_parameter<double>("max_curvature", 1.2);
    params.curvature_velocity_epsilon =
      this->declare_parameter<double>("curvature_velocity_epsilon", 1.0e-3);
    params.damping = this->declare_parameter<double>("damping", 0.02);
    return params;
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_.linear_velocity = std::clamp(msg->linear.x, 0.0, std::abs(max_linear_velocity_));
    latest_cmd_.pitch_rate = clampSymmetric(msg->angular.y, max_pitch_rate_);
    latest_cmd_.yaw_rate = clampSymmetric(msg->angular.z, max_yaw_rate_);
    last_cmd_time_ = this->now();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel limited -> v=%.3f pitch=%.3f yaw=%.3f", latest_cmd_.linear_velocity,
      latest_cmd_.pitch_rate, latest_cmd_.yaw_rate);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto & orientation = msg->pose.pose.orientation;
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (norm < 1.0e-9 || !std::isfinite(norm)) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose with an invalid quaternion");
      return;
    }

    resetState(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
      orientation.x / norm, orientation.y / norm, orientation.z / norm, orientation.w / norm);
    last_cmd_time_ = now();
    last_control_time_ = now();
    publishControllerState({0.0, 0.0, 0.0}, {});
    publishRobotState({0.0, 0.0, 0.0}, {});
    RCLCPP_INFO(
      get_logger(), "Reset 3D controller to (%.3f, %.3f, %.3f)", state_.head_position.x,
      state_.head_position.y, state_.head_position.z);
  }

  void resetState(double x, double y, double z, double yaw)
  {
    resetState(x, y, z, 0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0));
  }

  void resetState(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    state_ = controller_.makeInitialState();
    state_.head_position = {x, y, z};
    state_.head_frame = {{{
      1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)},
      {2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)},
      {2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)}}};
    state_.link_frames = asr::linkFrames(state_.head_frame, state_.joints.theta);
    state_.link_axes = asr::linkAxes(state_.link_frames);
    state_.body_points = asr::bodyPoints(state_.head_position, state_.link_axes, link_length_);
    latest_cmd_ = {0.0, 0.0, 0.0};
    latest_joint_velocity_ = {};
  }

  double clampSymmetric(double value, double limit) const
  {
    const double abs_limit = std::abs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
  }

  bool isZeroCommand(const asr::HeadCommand3D & cmd) const
  {
    return std::abs(cmd.linear_velocity) < 1.0e-9 && std::abs(cmd.pitch_rate) < 1.0e-9 &&
      std::abs(cmd.yaw_rate) < 1.0e-9;
  }

  void onControlTimer()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      dt = static_cast<double>(control_period_ms_) / 1000.0;
    }
    last_control_time_ = now;

    asr::HeadCommand3D cmd = latest_cmd_;
    if ((now - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      cmd = {0.0, 0.0, 0.0};
    }
    cmd = controller_.limitCommand(cmd);

    latest_joint_velocity_ = controller_.step(cmd, dt, state_);
    if (isZeroCommand(cmd)) {
      latest_joint_velocity_ = {};
    }
    publishControllerState(cmd, latest_joint_velocity_);
    publishRobotState(cmd, latest_joint_velocity_);
    if (publish_control_cmd_ && pub_control_cmd_) {
      publishControlCmd();
    }
  }

  void publishControllerState(
    const asr::HeadCommand3D & cmd, const asr::JointVelocity3D & joint_velocity)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kStateSize3D, 0.0);
    msg.data[0] = state_.time;
    msg.data[1] = state_.head_position.x;
    msg.data[2] = state_.head_position.y;
    msg.data[3] = state_.head_position.z;

    size_t index = 4;
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        msg.data[index++] = state_.head_frame.v[row][col];
      }
    }

    msg.data[13] = cmd.linear_velocity;
    msg.data[14] = cmd.pitch_rate;
    msg.data[15] = cmd.yaw_rate;
    // data[16..21] and data[22..27] are physical [yaw, pitch] pairs.
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      msg.data[16 + i] = state_.joints.theta[i];
      msg.data[22 + i] = joint_velocity.theta_dot[i];
    }
    for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
      msg.data[28 + 3 * point] = state_.body_points[point].x;
      msg.data[28 + 3 * point + 1] = state_.body_points[point].y;
      msg.data[28 + 3 * point + 2] = state_.body_points[point].z;
    }

    pub_controller_state_->publish(msg);
  }

  void publishModelCalibrationTransform()
  {
    const auto calibration_frame = asr::multiply(
      asr::rotationZ(model_yaw_offset_rad_), asr::rotationY(model_pitch_offset_rad_));
    geometry_msgs::msg::TransformStamped calibration;
    calibration.header.stamp = now();
    calibration.header.frame_id = controller_base_frame_;
    calibration.child_frame_id = root_frame_;
    calibration.transform.rotation = quaternionFromFrame(calibration_frame);
    static_tf_broadcaster_->sendTransform(calibration);
  }

  void publishRobotState(
    const asr::HeadCommand3D & cmd, const asr::JointVelocity3D & joint_velocity)
  {
    const auto stamp = now();
    const auto odom_orientation = quaternionFromFrame(state_.head_frame);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    joint_state.position.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    joint_state.velocity.reserve(kArticulationJointNames.size() + kRotorJointNames.size());
    for (size_t urdf_index = 0; urdf_index < kArticulationJointNames.size(); ++urdf_index) {
      const size_t source_index = static_cast<size_t>(joint_source_indices_[urdf_index]);
      double position = joint_signs_[urdf_index] * state_.joints.theta[source_index] +
                        joint_offsets_rad_[urdf_index];
      if (clip_joint_positions_) {
        position = std::clamp(
          position, -std::abs(joint_position_limit_rad_), std::abs(joint_position_limit_rad_));
      }
      joint_state.name.push_back(kArticulationJointNames[urdf_index]);
      joint_state.position.push_back(position);
      joint_state.velocity.push_back(
        joint_signs_[urdf_index] * joint_velocity.theta_dot[source_index]);
    }
    for (const auto & name : kRotorJointNames) {
      joint_state.name.push_back(name);
      joint_state.position.push_back(0.0);
      joint_state.velocity.push_back(0.0);
    }
    pub_joint_state_->publish(joint_state);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = controller_base_frame_;
    odometry.pose.pose.position.x = state_.head_position.x;
    odometry.pose.pose.position.y = state_.head_position.y;
    odometry.pose.pose.position.z = state_.head_position.z;
    odometry.pose.pose.orientation = odom_orientation;
    const auto forward = asr::column(state_.head_frame, 0);
    const auto pitch_axis = asr::column(state_.head_frame, 1);
    const auto yaw_axis = asr::column(state_.head_frame, 2);
    odometry.twist.twist.linear.x = cmd.linear_velocity * forward.x;
    odometry.twist.twist.linear.y = cmd.linear_velocity * forward.y;
    odometry.twist.twist.linear.z = cmd.linear_velocity * forward.z;
    odometry.twist.twist.angular.x = cmd.pitch_rate * pitch_axis.x + cmd.yaw_rate * yaw_axis.x;
    odometry.twist.twist.angular.y = cmd.pitch_rate * pitch_axis.y + cmd.yaw_rate * yaw_axis.y;
    odometry.twist.twist.angular.z = cmd.pitch_rate * pitch_axis.z + cmd.yaw_rate * yaw_axis.z;
    pub_odom_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = controller_base_frame_;
    transform.transform.translation.x = state_.head_position.x;
    transform.transform.translation.y = state_.head_position.y;
    transform.transform.translation.z = state_.head_position.z;
    transform.transform.rotation = odom_orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publishControlCmd()
  {
    asr_sdm_control_msgs::msg::ControlCmd msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "front_unit_following_controller_3d";
    msg.units_cmd.resize(asr::kNum3dJoints);

    for (size_t joint = 0; joint < asr::kNum3dJoints; ++joint) {
      auto & unit = msg.units_cmd[joint];
      unit.unit_id = static_cast<int32_t>(joint);
      unit.screw1_vel = 0;
      unit.screw2_vel = 0;
      // UnitCmd follows the physical serial order: front yaw, then rear pitch.
      unit.joint1_angle = scaledJointAngle(asr::yawIndex(joint));
      unit.joint2_angle = scaledJointAngle(asr::pitchIndex(joint));
    }

    pub_control_cmd_->publish(msg);
  }

  int32_t scaledJointAngle(size_t index) const
  {
    const double value = index < state_.joints.theta.size() ? state_.joints.theta[index] : 0.0;
    return scale_to_int32(value, joint_angle_scale_, -joint_angle_limit_, joint_angle_limit_);
  }

  std::string cmd_vel_topic_;
  std::string controller_state_topic_;
  std::string control_cmd_topic_;
  std::string initialpose_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string world_frame_;
  std::string controller_base_frame_;
  std::string root_frame_;
  std::vector<int64_t> joint_source_indices_;
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_rad_;
  bool clip_joint_positions_;
  double joint_position_limit_rad_;
  double model_yaw_offset_rad_;
  double model_pitch_offset_rad_;
  double initial_x_;
  double initial_y_;
  double initial_z_;
  double initial_yaw_;
  double link_length_{0.25};
  int control_period_ms_;
  double cmd_timeout_sec_;
  double max_linear_velocity_;
  double max_pitch_rate_;
  double max_yaw_rate_;
  bool publish_control_cmd_;
  double joint_angle_scale_;
  int joint_angle_limit_;

  asr::FrontUnitFollowingController3D controller_;
  asr::SimulationState3D state_;
  asr::HeadCommand3D latest_cmd_{0.0, 0.0, 0.0};
  asr::JointVelocity3D latest_joint_velocity_{};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeFrontUnitController3DNode>());
  rclcpp::shutdown();
  return 0;
}

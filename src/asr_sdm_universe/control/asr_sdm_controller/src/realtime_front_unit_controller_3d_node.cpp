#include "asr_sdm_controller/front_unit_following_controller_3d.hpp"

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include "asr_sdm_control_msgs/msg/unit_cmd.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <unistd.h>

using namespace std::chrono_literals;

namespace
{

constexpr double pi_value = 3.14159265358979323846;
constexpr size_t kStateSize3D = 43;

int32_t scale_to_int32(double value, double scale, int32_t min_value, int32_t max_value)
{
  const double scaled = value * scale;
  const double clamped = std::clamp(
    scaled, static_cast<double>(min_value), static_cast<double>(max_value));
  return static_cast<int32_t>(std::lround(clamped));
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
    control_period_ms_ = this->declare_parameter<int>("control_period_ms", 20);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.3);

    max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.12);
    max_pitch_rate_ = this->declare_parameter<double>("max_pitch_rate", 0.35);
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 0.35);
    publish_control_cmd_ = this->declare_parameter<bool>("publish_control_cmd", false);
    joint_angle_scale_ = this->declare_parameter<double>("joint_angle_scale", 1.0);
    joint_angle_limit_ = this->declare_parameter<int>("joint_angle_limit", 2147483647);

    state_ = controller_.makeInitialState();
    last_cmd_time_ = this->now();
    last_control_time_ = this->now();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&RealtimeFrontUnitController3DNode::onTwist, this, std::placeholders::_1));
    pub_controller_state_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      controller_state_topic_, rclcpp::QoS(10));
    if (publish_control_cmd_) {
      pub_control_cmd_ = this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>(
        control_cmd_topic_, rclcpp::QoS(1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&RealtimeFrontUnitController3DNode::onControlTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Realtime 3D controller started: cmd_vel=%s, state=%s, control_cmd=%s, publish_control_cmd=%s",
      cmd_vel_topic_.c_str(), controller_state_topic_.c_str(), control_cmd_topic_.c_str(),
      publish_control_cmd_ ? "true" : "false");
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
      unit.joint1_angle = scaledJointAngle(2 * joint);
      unit.joint2_angle = scaledJointAngle(2 * joint + 1);
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeFrontUnitController3DNode>());
  rclcpp::shutdown();
  return 0;
}

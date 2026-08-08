#include "asr_sdm_control_manager/front_unit_following_controller_2d.hpp"
#include "asr_sdm_control_manager/robot_model.hpp"

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
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr double pi_value = 3.14159265358979323846;

struct Pose2D
{
  double x;
  double y;
  double psi;
};

double clamp(double value, double limit)
{
  return std::max(-limit, std::min(value, limit));
}

int32_t scale_to_int32(double value, double scale, int32_t min_value, int32_t max_value)
{
  const double scaled = value * scale;
  const double clamped = std::clamp(
    scaled, static_cast<double>(min_value), static_cast<double>(max_value));
  return static_cast<int32_t>(std::lround(clamped));
}

}  // namespace

class RealtimeFrontUnitControllerNode : public rclcpp::Node
{
public:
  RealtimeFrontUnitControllerNode()
  : Node("realtime_front_unit_controller"),
    params_{makeRobotParameters()},
    controller_(params_),
    robot_model_(params_)
  {
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/control/asr_sdm/cmd_vel");
    control_cmd_topic_ =
      this->declare_parameter<std::string>("control_cmd_topic", "/control/asr_sdm/control_cmd");
    controller_state_topic_ = this->declare_parameter<std::string>(
      "controller_state_topic", "/control/asr_sdm/controller_state");
    control_period_ms_ = this->declare_parameter<int>("control_period_ms", 20);
    phi_dot_limit_ = this->declare_parameter<double>("phi_dot_limit", 2.0);
    phi_limit_ = this->declare_parameter<double>("phi_limit", 0.85 * pi_value);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.3);
    screw_velocity_scale_ = this->declare_parameter<double>("screw_velocity_scale", 1.0);
    joint_angle_scale_ = this->declare_parameter<double>("joint_angle_scale", 1.0);
    screw_velocity_limit_ = this->declare_parameter<int>("screw_velocity_limit", 2147483647);
    joint_angle_limit_ = this->declare_parameter<int>("joint_angle_limit", 2147483647);

    state_.phi = {0.0, 0.0, 0.0};
    last_cmd_time_ = this->now();
    last_control_time_ = this->now();
    start_time_ = this->now();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1),
      std::bind(&RealtimeFrontUnitControllerNode::onTwist, this, std::placeholders::_1));
    pub_control_cmd_ = this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>(
      control_cmd_topic_, rclcpp::QoS(1));
    pub_controller_state_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      controller_state_topic_, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&RealtimeFrontUnitControllerNode::onControlTimer, this));

    RCLCPP_INFO(
      this->get_logger(), "Realtime controller started: cmd_vel=%s, control_cmd=%s, state=%s",
      cmd_vel_topic_.c_str(), control_cmd_topic_.c_str(), controller_state_topic_.c_str());
  }

private:
  asr::RobotParameters makeRobotParameters()
  {
    asr::RobotParameters params;
    params.link_length = this->declare_parameter<double>("link_length", 0.25);
    params.screw_radius = this->declare_parameter<double>("screw_radius", 0.075);
    params.alpha = {-M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0};
    return params;
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_.linear_velocity = msg->linear.x;
    latest_cmd_.angular_velocity = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void onControlTimer()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_control_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      dt = static_cast<double>(control_period_ms_) / 1000.0;
    }
    last_control_time_ = now;

    asr::HeadCommand cmd = latest_cmd_;
    if ((now - last_cmd_time_).seconds() > cmd_timeout_sec_) {
      cmd.linear_velocity = 0.0;
      cmd.angular_velocity = 0.0;
    }

    const asr::JointVelocity joint_vel = controller_.computeJointVelocity(cmd, state_);
    const asr::ScrewVelocity screw_vel = robot_model_.computeScrewVelocity(cmd, state_, joint_vel);

    for (size_t i = 0; i < state_.phi.size(); ++i) {
      const double phi_dot = i < joint_vel.phi_dot.size() ? joint_vel.phi_dot[i] : 0.0;
      state_.phi[i] = clamp(state_.phi[i] + clamp(phi_dot, phi_dot_limit_) * dt, phi_limit_);
    }

    head_pose_.x += cmd.linear_velocity * std::cos(head_pose_.psi) * dt;
    head_pose_.y += cmd.linear_velocity * std::sin(head_pose_.psi) * dt;
    head_pose_.psi += cmd.angular_velocity * dt;
    sim_time_ += dt;

    publishControlCmd(screw_vel);
    publishControllerState(cmd, joint_vel, screw_vel);
  }

  void publishControlCmd(const asr::ScrewVelocity & screw_vel)
  {
    asr_sdm_control_msgs::msg::ControlCmd msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "front_unit_following_controller";
    msg.units_cmd.resize(2);

    fillUnitCmd(msg.units_cmd[0], 0, screw_vel, 0, 0, 1);
    fillUnitCmd(msg.units_cmd[1], 1, screw_vel, 2, 2, std::numeric_limits<size_t>::max());

    pub_control_cmd_->publish(msg);
  }

  void fillUnitCmd(
    asr_sdm_control_msgs::msg::UnitCmd & unit, int32_t unit_id,
    const asr::ScrewVelocity & screw_vel, size_t screw_index, size_t joint1_index,
    size_t joint2_index) const
  {
    unit.unit_id = unit_id;
    unit.screw1_vel = scaledScrewVelocity(screw_vel, screw_index);
    unit.screw2_vel = scaledScrewVelocity(screw_vel, screw_index + 1);
    unit.joint1_angle = scaledJointAngle(joint1_index);
    unit.joint2_angle = scaledJointAngle(joint2_index);
  }

  int32_t scaledScrewVelocity(const asr::ScrewVelocity & screw_vel, size_t index) const
  {
    const double value = index < screw_vel.theta_dot.size() ? screw_vel.theta_dot[index] : 0.0;
    return scale_to_int32(value, screw_velocity_scale_, -screw_velocity_limit_, screw_velocity_limit_);
  }

  int32_t scaledJointAngle(size_t index) const
  {
    const double value = index < state_.phi.size() ? state_.phi[index] : 0.0;
    return scale_to_int32(value, joint_angle_scale_, -joint_angle_limit_, joint_angle_limit_);
  }

  void publishControllerState(
    const asr::HeadCommand & cmd, const asr::JointVelocity & joint_vel,
    const asr::ScrewVelocity & screw_vel)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(16, 0.0);
    msg.data[0] = sim_time_;
    msg.data[1] = head_pose_.x;
    msg.data[2] = head_pose_.y;
    msg.data[3] = head_pose_.psi;
    msg.data[4] = cmd.linear_velocity;
    msg.data[5] = cmd.angular_velocity;

    for (size_t i = 0; i < 3; ++i) {
      msg.data[6 + i] = i < state_.phi.size() ? state_.phi[i] : 0.0;
      msg.data[9 + i] = i < joint_vel.phi_dot.size() ? joint_vel.phi_dot[i] : 0.0;
    }
    for (size_t i = 0; i < 4; ++i) {
      msg.data[12 + i] = i < screw_vel.theta_dot.size() ? screw_vel.theta_dot[i] : 0.0;
    }

    pub_controller_state_->publish(msg);
  }

  std::string cmd_vel_topic_;
  std::string control_cmd_topic_;
  std::string controller_state_topic_;
  int control_period_ms_;
  double phi_dot_limit_;
  double phi_limit_;
  double cmd_timeout_sec_;
  double screw_velocity_scale_;
  double joint_angle_scale_;
  int screw_velocity_limit_;
  int joint_angle_limit_;

  asr::RobotParameters params_;
  asr::FrontUnitFollowingController controller_;
  asr::RobotModel robot_model_;
  asr::HeadCommand latest_cmd_{0.0, 0.0};
  asr::JointState state_;
  Pose2D head_pose_{0.0, 0.0, pi_value / 2.0};
  double sim_time_{0.0};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Time start_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_controller_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeFrontUnitControllerNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>

class JoystickTeleop : public rclcpp::Node
{
public:
  JoystickTeleop() : Node("joystick_teleop")
  {
    this->declare_parameter<int>("linear_axis", 1);
    this->declare_parameter<int>("angular_axis", 0);
    this->declare_parameter<int>("pitch_axis", 4);
    this->declare_parameter<double>("linear_scale", 0.12);
    this->declare_parameter<double>("angular_scale", 0.5);
    this->declare_parameter<double>("pitch_scale", 0.5);

    this->declare_parameter<bool>("use_trigger_linear", true);
    this->declare_parameter<int>("axis_l2", 2);
    this->declare_parameter<int>("axis_r2", 5);
    this->declare_parameter<double>("l2_released_value", 1.0);
    this->declare_parameter<double>("r2_released_value", 1.0);
    this->declare_parameter<double>("trigger_pressed_value", -1.0);
    this->declare_parameter<double>("deadzone", 0.08);
    this->declare_parameter<bool>("invert_linear", false);
    this->declare_parameter<bool>("invert_pitch", true);
    this->declare_parameter<bool>("invert_yaw", false);
    this->declare_parameter<std::string>("robot_cmd_topic", "/control/asr_sdm/robot_cmd");

    const auto robot_cmd_topic = this->get_parameter("robot_cmd_topic").as_string();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickTeleop::joy_callback, this, std::placeholders::_1));

    pub_control_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(robot_cmd_topic, 10);

    RCLCPP_INFO(
      this->get_logger(), "Joystick Teleop Node started. robot_cmd=%s", robot_cmd_topic.c_str());
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();

    const int linear_axis = this->get_parameter("linear_axis").as_int();
    const int angular_axis = this->get_parameter("angular_axis").as_int();
    const int pitch_axis = this->get_parameter("pitch_axis").as_int();
    const double linear_scale = this->get_parameter("linear_scale").as_double();
    const double angular_scale = this->get_parameter("angular_scale").as_double();
    const double pitch_scale = this->get_parameter("pitch_scale").as_double();
    const bool use_trigger_linear = this->get_parameter("use_trigger_linear").as_bool();
    const double deadzone = this->get_parameter("deadzone").as_double();

    if (use_trigger_linear) {
      twist.linear.x = triggerLinearVelocity(*msg, linear_scale, deadzone);
    } else {
      twist.linear.x = std::max(0.0, readAxis(*msg, linear_axis)) * linear_scale;
    }

    twist.angular.y = applyDeadzone(readAxis(*msg, pitch_axis), deadzone) * pitch_scale;
    const double yaw_axis_value = readAxis(*msg, angular_axis);
    twist.angular.z = (use_trigger_linear ? applyDeadzone(yaw_axis_value, deadzone) : yaw_axis_value) * angular_scale;

    if (this->get_parameter("invert_linear").as_bool()) {
      twist.linear.x = -twist.linear.x;
    }
    if (this->get_parameter("invert_pitch").as_bool()) {
      twist.angular.y = -twist.angular.y;
    }
    if (this->get_parameter("invert_yaw").as_bool()) {
      twist.angular.z = -twist.angular.z;
    }

    pub_control_cmd_->publish(twist);
  }

  double readAxis(const sensor_msgs::msg::Joy & joy, int axis) const
  {
    if (axis < 0 || static_cast<size_t>(axis) >= joy.axes.size()) {
      return 0.0;
    }
    return static_cast<double>(joy.axes[axis]);
  }

  double triggerLinearVelocity(
    const sensor_msgs::msg::Joy & joy, double linear_scale, double deadzone) const
  {
    const int axis_l2 = this->get_parameter("axis_l2").as_int();
    const int axis_r2 = this->get_parameter("axis_r2").as_int();
    const double l2_released_value = this->get_parameter("l2_released_value").as_double();
    const double r2_released_value = this->get_parameter("r2_released_value").as_double();
    const double trigger_pressed_value = this->get_parameter("trigger_pressed_value").as_double();

    const double l2 = normalizedTrigger(
      readAxisOrDefault(joy, axis_l2, l2_released_value), l2_released_value, trigger_pressed_value,
      deadzone);
    const double r2 = normalizedTrigger(
      readAxisOrDefault(joy, axis_r2, r2_released_value), r2_released_value, trigger_pressed_value,
      deadzone);
    return std::max(l2, r2) * linear_scale;
  }

  double readAxisOrDefault(const sensor_msgs::msg::Joy & joy, int axis, double default_value) const
  {
    if (axis < 0 || static_cast<size_t>(axis) >= joy.axes.size()) {
      return default_value;
    }
    return static_cast<double>(joy.axes[axis]);
  }

  double normalizedTrigger(
    double raw_axis, double released_value, double pressed_value, double deadzone) const
  {
    const double raw_delta = raw_axis - released_value;
    if (std::abs(raw_delta) < deadzone) {
      return 0.0;
    }

    if (std::abs(released_value) < 0.25) {
      return std::clamp(std::abs(raw_delta), 0.0, 1.0);
    }

    const double press_direction = std::abs(pressed_value - released_value) >= 1.0e-6 ?
      pressed_value - released_value : -released_value;
    const double press_delta = raw_delta * (press_direction >= 0.0 ? 1.0 : -1.0);
    const double press_range = std::max(std::abs(pressed_value - released_value), 1.0);
    return std::clamp(press_delta / press_range, 0.0, 1.0);
  }

  double applyDeadzone(double value, double deadzone) const
  {
    return std::abs(value) < deadzone ? 0.0 : value;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickTeleop>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <asr_sdm_control_msgs/msg/corrections.hpp>
#include <asr_sdm_control_msgs/msg/robot_command.hpp>
#include <asr_sdm_control_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.h>
#include <std_msgs/msg/bool.hpp>

namespace {
double getYaw(const geometry_msgs::msg::Quaternion& q)
{
  // yaw (z-axis rotation) from quaternion
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class SO3ControlComponent : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SO3ControlComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("so3_control", options)
    , position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , use_external_yaw_(false)
  {
    std::string quadrotor_name = this->declare_parameter("quadrotor_name", std::string("quadrotor"));
    frame_id_ = quadrotor_name;

    double mass = this->declare_parameter("mass", 0.5);
    controller_.setMass(mass);

    use_external_yaw_ = this->declare_parameter("use_external_yaw", true);

    kR_[0] = this->declare_parameter("gains.rot.x", 1.5);
    kR_[1] = this->declare_parameter("gains.rot.y", 1.5);
    kR_[2] = this->declare_parameter("gains.rot.z", 1.0);
    kOm_[0] = this->declare_parameter("gains.ang.x", 0.13);
    kOm_[1] = this->declare_parameter("gains.ang.y", 0.13);
    kOm_[2] = this->declare_parameter("gains.ang.z", 0.1);

    corrections_[0] = this->declare_parameter("corrections.z", 0.0);
    corrections_[1] = this->declare_parameter("corrections.r", 0.0);
    corrections_[2] = this->declare_parameter("corrections.p", 0.0);

    so3_command_pub_ = this->create_publisher<asr_sdm_control_msgs::msg::SO3Command>("so3_cmd", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SO3ControlComponent::odom_callback, this, std::placeholders::_1));
    position_cmd_sub_ = this->create_subscription<asr_sdm_control_msgs::msg::RobotCommand>(
      "position_cmd", 10, std::bind(&SO3ControlComponent::position_cmd_callback, this, std::placeholders::_1));
    enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "motors", 2, std::bind(&SO3ControlComponent::enable_motors_callback, this, std::placeholders::_1));
    corrections_sub_ = this->create_subscription<asr_sdm_control_msgs::msg::Corrections>(
      "corrections", 10, std::bind(&SO3ControlComponent::corrections_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&SO3ControlComponent::imu_callback, this, std::placeholders::_1));
  }

private:
  void publishSO3Command(void)
  {
    controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                                 des_yaw_dot_, kx_, kv_);

    const Eigen::Vector3d&    force       = controller_.getComputedForce();
    const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

    asr_sdm_control_msgs::msg::SO3Command so3_command;
    so3_command.header.stamp    = this->now();
    so3_command.header.frame_id = frame_id_;
    so3_command.force.x         = force(0);
    so3_command.force.y         = force(1);
    so3_command.force.z         = force(2);
    so3_command.orientation.x   = orientation.x();
    so3_command.orientation.y   = orientation.y();
    so3_command.orientation.z   = orientation.z();
    so3_command.orientation.w   = orientation.w();
    for (int i = 0; i < 3; i++)
    {
      so3_command.k_r[i]  = kR_[i];
      so3_command.k_om[i] = kOm_[i];
    }
    so3_command.aux.current_yaw          = current_yaw_;
    so3_command.aux.kf_correction        = corrections_[0];
    so3_command.aux.angle_corrections[0] = corrections_[1];
    so3_command.aux.angle_corrections[1] = corrections_[2];
    so3_command.aux.enable_motors        = enable_motors_;
    so3_command.aux.use_external_yaw     = use_external_yaw_;
    so3_command_pub_->publish(so3_command);
  }

  void position_cmd_callback(const asr_sdm_control_msgs::msg::RobotCommand::ConstSharedPtr cmd)
  {
    des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                               cmd->acceleration.z);
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

    des_yaw_              = cmd->yaw;
    des_yaw_dot_          = cmd->yaw_dot;
    position_cmd_updated_ = true;
    position_cmd_init_    = true;

    publishSO3Command();
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const Eigen::Vector3d position(odom->pose.pose.position.x,
                                   odom->pose.pose.position.y,
                                   odom->pose.pose.position.z);
    const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                   odom->twist.twist.linear.y,
                                   odom->twist.twist.linear.z);

    current_yaw_ = getYaw(odom->pose.pose.orientation);

    controller_.setPosition(position);
    controller_.setVelocity(velocity);

    if (position_cmd_init_)
    {
      if (!position_cmd_updated_)
        publishSO3Command();
      position_cmd_updated_ = false;
    }
  }

  void enable_motors_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data)
      RCLCPP_INFO(this->get_logger(), "Enabling motors");
    else
      RCLCPP_INFO(this->get_logger(), "Disabling motors");

    enable_motors_ = msg->data;
  }

  void corrections_callback(const asr_sdm_control_msgs::msg::Corrections::ConstSharedPtr msg)
  {
    corrections_[0] = msg->kf_correction;
    corrections_[1] = msg->angle_corrections[0];
    corrections_[2] = msg->angle_corrections[1];
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
  {
    const Eigen::Vector3d acc(imu->linear_acceleration.x,
                              imu->linear_acceleration.y,
                              imu->linear_acceleration.z);
    controller_.setAcc(acc);
  }

  SO3Control controller_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::RobotCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<asr_sdm_control_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;
  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SO3ControlComponent)

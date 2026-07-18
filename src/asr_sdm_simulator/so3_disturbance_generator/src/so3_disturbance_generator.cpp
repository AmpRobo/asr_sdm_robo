#include <iostream>
#include <string.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_utils.h"

using namespace arma;
using namespace std;

#define CORRECTION_RATE 1

// Runtime-adjustable configuration (previously dynamic_reconfigure).
struct DisturbanceUIConfig
{
  // External disturbance
  double fxy = 0.0, stdfxy = 0.0;
  double fz = 0.0, stdfz = 0.0;
  double mrp = 0.0, stdmrp = 0.0;
  double myaw = 0.0, stdmyaw = 0.0;
  // Measurement noise
  bool   enable_noisy_odom = false;
  double stdxyz = 0.0, stdvxyz = 0.0, stdrp = 0.0, stdyaw = 0.0;
  // Odom drifting
  bool   enable_drift_odom = true;
  double stdvdriftxyz = 0.0, stdvdriftyaw = 0.0;
  double vdriftx = 0.0, vdrifty = 0.0, vdriftz = 0.0, vdriftyaw = 0.0;
};

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubo;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubc;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubf;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubm;
DisturbanceUIConfig config;
nav_msgs::msg::Odometry noisy_odom;
geometry_msgs::msg::PoseStamped correction;

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  noisy_odom.header = msg->header;
  correction.header = msg->header;
  // Get odom
  colvec pose(6);
  colvec vel(3);
  pose(0)        = msg->pose.pose.position.x;
  pose(1)        = msg->pose.pose.position.y;
  pose(2)        = msg->pose.pose.position.z;
  colvec q       = zeros<colvec>(4);
  q(0)           = msg->pose.pose.orientation.w;
  q(1)           = msg->pose.pose.orientation.x;
  q(2)           = msg->pose.pose.orientation.y;
  q(3)           = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  vel(0)         = msg->twist.twist.linear.x;
  vel(1)         = msg->twist.twist.linear.y;
  vel(2)         = msg->twist.twist.linear.z;
  // Drift Odom
  static colvec drift_pose      = pose;
  static colvec drift_vel       = vel;
  static colvec correction_pose = zeros<colvec>(6);
  static colvec prev_pose       = pose;
  static rclcpp::Time prev_pose_t(msg->header.stamp);
  if (config.enable_drift_odom)
  {
    double dt       = (rclcpp::Time(msg->header.stamp) - prev_pose_t).seconds();
    prev_pose_t     = msg->header.stamp;
    colvec d        = pose_update(pose_inverse(prev_pose), pose);
    prev_pose       = pose;
    d(0)           += (config.vdriftx   + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(1)           += (config.vdrifty   + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(2)           += (config.vdriftz   + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(3)           += (config.vdriftyaw + config.stdvdriftyaw * as_scalar(randn(1))) * dt;
    drift_pose      = pose_update(drift_pose, d);
    drift_vel       = ypr_to_R(drift_pose.rows(3,5)) * trans(ypr_to_R(pose.rows(3,5))) * vel;
    correction_pose = pose_update(pose, pose_inverse(drift_pose));
  }
  else
  {
    drift_pose      = pose;
    drift_vel       = vel;
    correction_pose = zeros<colvec>(6);
  }
  // Noisy Odom
  static colvec noisy_pose = drift_pose;
  static colvec noisy_vel  = drift_vel;
  if (config.enable_noisy_odom)
  {
    colvec noise_pose = zeros<colvec>(6);
    colvec noise_vel  = zeros<colvec>(3);
    noise_pose(0) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(1) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(2) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(3) = config.stdyaw  * as_scalar(randn(1));
    noise_pose(4) = config.stdrp   * as_scalar(randn(1));
    noise_pose(5) = config.stdrp   * as_scalar(randn(1));
    noise_vel(0)  = config.stdvxyz * as_scalar(randn(1));
    noise_vel(1)  = config.stdvxyz * as_scalar(randn(1));
    noise_vel(2)  = config.stdvxyz * as_scalar(randn(1));
    noisy_pose = drift_pose + noise_pose;
    noisy_vel  = drift_vel  + noise_vel;
    noisy_odom.pose.covariance[0+0*6]         = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[1+1*6]         = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[2+2*6]         = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[(0+3)+(0+3)*6] = config.stdyaw * config.stdyaw;
    noisy_odom.pose.covariance[(1+3)+(1+3)*6] = config.stdrp  * config.stdrp;
    noisy_odom.pose.covariance[(2+3)+(2+3)*6] = config.stdrp  * config.stdrp;
    noisy_odom.twist.covariance[0+0*6]        = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[1+1*6]        = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[2+2*6]        = config.stdvxyz * config.stdvxyz;
  }
  else
  {
    noisy_pose = drift_pose;
    noisy_vel  = drift_vel;
    noisy_odom.pose.covariance[0+0*6]         = 0;
    noisy_odom.pose.covariance[1+1*6]         = 0;
    noisy_odom.pose.covariance[2+2*6]         = 0;
    noisy_odom.pose.covariance[(0+3)+(0+3)*6] = 0;
    noisy_odom.pose.covariance[(1+3)+(1+3)*6] = 0;
    noisy_odom.pose.covariance[(2+3)+(2+3)*6] = 0;
    noisy_odom.twist.covariance[0+0*6]        = 0;
    noisy_odom.twist.covariance[1+1*6]        = 0;
    noisy_odom.twist.covariance[2+2*6]        = 0;
  }
  // Assemble and publish odom
  noisy_odom.pose.pose.position.x    = noisy_pose(0);
  noisy_odom.pose.pose.position.y    = noisy_pose(1);
  noisy_odom.pose.pose.position.z    = noisy_pose(2);
  noisy_odom.twist.twist.linear.x    = noisy_vel(0);
  noisy_odom.twist.twist.linear.y    = noisy_vel(1);
  noisy_odom.twist.twist.linear.z    = noisy_vel(2);
  colvec noisy_q                     = R_to_quaternion(ypr_to_R(noisy_pose.rows(3,5)));
  noisy_odom.pose.pose.orientation.w = noisy_q(0);
  noisy_odom.pose.pose.orientation.x = noisy_q(1);
  noisy_odom.pose.pose.orientation.y = noisy_q(2);
  noisy_odom.pose.pose.orientation.z = noisy_q(3);
  pubo->publish(noisy_odom);
  // Check time interval and publish correction
  static rclcpp::Time prev_correction_t(msg->header.stamp);
  if ((rclcpp::Time(msg->header.stamp) - prev_correction_t).seconds() > 1.0 / CORRECTION_RATE)
  {
    prev_correction_t             = msg->header.stamp;
    correction.pose.position.x    = correction_pose(0);
    correction.pose.position.y    = correction_pose(1);
    correction.pose.position.z    = correction_pose(2);
    colvec correction_q           = R_to_quaternion(ypr_to_R(correction_pose.rows(3,5)));
    correction.pose.orientation.w = correction_q(0);
    correction.pose.orientation.x = correction_q(1);
    correction.pose.orientation.y = correction_q(2);
    correction.pose.orientation.z = correction_q(3);
    pubc->publish(correction);
  }
}

void declare_params()
{
  config.fxy          = node->declare_parameter("fxy", config.fxy);
  config.stdfxy       = node->declare_parameter("stdfxy", config.stdfxy);
  config.fz           = node->declare_parameter("fz", config.fz);
  config.stdfz        = node->declare_parameter("stdfz", config.stdfz);
  config.mrp          = node->declare_parameter("mrp", config.mrp);
  config.stdmrp       = node->declare_parameter("stdmrp", config.stdmrp);
  config.myaw         = node->declare_parameter("myaw", config.myaw);
  config.stdmyaw      = node->declare_parameter("stdmyaw", config.stdmyaw);
  config.enable_noisy_odom = node->declare_parameter("enable_noisy_odom", config.enable_noisy_odom);
  config.stdxyz       = node->declare_parameter("stdxyz", config.stdxyz);
  config.stdvxyz      = node->declare_parameter("stdvxyz", config.stdvxyz);
  config.stdrp        = node->declare_parameter("stdrp", config.stdrp);
  config.stdyaw       = node->declare_parameter("stdyaw", config.stdyaw);
  config.enable_drift_odom = node->declare_parameter("enable_drift_odom", config.enable_drift_odom);
  config.stdvdriftxyz = node->declare_parameter("stdvdriftxyz", config.stdvdriftxyz);
  config.stdvdriftyaw = node->declare_parameter("stdvdriftyaw", config.stdvdriftyaw);
  config.vdriftx      = node->declare_parameter("vdriftx", config.vdriftx);
  config.vdrifty      = node->declare_parameter("vdrifty", config.vdrifty);
  config.vdriftz      = node->declare_parameter("vdriftz", config.vdriftz);
  config.vdriftyaw    = node->declare_parameter("vdriftyaw", config.vdriftyaw);
}

rcl_interfaces::msg::SetParametersResult
param_callback(const std::vector<rclcpp::Parameter>& params)
{
  for (const auto& p : params)
  {
    const std::string& n = p.get_name();
    if (n == "fxy")               config.fxy = p.as_double();
    else if (n == "stdfxy")       config.stdfxy = p.as_double();
    else if (n == "fz")           config.fz = p.as_double();
    else if (n == "stdfz")        config.stdfz = p.as_double();
    else if (n == "mrp")          config.mrp = p.as_double();
    else if (n == "stdmrp")       config.stdmrp = p.as_double();
    else if (n == "myaw")         config.myaw = p.as_double();
    else if (n == "stdmyaw")      config.stdmyaw = p.as_double();
    else if (n == "enable_noisy_odom") config.enable_noisy_odom = p.as_bool();
    else if (n == "stdxyz")       config.stdxyz = p.as_double();
    else if (n == "stdvxyz")      config.stdvxyz = p.as_double();
    else if (n == "stdrp")        config.stdrp = p.as_double();
    else if (n == "stdyaw")       config.stdyaw = p.as_double();
    else if (n == "enable_drift_odom") config.enable_drift_odom = p.as_bool();
    else if (n == "stdvdriftxyz") config.stdvdriftxyz = p.as_double();
    else if (n == "stdvdriftyaw") config.stdvdriftyaw = p.as_double();
    else if (n == "vdriftx")      config.vdriftx = p.as_double();
    else if (n == "vdrifty")      config.vdrifty = p.as_double();
    else if (n == "vdriftz")      config.vdriftz = p.as_double();
    else if (n == "vdriftyaw")    config.vdriftyaw = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void set_disturbance()
{
  geometry_msgs::msg::Vector3 f;
  geometry_msgs::msg::Vector3 m;
  f.x = config.fxy  + config.stdfxy  * as_scalar(randn(1));
  f.y = config.fxy  + config.stdfxy  * as_scalar(randn(1));
  f.z = config.fz   + config.stdfz   * as_scalar(randn(1));
  m.x = config.mrp  + config.stdmrp  * as_scalar(randn(1));
  m.y = config.mrp  + config.stdmrp  * as_scalar(randn(1));
  m.z = config.myaw + config.stdmyaw * as_scalar(randn(1));
  pubf->publish(f);
  pubm->publish(m);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("so3_disturbance_generator");

  declare_params();
  auto cb_handle = node->add_on_set_parameters_callback(param_callback);

  auto sub1 = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);
  pubo = node->create_publisher<nav_msgs::msg::Odometry>(        "noisy_odom",         10);
  pubc = node->create_publisher<geometry_msgs::msg::PoseStamped>("correction",         10);
  pubf = node->create_publisher<geometry_msgs::msg::Vector3>(    "force_disturbance",  10);
  pubm = node->create_publisher<geometry_msgs::msg::Vector3>(    "moment_disturbance", 10);

  rclcpp::Rate r(100.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    set_disturbance();
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

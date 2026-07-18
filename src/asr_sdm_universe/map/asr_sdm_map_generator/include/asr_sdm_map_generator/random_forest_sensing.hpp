#pragma once

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace asr_sdm_map_generator
{

class RandomForestSensing : public rclcpp::Node
{
public:
  explicit RandomForestSensing(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParameters();
  void generateRandomMap();
  void publishSensedPoints();
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void clickCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  bool isTooCloseToProtectedPoint(double x, double y) const;
  void addPillarObstacle(double x, double y, double width, pcl::PointCloud<pcl::PointXYZ> & cloud);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_sub_;
  rclcpp::TimerBase::SharedPtr sense_timer_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_local_map_;
  pcl::PointCloud<pcl::PointXYZ> cloud_map_;
  pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

  std::default_random_engine eng_;
  std::uniform_real_distribution<double> rand_x_;
  std::uniform_real_distribution<double> rand_y_;
  std::uniform_real_distribution<double> rand_w_;
  std::uniform_real_distribution<double> rand_h_;
  std::uniform_real_distribution<double> rand_radius_;
  std::uniform_real_distribution<double> rand_radius2_;
  std::uniform_real_distribution<double> rand_theta_;
  std::uniform_real_distribution<double> rand_z_;

  std::vector<double> state_;

  std::string local_cloud_topic_{"/asr_sdm_map_generator/local_cloud"};
  std::string global_cloud_topic_{"/asr_sdm_map_generator/global_cloud"};
  std::string click_map_topic_{"/pcl_render_node/local_map"};
  std::string odometry_topic_{"odometry"};
  std::string add_static_obstacle_{"/simulator/planning_simulator/add_static_obstacle"};

  int obs_num_{30};
  int circle_num_{30};
  double x_size_{50.0};
  double y_size_{50.0};
  double z_size_{5.0};
  double x_l_{0.0};
  double x_h_{0.0};
  double y_l_{0.0};
  double y_h_{0.0};
  double w_l_{0.3};
  double w_h_{0.8};
  double h_l_{3.0};
  double h_h_{7.0};
  double sensing_range_{10.0};
  double resolution_{0.1};
  double sense_rate_{10.0};
  double init_x_{0.0};
  double init_y_{0.0};
  double clear_radius_{2.0};
  double protect_x_{19.0};
  double protect_y_{0.0};
  double radius_l_{7.0};
  double radius_h_{7.0};
  double z_l_{7.0};
  double z_h_{7.0};
  double theta_{7.0};
  bool publish_local_{false};
  bool enable_click_map_{false};
  bool map_ok_{false};
  bool has_odom_{false};
};

}  // namespace asr_sdm_map_generator

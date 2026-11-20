#include "asr_sdm_map_generator/random_forest_sensing.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace asr_sdm_map_generator
{

RandomForestSensing::RandomForestSensing(const rclcpp::NodeOptions & options)
: Node("random_map_sensing", options), eng_(std::random_device{}())
{
  declareParameters();
  loadParameters();

  if (seed_ >= 0) {
    eng_.seed(static_cast<std::default_random_engine::result_type>(seed_));
  }

  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_cloud_topic_, 1);
  all_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(global_cloud_topic_, 1);
  click_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(click_map_topic_, 1);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 50,
    std::bind(&RandomForestSensing::odometryCallback, this, std::placeholders::_1));

  if (enable_click_map_) {
    click_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      add_static_obstacle_, 10,
      std::bind(&RandomForestSensing::clickCallback, this, std::placeholders::_1));
  }

  x_l_ = -x_size_ / 2.0;
  x_h_ = x_size_ / 2.0;
  y_l_ = -y_size_ / 2.0;
  y_h_ = y_size_ / 2.0;
  obs_num_ = std::min(obs_num_, static_cast<int>(x_size_ * 10));

  generateMap();

  const auto period = std::chrono::duration<double>(1.0 / std::max(sense_rate_, 1e-3));
  sense_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&RandomForestSensing::publishSensedPoints, this));

  RCLCPP_INFO(
    get_logger(), "random_map_sensing ready (type=%s, publish_local=%s)", map_type_.c_str(),
    publish_local_ ? "true" : "false");
}

void RandomForestSensing::declareParameters()
{
  declare_parameter("init_state_x", 0.0);
  declare_parameter("init_state_y", 0.0);

  declare_parameter("map.type", map_type_);
  declare_parameter("map.seed", seed_);
  declare_parameter("map.x_size", 50.0);
  declare_parameter("map.y_size", 50.0);
  declare_parameter("map.z_size", 5.0);
  declare_parameter("map.obs_num", 30);
  declare_parameter("map.resolution", 0.1);
  declare_parameter("map.circle_num", 30);
  declare_parameter("map.clear_radius", 2.0);
  declare_parameter("map.protect_x", 19.0);
  declare_parameter("map.protect_y", 0.0);

  declare_parameter("ObstacleShape.lower_rad", 0.3);
  declare_parameter("ObstacleShape.upper_rad", 0.8);
  declare_parameter("ObstacleShape.lower_hei", 3.0);
  declare_parameter("ObstacleShape.upper_hei", 7.0);
  declare_parameter("ObstacleShape.radius_l", 7.0);
  declare_parameter("ObstacleShape.radius_h", 7.0);
  declare_parameter("ObstacleShape.z_l", 7.0);
  declare_parameter("ObstacleShape.z_h", 7.0);
  declare_parameter("ObstacleShape.theta", 7.0);
  declare_parameter("ObstacleShape.thickness", 0.4);

  declare_parameter("perlin3d.complexity", perlin3d_.complexity);
  declare_parameter("perlin3d.fill", perlin3d_.fill);
  declare_parameter("perlin3d.fractal", perlin3d_.fractal);
  declare_parameter("perlin3d.attenuation", perlin3d_.attenuation);

  declare_parameter("post2d.width_min", post2d_.width_min);
  declare_parameter("post2d.width_max", post2d_.width_max);
  declare_parameter("post2d.obstacle_number", post2d_.obstacle_number);

  declare_parameter("maze2d.road_width", maze2d_.road_width);
  declare_parameter("maze2d.add_wall_x", maze2d_.add_wall_x);
  declare_parameter("maze2d.add_wall_y", maze2d_.add_wall_y);

  declare_parameter("maze3d.num_nodes", maze3d_.num_nodes);
  declare_parameter("maze3d.connectivity", maze3d_.connectivity);
  declare_parameter("maze3d.road_radius", maze3d_.road_radius);

  declare_parameter("sensing.radius", 10.0);
  declare_parameter("sensing.rate", 10.0);
  declare_parameter("sensing.publish_local", false);
  declare_parameter("sensing.enable_click_map", false);

  declare_parameter("topics.local_cloud", local_cloud_topic_);
  declare_parameter("topics.global_cloud", global_cloud_topic_);
  declare_parameter("topics.click_map", click_map_topic_);
  declare_parameter("topics.odometry", odometry_topic_);
  declare_parameter("topics.add_static_obstacle", add_static_obstacle_);
}

void RandomForestSensing::loadParameters()
{
  init_x_ = get_parameter("init_state_x").as_double();
  init_y_ = get_parameter("init_state_y").as_double();

  map_type_ = get_parameter("map.type").as_string();
  seed_ = static_cast<int>(get_parameter("map.seed").as_int());
  x_size_ = get_parameter("map.x_size").as_double();
  y_size_ = get_parameter("map.y_size").as_double();
  z_size_ = get_parameter("map.z_size").as_double();
  obs_num_ = get_parameter("map.obs_num").as_int();
  resolution_ = get_parameter("map.resolution").as_double();
  circle_num_ = get_parameter("map.circle_num").as_int();
  clear_radius_ = get_parameter("map.clear_radius").as_double();
  protect_x_ = get_parameter("map.protect_x").as_double();
  protect_y_ = get_parameter("map.protect_y").as_double();

  w_l_ = get_parameter("ObstacleShape.lower_rad").as_double();
  w_h_ = get_parameter("ObstacleShape.upper_rad").as_double();
  h_l_ = get_parameter("ObstacleShape.lower_hei").as_double();
  h_h_ = get_parameter("ObstacleShape.upper_hei").as_double();
  radius_l_ = get_parameter("ObstacleShape.radius_l").as_double();
  radius_h_ = get_parameter("ObstacleShape.radius_h").as_double();
  z_l_ = get_parameter("ObstacleShape.z_l").as_double();
  z_h_ = get_parameter("ObstacleShape.z_h").as_double();
  theta_ = get_parameter("ObstacleShape.theta").as_double();
  circle_thickness_ = get_parameter("ObstacleShape.thickness").as_double();

  perlin3d_.complexity = get_parameter("perlin3d.complexity").as_double();
  perlin3d_.fill = get_parameter("perlin3d.fill").as_double();
  perlin3d_.fractal = static_cast<int>(get_parameter("perlin3d.fractal").as_int());
  perlin3d_.attenuation = get_parameter("perlin3d.attenuation").as_double();

  post2d_.width_min = get_parameter("post2d.width_min").as_double();
  post2d_.width_max = get_parameter("post2d.width_max").as_double();
  post2d_.obstacle_number = static_cast<int>(get_parameter("post2d.obstacle_number").as_int());

  maze2d_.road_width = get_parameter("maze2d.road_width").as_double();
  maze2d_.add_wall_x = get_parameter("maze2d.add_wall_x").as_bool();
  maze2d_.add_wall_y = get_parameter("maze2d.add_wall_y").as_bool();

  maze3d_.num_nodes = static_cast<int>(get_parameter("maze3d.num_nodes").as_int());
  maze3d_.connectivity = get_parameter("maze3d.connectivity").as_double();
  maze3d_.road_radius = static_cast<int>(get_parameter("maze3d.road_radius").as_int());

  sensing_range_ = get_parameter("sensing.radius").as_double();
  sense_rate_ = get_parameter("sensing.rate").as_double();
  publish_local_ = get_parameter("sensing.publish_local").as_bool();
  enable_click_map_ = get_parameter("sensing.enable_click_map").as_bool();

  local_cloud_topic_ = get_parameter("topics.local_cloud").as_string();
  global_cloud_topic_ = get_parameter("topics.global_cloud").as_string();
  click_map_topic_ = get_parameter("topics.click_map").as_string();
  odometry_topic_ = get_parameter("topics.odometry").as_string();
  add_static_obstacle_ = get_parameter("topics.add_static_obstacle").as_string();
}

bool RandomForestSensing::isTooCloseToProtectedPoint(double x, double y) const
{
  const double dx_init = x - init_x_;
  const double dy_init = y - init_y_;
  if (std::sqrt(dx_init * dx_init + dy_init * dy_init) < clear_radius_) {
    return true;
  }

  const double dx_protect = x - protect_x_;
  const double dy_protect = y - protect_y_;
  return std::sqrt(dx_protect * dx_protect + dy_protect * dy_protect) < clear_radius_;
}

void RandomForestSensing::clearProtectedArea(pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  auto & points = cloud.points;
  points.erase(
    std::remove_if(
      points.begin(), points.end(),
      [this](const pcl::PointXYZ & pt) { return isTooCloseToProtectedPoint(pt.x, pt.y); }),
    points.end());
}

void RandomForestSensing::addPillarObstacle(
  double x, double y, double width, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;

  const int wid_num = static_cast<int>(std::ceil(width / resolution_));
  pcl::PointXYZ pt;

  for (int r = -wid_num / 2; r < wid_num / 2; ++r) {
    for (int s = -wid_num / 2; s < wid_num / 2; ++s) {
      const double h = rand_h_(eng_);
      const int hei_num = static_cast<int>(std::ceil(h / resolution_));
      for (int t = -30; t < hei_num; ++t) {
        pt.x = static_cast<float>(x + (r + 0.5) * resolution_ + 1e-2);
        pt.y = static_cast<float>(y + (s + 0.5) * resolution_ + 1e-2);
        pt.z = static_cast<float>((t + 0.5) * resolution_ + 1e-2);
        cloud.points.push_back(pt);
      }
    }
  }
}

void RandomForestSensing::addEllipseObstacle(
  double x, double y, double z, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;
  z = std::floor(z / resolution_) * resolution_ + resolution_ / 2.0;

  const double theta = rand_theta_(eng_);
  Eigen::Matrix3d rotate;
  rotate << std::cos(theta), -std::sin(theta), 0.0, std::sin(theta), std::cos(theta), 0.0, 0.0,
    0.0, 1.0;

  const double radius1 = rand_radius_(eng_);
  const double radius2 = rand_radius2_(eng_);
  const Eigen::Vector3d center(x, y, z);

  // Tube diameter. Zero keeps the original single-point centerline.
  const double half = std::max(circle_thickness_, 0.0) * 0.5;
  const int radial_steps =
    resolution_ > 0.0 ? static_cast<int>(std::ceil(half / resolution_)) : 0;
  const double half_sq = half * half;

  const double max_radius = std::max(std::abs(radius1), std::abs(radius2));
  double angle_step = resolution_ / 2.0;
  if (half > 0.0 && max_radius > 1e-6 && resolution_ > 0.0) {
    angle_step = resolution_ / max_radius;
  }
  if (angle_step <= 0.0) {
    angle_step = M_PI / 16.0;
  }

  pcl::PointXYZ pt;
  for (double angle = 0.0; angle < 2.0 * M_PI; angle += angle_step) {
    const Eigen::Vector3d cpt(0.0, radius1 * std::cos(angle), radius2 * std::sin(angle));
    for (int ix = -radial_steps; ix <= radial_steps; ++ix) {
      for (int iy = -radial_steps; iy <= radial_steps; ++iy) {
        for (int iz = -radial_steps; iz <= radial_steps; ++iz) {
          const Eigen::Vector3d offset(ix * resolution_, iy * resolution_, iz * resolution_);
          if (offset.squaredNorm() > half_sq + 1e-12) {
            continue;
          }
          const Eigen::Vector3d world = rotate * (cpt + offset) + center;
          pt.x = static_cast<float>(world(0));
          pt.y = static_cast<float>(world(1));
          pt.z = static_cast<float>(world(2));
          cloud.points.push_back(pt);
        }
      }
    }
  }
}

void RandomForestSensing::generateMap()
{
  cloud_map_.clear();

  // rand_w_/rand_h_ also size the click-added pillars, so set them for every map type.
  rand_w_ = std::uniform_real_distribution<double>(w_l_, w_h_);
  rand_h_ = std::uniform_real_distribution<double>(h_l_, h_h_);

  const MapGrid grid{x_size_, y_size_, z_size_, resolution_};
  if (map_type_ == "random_forest") {
    generateRandomForest();
  } else if (map_type_ == "perlin3d") {
    generatePerlin3d(grid, perlin3d_, eng_, cloud_map_);
  } else if (map_type_ == "post2d") {
    generatePost2d(grid, post2d_, eng_, cloud_map_);
  } else if (map_type_ == "maze2d") {
    generateMaze2d(grid, maze2d_, eng_, cloud_map_);
  } else if (map_type_ == "maze3d") {
    generateMaze3d(grid, maze3d_, eng_, cloud_map_);
  } else {
    throw std::invalid_argument(
      "Unknown map.type '" + map_type_ +
      "' (expected random_forest, perlin3d, post2d, maze2d or maze3d)");
  }
  if (map_type_ != "random_forest") {
    // Random forest rejects obstacles near the protected points while placing them.
    clearProtectedArea(cloud_map_);
  }

  cloud_map_.width = cloud_map_.points.size();
  cloud_map_.height = 1;
  cloud_map_.is_dense = true;

  if (!cloud_map_.empty()) {
    kdtree_local_map_.setInputCloud(cloud_map_.makeShared());
    map_ok_ = true;
  }

  RCLCPP_INFO(
    get_logger(), "Generated %s map with %zu points", map_type_.c_str(), cloud_map_.points.size());
}

void RandomForestSensing::generateRandomForest()
{
  rand_x_ = std::uniform_real_distribution<double>(x_l_, x_h_);
  rand_y_ = std::uniform_real_distribution<double>(y_l_, y_h_);
  rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);
  rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);

  constexpr int kMaxAttemptsFactor = 100;

  int placed = 0;
  int attempts = 0;
  const int max_attempts = std::max(obs_num_ * kMaxAttemptsFactor, 1);
  while (placed < obs_num_ && attempts < max_attempts) {
    ++attempts;
    const double x = rand_x_(eng_);
    const double y = rand_y_(eng_);
    if (isTooCloseToProtectedPoint(x, y)) {
      continue;
    }
    addPillarObstacle(x, y, rand_w_(eng_), cloud_map_);
    ++placed;
  }
  if (placed < obs_num_) {
    RCLCPP_WARN(
      get_logger(), "Only placed %d/%d pillar obstacles (clearance constraints)", placed, obs_num_);
  }

  placed = 0;
  attempts = 0;
  const int max_circle_attempts = std::max(circle_num_ * kMaxAttemptsFactor, 1);
  while (placed < circle_num_ && attempts < max_circle_attempts) {
    ++attempts;
    const double x = rand_x_(eng_);
    const double y = rand_y_(eng_);
    const double z = rand_z_(eng_);
    if (isTooCloseToProtectedPoint(x, y)) {
      continue;
    }
    addEllipseObstacle(x, y, z, cloud_map_);
    ++placed;
  }
  if (placed < circle_num_) {
    RCLCPP_WARN(
      get_logger(), "Only placed %d/%d circle obstacles (clearance constraints)", placed,
      circle_num_);
  }
}

void RandomForestSensing::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (odom->child_frame_id == "X" || odom->child_frame_id == "O") {
    return;
  }

  has_odom_ = true;
  state_ = {
    odom->pose.pose.position.x,
    odom->pose.pose.position.y,
    odom->pose.pose.position.z,
    odom->twist.twist.linear.x,
    odom->twist.twist.linear.y,
    odom->twist.twist.linear.z,
    0.0,
    0.0,
    0.0};
}

void RandomForestSensing::publishSensedPoints()
{
  sensor_msgs::msg::PointCloud2 global_map_msg;
  pcl::toROSMsg(cloud_map_, global_map_msg);
  global_map_msg.header.frame_id = "world";
  global_map_msg.header.stamp = now();
  all_map_pub_->publish(global_map_msg);

  if (!publish_local_ || !map_ok_ || !has_odom_ || state_.size() < 3) {
    return;
  }

  pcl::PointXYZ search_point(
    static_cast<float>(state_[0]), static_cast<float>(state_[1]), static_cast<float>(state_[2]));
  if (std::isnan(search_point.x) || std::isnan(search_point.y) || std::isnan(search_point.z)) {
    return;
  }

  std::vector<int> point_idx_radius_search;
  std::vector<float> point_radius_squared_distance;
  if (
    kdtree_local_map_.radiusSearch(
      search_point, sensing_range_, point_idx_radius_search, point_radius_squared_distance) <= 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "[Map server] No obstacles in range.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> local_map;
  local_map.points.reserve(point_idx_radius_search.size());
  for (const int idx : point_idx_radius_search) {
    local_map.points.push_back(cloud_map_.points[idx]);
  }
  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  sensor_msgs::msg::PointCloud2 local_map_msg;
  pcl::toROSMsg(local_map, local_map_msg);
  local_map_msg.header.frame_id = "world";
  local_map_msg.header.stamp = now();
  local_map_pub_->publish(local_map_msg);
}

void RandomForestSensing::clickCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  const double width = rand_w_(eng_);

  x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;

  const int wid_num = static_cast<int>(std::ceil(width / resolution_));
  pcl::PointXYZ pt;

  for (int r = -wid_num / 2; r < wid_num / 2; ++r) {
    for (int s = -wid_num / 2; s < wid_num / 2; ++s) {
      const double h = rand_h_(eng_);
      const int hei_num = static_cast<int>(std::ceil(h / resolution_));
      for (int t = -1; t < hei_num; ++t) {
        pt.x = static_cast<float>(x + (r + 0.5) * resolution_ + 1e-2);
        pt.y = static_cast<float>(y + (s + 0.5) * resolution_ + 1e-2);
        pt.z = static_cast<float>((t + 0.5) * resolution_ + 1e-2);
        clicked_cloud_.points.push_back(pt);
        cloud_map_.points.push_back(pt);
      }
    }
  }

  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;
  cloud_map_.width = cloud_map_.points.size();

  sensor_msgs::msg::PointCloud2 click_map_msg;
  pcl::toROSMsg(clicked_cloud_, click_map_msg);
  click_map_msg.header.frame_id = "world";
  click_map_msg.header.stamp = now();
  click_map_pub_->publish(click_map_msg);

  // Refresh global map immediately so RViz Global Map updates without waiting
  // for the sensing timer.
  sensor_msgs::msg::PointCloud2 global_map_msg;
  pcl::toROSMsg(cloud_map_, global_map_msg);
  global_map_msg.header.frame_id = "world";
  global_map_msg.header.stamp = now();
  all_map_pub_->publish(global_map_msg);

  kdtree_local_map_.setInputCloud(cloud_map_.makeShared());
  map_ok_ = true;
  RCLCPP_INFO(
    get_logger(), "Added click obstacle at (%.2f, %.2f), map points=%zu", x, y,
    cloud_map_.points.size());
}

}  // namespace asr_sdm_map_generator

/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology,
 * <uav.ust.hk> Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at
 * gmail dot com> for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include "asr_sdm_esdf_map/esdf_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace {

std_msgs::msg::ColorRGBA heightColor(double value, double alpha)
{
  value = std::max(0.0, std::min(1.0, value));

  std_msgs::msg::ColorRGBA color;
  color.a = alpha;
  color.r = std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 3.0)));
  color.g = std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 2.0)));
  color.b = std::max(0.0, std::min(1.0, 1.5 - std::abs(4.0 * value - 1.0)));
  return color;
}

struct SurfaceVertex
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  bool valid = false;
};



}  // namespace

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;

  /* get parameter — ROS 2 dotted names correspond to esdf_map/ros1 slash keys */
  double x_size, y_size, z_size;
  node_->declare_parameter("esdf_map.resolution", -1.0);
  mp_.resolution_ = node_->get_parameter("esdf_map.resolution").as_double();
  node_->declare_parameter("esdf_map.map_size_x", -1.0);
  node_->declare_parameter("esdf_map.map_size_y", -1.0);
  node_->declare_parameter("esdf_map.map_size_z", -1.0);
  x_size = node_->get_parameter("esdf_map.map_size_x").as_double();
  y_size = node_->get_parameter("esdf_map.map_size_y").as_double();
  z_size = node_->get_parameter("esdf_map.map_size_z").as_double();

  node_->declare_parameter("esdf_map.local_update_range_x", -1.0);
  node_->declare_parameter("esdf_map.local_update_range_y", -1.0);
  node_->declare_parameter("esdf_map.local_update_range_z", -1.0);
  mp_.local_update_range_(0) = node_->get_parameter("esdf_map.local_update_range_x").as_double();
  mp_.local_update_range_(1) = node_->get_parameter("esdf_map.local_update_range_y").as_double();
  mp_.local_update_range_(2) = node_->get_parameter("esdf_map.local_update_range_z").as_double();

  node_->declare_parameter("esdf_map.obstacles_inflation", -1.0);
  mp_.obstacles_inflation_ = node_->get_parameter("esdf_map.obstacles_inflation").as_double();

  node_->declare_parameter("esdf_map.fx", -1.0);
  node_->declare_parameter("esdf_map.fy", -1.0);
  node_->declare_parameter("esdf_map.cx", -1.0);
  node_->declare_parameter("esdf_map.cy", -1.0);
  mp_.fx_ = node_->get_parameter("esdf_map.fx").as_double();
  mp_.fy_ = node_->get_parameter("esdf_map.fy").as_double();
  mp_.cx_ = node_->get_parameter("esdf_map.cx").as_double();
  mp_.cy_ = node_->get_parameter("esdf_map.cy").as_double();

  node_->declare_parameter("esdf_map.use_depth_filter", true);
  mp_.use_depth_filter_ = node_->get_parameter("esdf_map.use_depth_filter").as_bool();

  node_->declare_parameter("esdf_map.depth_filter_tolerance", -1.0);
  mp_.depth_filter_tolerance_ = node_->get_parameter("esdf_map.depth_filter_tolerance").as_double();
  node_->declare_parameter("esdf_map.depth_filter_maxdist", -1.0);
  mp_.depth_filter_maxdist_ = node_->get_parameter("esdf_map.depth_filter_maxdist").as_double();
  node_->declare_parameter("esdf_map.depth_filter_mindist", -1.0);
  mp_.depth_filter_mindist_ = node_->get_parameter("esdf_map.depth_filter_mindist").as_double();
  node_->declare_parameter("esdf_map.depth_filter_margin", -1);
  mp_.depth_filter_margin_ = node_->get_parameter("esdf_map.depth_filter_margin").as_int();
  node_->declare_parameter("esdf_map.k_depth_scaling_factor", -1.0);
  mp_.k_depth_scaling_factor_ = node_->get_parameter("esdf_map.k_depth_scaling_factor").as_double();
  node_->declare_parameter("esdf_map.skip_pixel", -1);
  mp_.skip_pixel_ = node_->get_parameter("esdf_map.skip_pixel").as_int();

  node_->declare_parameter("esdf_map.p_hit", 0.70);
  node_->declare_parameter("esdf_map.p_miss", 0.35);
  node_->declare_parameter("esdf_map.p_min", 0.12);
  node_->declare_parameter("esdf_map.p_max", 0.97);
  node_->declare_parameter("esdf_map.p_occ", 0.80);
  mp_.p_hit_ = node_->get_parameter("esdf_map.p_hit").as_double();
  mp_.p_miss_ = node_->get_parameter("esdf_map.p_miss").as_double();
  mp_.p_min_ = node_->get_parameter("esdf_map.p_min").as_double();
  mp_.p_max_ = node_->get_parameter("esdf_map.p_max").as_double();
  mp_.p_occ_ = node_->get_parameter("esdf_map.p_occ").as_double();

  node_->declare_parameter("esdf_map.min_ray_length", -0.1);
  node_->declare_parameter("esdf_map.max_ray_length", -0.1);
  mp_.min_ray_length_ = node_->get_parameter("esdf_map.min_ray_length").as_double();
  mp_.max_ray_length_ = node_->get_parameter("esdf_map.max_ray_length").as_double();

  node_->declare_parameter("esdf_map.esdf_slice_height", -0.1);
  node_->declare_parameter("esdf_map.visualization_truncate_height", -0.1);
  node_->declare_parameter("esdf_map.virtual_ceil_height", -0.1);
  mp_.esdf_slice_height_ = node_->get_parameter("esdf_map.esdf_slice_height").as_double();
  mp_.visualization_truncate_height_ =
    node_->get_parameter("esdf_map.visualization_truncate_height").as_double();
  mp_.virtual_ceil_height_ = node_->get_parameter("esdf_map.virtual_ceil_height").as_double();

  node_->declare_parameter("esdf_map.publish_esdf_3d", true);
  node_->declare_parameter("esdf_map.esdf_3d_local_only", true);
  node_->declare_parameter("esdf_map.esdf_3d_stride", 1);
  node_->declare_parameter("esdf_map.esdf_3d_min_distance", 0.0);
  node_->declare_parameter("esdf_map.esdf_3d_max_distance", 3.0);
  node_->declare_parameter("esdf_map.esdf_3d_publish_negative_distance", true);
  node_->declare_parameter("esdf_map.publish_esdf_distance", true);
  mp_.publish_esdf_3d_ = node_->get_parameter("esdf_map.publish_esdf_3d").as_bool();
  mp_.esdf_3d_local_only_ = node_->get_parameter("esdf_map.esdf_3d_local_only").as_bool();
  mp_.esdf_3d_stride_ = node_->get_parameter("esdf_map.esdf_3d_stride").as_int();
  mp_.esdf_3d_min_distance_ = node_->get_parameter("esdf_map.esdf_3d_min_distance").as_double();
  mp_.esdf_3d_max_distance_ = node_->get_parameter("esdf_map.esdf_3d_max_distance").as_double();
  mp_.esdf_3d_publish_negative_distance_ =
    node_->get_parameter("esdf_map.esdf_3d_publish_negative_distance").as_bool();
  mp_.publish_esdf_distance_ = node_->get_parameter("esdf_map.publish_esdf_distance").as_bool();
  mp_.esdf_3d_stride_ = std::max(1, mp_.esdf_3d_stride_);
  mp_.esdf_3d_max_distance_ = std::max(mp_.esdf_3d_min_distance_ + 1e-3, mp_.esdf_3d_max_distance_);

  node_->declare_parameter("esdf_map.publish_occupied_map", true);
  node_->declare_parameter("esdf_map.occupied_map_local_only", false);
  node_->declare_parameter("esdf_map.occupied_map_use_inflated", true);
  node_->declare_parameter("esdf_map.occupied_map_stride", 1);
  node_->declare_parameter("esdf_map.occupied_map_alpha", 0.85);
  node_->declare_parameter("esdf_map.occupied_map_mesh_resolution", 0.30);
  node_->declare_parameter("esdf_map.occupied_map_mesh_max_height_gap", 0.60);
  mp_.publish_occupied_map_ =
    node_->get_parameter("esdf_map.publish_occupied_map").as_bool();
  mp_.occupied_map_local_only_ =
    node_->get_parameter("esdf_map.occupied_map_local_only").as_bool();
  mp_.occupied_map_use_inflated_ =
    node_->get_parameter("esdf_map.occupied_map_use_inflated").as_bool();
  mp_.occupied_map_stride_ = node_->get_parameter("esdf_map.occupied_map_stride").as_int();
  mp_.occupied_map_alpha_ = node_->get_parameter("esdf_map.occupied_map_alpha").as_double();
  mp_.occupied_map_mesh_resolution_ =
    node_->get_parameter("esdf_map.occupied_map_mesh_resolution").as_double();
  mp_.occupied_map_mesh_max_height_gap_ =
    node_->get_parameter("esdf_map.occupied_map_mesh_max_height_gap").as_double();
  mp_.occupied_map_stride_ = std::max(1, mp_.occupied_map_stride_);
  mp_.occupied_map_alpha_ = std::max(0.0, std::min(1.0, mp_.occupied_map_alpha_));
  mp_.occupied_map_mesh_resolution_ =
    std::max(mp_.resolution_, mp_.occupied_map_mesh_resolution_);
  mp_.occupied_map_mesh_max_height_gap_ =
    std::max(mp_.resolution_, mp_.occupied_map_mesh_max_height_gap_);

  node_->declare_parameter("esdf_map.show_occ_time", false);
  node_->declare_parameter("esdf_map.show_esdf_time", false);
  node_->declare_parameter("esdf_map.pose_type", 1);
  mp_.show_occ_time_ = node_->get_parameter("esdf_map.show_occ_time").as_bool();
  mp_.show_esdf_time_ = node_->get_parameter("esdf_map.show_esdf_time").as_bool();
  mp_.pose_type_ = node_->get_parameter("esdf_map.pose_type").as_int();

  node_->declare_parameter("esdf_map.frame_id", std::string("world"));
  mp_.frame_id_ = node_->get_parameter("esdf_map.frame_id").as_string();

  node_->declare_parameter("esdf_map.input_mode", std::string("vio"));
  node_->declare_parameter("esdf_map.depth_topic", std::string("/sensing/camera/realsense/depth"));
  node_->declare_parameter("esdf_map.pose_topic", std::string("/localization/video_inertial_odom/pose"));
  node_->declare_parameter("esdf_map.odom_topic", std::string("/esdf_map/odom"));
  node_->declare_parameter("esdf_map.cloud_topic", std::string("/esdf_map/cloud_input"));
  node_->declare_parameter(
    "esdf_map.vio_pose_topic", std::string("/localization/video_inertial_odom/pose"));
  node_->declare_parameter(
    "esdf_map.vio_points_topic", std::string("/localization/video_inertial_odom/points"));
  node_->declare_parameter("esdf_map.vio_points_ns_filter", std::string("pts"));
  node_->declare_parameter("esdf_map.vio_points_accumulate", true);

  mp_.input_mode_ = node_->get_parameter("esdf_map.input_mode").as_string();
  mp_.depth_topic_ = node_->get_parameter("esdf_map.depth_topic").as_string();
  mp_.pose_topic_ = node_->get_parameter("esdf_map.pose_topic").as_string();
  mp_.odom_topic_ = node_->get_parameter("esdf_map.odom_topic").as_string();
  mp_.cloud_topic_ = node_->get_parameter("esdf_map.cloud_topic").as_string();
  mp_.vio_pose_topic_ = node_->get_parameter("esdf_map.vio_pose_topic").as_string();
  mp_.vio_points_topic_ = node_->get_parameter("esdf_map.vio_points_topic").as_string();
  mp_.vio_points_ns_filter_ = node_->get_parameter("esdf_map.vio_points_ns_filter").as_string();
  mp_.vio_points_accumulate_ = node_->get_parameter("esdf_map.vio_points_accumulate").as_bool();

  if (mp_.input_mode_ != "vio" && mp_.input_mode_ != "depth") {
    RCLCPP_WARN(
      node_->get_logger(),
      "Unsupported esdf_map.input_mode='%s'. Only 'vio' and 'depth' are valid. Falling back to 'vio'.",
      mp_.input_mode_.c_str());
    mp_.input_mode_ = "vio";
  }


  node_->declare_parameter("esdf_map.local_bound_inflate", 1.0);
  node_->declare_parameter("esdf_map.local_map_margin", 1);
  node_->declare_parameter("esdf_map.ground_height", 1.0);
  mp_.local_bound_inflate_ = node_->get_parameter("esdf_map.local_bound_inflate").as_double();
  mp_.local_map_margin_ = node_->get_parameter("esdf_map.local_map_margin").as_int();
  mp_.ground_height_ = node_->get_parameter("esdf_map.ground_height").as_double();

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  RCLCPP_INFO(node_->get_logger(), "hit: %f", mp_.prob_hit_log_);
  RCLCPP_INFO(node_->get_logger(), "miss: %f", mp_.prob_miss_log_);
  RCLCPP_INFO(node_->get_logger(), "min log: %f", mp_.clamp_min_log_);
  RCLCPP_INFO(node_->get_logger(), "max: %f", mp_.clamp_max_log_);
  RCLCPP_INFO(node_->get_logger(), "thresh log: %f", mp_.min_occupancy_log_);

  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback: vio and depth are mutually exclusive input modes. */

  if (mp_.input_mode_ == "depth") {
    depth_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      mp_.depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SDFMap::depthCallback, this, std::placeholders::_1));
    pose_cov_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      mp_.pose_topic_, rclcpp::QoS(50),
      std::bind(&SDFMap::vioPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "ESDF depth input enabled: depth=%s, pose=%s. No odometry topic is required.",
      mp_.depth_topic_.c_str(), mp_.pose_topic_.c_str());
  } else {
    pose_cov_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      mp_.vio_pose_topic_, rclcpp::QoS(50),
      std::bind(&SDFMap::vioPoseCallback, this, std::placeholders::_1));
    vio_points_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
      mp_.vio_points_topic_, rclcpp::QoS(1000),
      std::bind(&SDFMap::vioPointsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "ESDF VIO sparse input enabled: pose=%s, points=%s",
      mp_.vio_pose_topic_.c_str(), mp_.vio_points_topic_.c_str());
  }

  auto period = std::chrono::milliseconds(50);
  occ_timer_ = node_->create_wall_timer(period, std::bind(&SDFMap::updateOccupancyCallback, this));
  esdf_timer_ = node_->create_wall_timer(period, std::bind(&SDFMap::updateESDFCallback, this));
  vis_timer_ = node_->create_wall_timer(period, std::bind(&SDFMap::visCallback, this));
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/cloud", 10);
  map_inf_pub_ =
    node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/occupancy_inflate", 10);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/esdf", 10);
  esdf_3d_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/esdf_3d", 10);
  esdf_distance_pub_ =
    node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/esdf_distance", 10);
  occupied_map_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/esdf_map/occupied_map", 10);
  update_range_pub_ =
    node_->create_publisher<visualization_msgs::msg::Marker>("/esdf_map/update_range", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/unknown", 10);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/esdf_map/depth_cloud", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());
}

void SDFMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d()
{
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
        [&](int z) {
          return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1
                   ? 0
                   : std::numeric_limits<double>::max();
        },
        [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
        max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
        [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
        [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
        max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
        [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
        [&](int x, double val) {
          md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
          //  min(mp_.resolution_ * std::sqrt(val),
          //      md_.distance_buffer_[toAddress(x, y, z)]);
        },
        min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "what?");
        }
      }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
        [&](int z) {
          return md_.occupancy_buffer_neg
                       [x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                        y * mp_.map_voxel_num_(2) + z] == 1
                   ? 0
                   : std::numeric_limits<double>::max();
        },
        [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
        max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
        [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
        [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
        max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
        [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
        [&](int x, double val) {
          md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
        },
        min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id)) return INVALID_IDX;
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage()
{
  md_.proj_points_cnt = 0;

  const int cols = md_.depth_image_.cols;
  const int rows = md_.depth_image_.rows;
  if (cols <= 0 || rows <= 0) return;

  const int step = std::max(1, mp_.skip_pixel_);
  const size_t needed = mp_.use_depth_filter_
                          ? static_cast<size_t>((rows + step - 1) / step + 1) *
                              static_cast<size_t>((cols + step - 1) / step + 1)
                          : static_cast<size_t>(rows) * static_cast<size_t>(cols);
  if (md_.proj_points_.size() < needed) {
    md_.proj_points_.resize(needed);
  }

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  auto push_projected_point = [&](int u, int v, double depth) {
    Eigen::Vector3d proj_pt;
    proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
    proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
    proj_pt(2) = depth;
    proj_pt = camera_r * proj_pt + md_.camera_pos_;

    if (static_cast<size_t>(md_.proj_points_cnt) >= md_.proj_points_.size()) {
      md_.proj_points_.push_back(proj_pt);
    } else {
      md_.proj_points_[md_.proj_points_cnt] = proj_pt;
    }
    ++md_.proj_points_cnt;
  };

  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; ++v) {
      const uint16_t * row_ptr = md_.depth_image_.ptr<uint16_t>(v);
      for (int u = 0; u < cols; ++u) {
        const uint16_t raw_depth = row_ptr[u];
        if (raw_depth == 0) continue;

        double depth = raw_depth * inv_factor;
        if (depth < mp_.depth_filter_mindist_) continue;
        if (depth > mp_.depth_filter_maxdist_) depth = mp_.max_ray_length_ + 0.1;

        push_projected_point(u, v, depth);
      }
    }
  } else {
    if (!md_.has_first_depth_) {
      md_.has_first_depth_ = true;
    } else {
      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += step) {
        const uint16_t * row_ptr = md_.depth_image_.ptr<uint16_t>(v);
        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += step) {
          const uint16_t raw_depth = row_ptr[u];
          double depth = raw_depth * inv_factor;

          if (raw_depth == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          push_projected_point(u, v, depth);
        }
      }
    }
  }

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void SDFMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty()) {
    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
      md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns]
        ? mp_.prob_hit_log_
        : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] = std::min(
      std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
      mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(
  const Eigen::Vector3d & pt, const Eigen::Vector3d & camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut =
    md_.local_bound_min_ -
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut =
    md_.local_bound_max_ +
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {
      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {
      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {
      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (
              idx_inf < 0 ||
              idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void SDFMap::visCallback()
{
  if (!md_.has_odom_ || !md_.has_cloud_) return;

  publishMap();
  publishMapInflate(false);
  if (mp_.publish_occupied_map_) publishOccupiedMap();
  publishESDF();
  if (mp_.publish_esdf_3d_) publishESDF3D();
  if (mp_.publish_esdf_distance_) publishESDFDistance();
  publishUpdateRange();
}

void SDFMap::updateOccupancyCallback()
{
  if (!md_.occ_need_update_) return;

  auto t1 = node_->now();

  projectDepthImage();
  raycastProcess();

  if (md_.local_updated_) clearAndInflateLocalMap();

  auto t2 = node_->now();

  md_.fuse_time_ += (t2 - t1).seconds();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).seconds());

  if (mp_.show_occ_time_)
    RCLCPP_WARN(
      node_->get_logger(), "Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
      md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback()
{
  if (!md_.esdf_need_update_) return;

  auto t1 = node_->now();

  updateESDF3d();

  auto t2 = node_->now();

  md_.esdf_time_ += (t2 - t1).seconds();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).seconds());

  if (mp_.show_esdf_time_)
    RCLCPP_WARN(
      node_->get_logger(), "ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
      md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}




void SDFMap::depthPoseCallback(
  sensor_msgs::msg::Image::ConstSharedPtr img, geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(
    pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y,
    pose->pose.orientation.z);
  if (md_.camera_q_.norm() < 1e-6) md_.camera_q_ = Eigen::Quaterniond::Identity();
  md_.camera_q_.normalize();
  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.has_cloud_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void SDFMap::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (md_.has_first_depth_) return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void SDFMap::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*msg, latest_cloud);
  insertPointCloud(latest_cloud);
}

void SDFMap::vioPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose)
{

  md_.camera_pos_(0) = pose->pose.pose.position.x;
  md_.camera_pos_(1) = pose->pose.pose.position.y;
  md_.camera_pos_(2) = pose->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(
    pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
    pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
  if (md_.camera_q_.norm() < 1e-6) md_.camera_q_ = Eigen::Quaterniond::Identity();
  md_.camera_q_.normalize();

  md_.has_odom_ = isInMap(md_.camera_pos_);
}

void SDFMap::vioPointsCallback(visualization_msgs::msg::Marker::ConstSharedPtr marker)
{
  if (!md_.has_odom_) return;

  std::stringstream key_stream;
  key_stream << marker->ns << ":" << marker->id;
  const std::string key = key_stream.str();

  if (marker->action == visualization_msgs::msg::Marker::DELETEALL) {
    vio_marker_points_.clear();
    this->resetBuffer();
    md_.esdf_need_update_ = true;
    return;
  }

  if (marker->action == visualization_msgs::msg::Marker::DELETE) {
    vio_marker_points_.erase(key);
    return;
  }

  if (marker->action != visualization_msgs::msg::Marker::ADD) return;

  if (!mp_.vio_points_ns_filter_.empty() && marker->ns != mp_.vio_points_ns_filter_) return;

  Eigen::Vector3d t(
    marker->pose.position.x, marker->pose.position.y, marker->pose.position.z);
  Eigen::Quaterniond q(
    marker->pose.orientation.w, marker->pose.orientation.x,
    marker->pose.orientation.y, marker->pose.orientation.z);
  if (q.norm() < 1e-6) q = Eigen::Quaterniond::Identity();
  q.normalize();

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(std::max<size_t>(1, marker->points.size()));

  if (!marker->points.empty()) {
    for (const auto & point : marker->points) {
      Eigen::Vector3d local(point.x, point.y, point.z);
      pts.push_back(t + q * local);
    }
  } else {
    pts.push_back(t);
  }

  if (pts.empty()) return;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (mp_.vio_points_accumulate_) {
    vio_marker_points_[key] = pts;

    for (const auto & item : vio_marker_points_) {
      for (const auto & point : item.second) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        cloud.push_back(pcl_point);
      }
    }
  } else {
    for (const auto & point : pts) {
      pcl::PointXYZ pcl_point;
      pcl_point.x = point.x();
      pcl_point.y = point.y();
      pcl_point.z = point.z();
      cloud.push_back(pcl_point);
    }
  }

  insertPointCloud(cloud);
}

void SDFMap::insertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & latest_cloud)
{

  if (!md_.has_odom_) {
    return;
  }

  if (latest_cloud.points.empty()) return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

  md_.has_cloud_ = true;

  this->resetBuffer(
    md_.camera_pos_ - mp_.local_update_range_, md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (
      fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
      fabs(devi(2)) < mp_.local_update_range_(2)) {
      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {
            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.update_num_ += 1;
  md_.esdf_need_update_ = true;
}

void SDFMap::publishMap()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // VIO sparse input writes directly to the inflated occupancy buffer.
  // Keep /esdf_map/cloud useful in VIO mode by falling back to inflated cells.
  if (cloud.empty()) {
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {
          if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void SDFMap::publishMapInflate(bool all_info)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishOccupiedMap()
{
  visualization_msgs::msg::Marker mk;

  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = node_->now();
  mk.ns = "occupied_map";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 1.0;
  mk.scale.y = 1.0;
  mk.scale.z = 1.0;

  // RViz still needs marker.color.a to be nonzero even when per-vertex colors are used.
  mk.color.r = 1.0;
  mk.color.g = 1.0;
  mk.color.b = 1.0;
  mk.color.a = std::max(0.05, mp_.occupied_map_alpha_);

  Eigen::Vector3i min_cut;
  Eigen::Vector3i max_cut;

  if (mp_.occupied_map_local_only_) {
    min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  } else {
    min_cut = mp_.map_min_idx_;
    max_cut = mp_.map_max_idx_;
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  const int mesh_step = std::max(
    mp_.occupied_map_stride_,
    static_cast<int>(std::ceil(mp_.occupied_map_mesh_resolution_ / mp_.resolution_)));
  const double mesh_resolution = mesh_step * mp_.resolution_;
  const double half = 0.5 * mesh_resolution;
  const double z_min = mp_.ground_height_;
  const double z_max = std::max(mp_.ground_height_ + 1e-3, mp_.visualization_truncate_height_);

  auto occupied = [&](int x, int y, int z) -> bool {
    const Eigen::Vector3i id(x, y, z);
    if (!isInMap(id)) return false;
    const int address = toAddress(id);
    if (mp_.occupied_map_use_inflated_) {
      return md_.occupancy_buffer_inflate_[address] != 0;
    }
    return md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
  };

  auto block_key = [](int x, int y, int z) -> long long {
    return (static_cast<long long>(x) << 42) ^
           (static_cast<long long>(y) << 21) ^
           static_cast<long long>(z);
  };

  auto add_vertex = [&](const Eigen::Vector3d & v, const std_msgs::msg::ColorRGBA & color) {
    geometry_msgs::msg::Point point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    mk.points.push_back(point);
    mk.colors.push_back(color);
  };

  auto add_tri = [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b,
                     const Eigen::Vector3d & c, const std_msgs::msg::ColorRGBA & color) {
    add_vertex(a, color);
    add_vertex(b, color);
    add_vertex(c, color);
  };

  auto add_quad = [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b,
                      const Eigen::Vector3d & c, const Eigen::Vector3d & d,
                      const std_msgs::msg::ColorRGBA & color) {
    add_tri(a, b, c, color);
    add_tri(a, c, d, color);
  };

  std::unordered_map<long long, Eigen::Vector3i> blocks;
  blocks.reserve(4096);

  // Build a cube-shaped triangle mesh from occupied blocks.  This keeps the
  // occupied_map output as a mesh marker while giving the occupied cells a clear
  // square/cube visual shape in RViz2.
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (!occupied(x, y, z)) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        const int gx = x / mesh_step;
        const int gy = y / mesh_step;
        const int gz = z / mesh_step;
        const long long key = block_key(gx, gy, gz);
        if (blocks.find(key) == blocks.end()) {
          blocks.emplace(key, Eigen::Vector3i(gx, gy, gz));
        }
      }

  auto has_block = [&](int gx, int gy, int gz) -> bool {
    return blocks.find(block_key(gx, gy, gz)) != blocks.end();
  };

  for (const auto & item : blocks) {
    const Eigen::Vector3i & id = item.second;
    const int gx = id(0);
    const int gy = id(1);
    const int gz = id(2);

    const Eigen::Vector3d center(
      mp_.map_origin_(0) + (gx + 0.5) * mesh_resolution,
      mp_.map_origin_(1) + (gy + 0.5) * mesh_resolution,
      mp_.map_origin_(2) + (gz + 0.5) * mesh_resolution);

    const double height_ratio = (center.z() - z_min) / (z_max - z_min);
    const auto color = heightColor(height_ratio, mp_.occupied_map_alpha_);

    const Eigen::Vector3d v000(center.x() - half, center.y() - half, center.z() - half);
    const Eigen::Vector3d v100(center.x() + half, center.y() - half, center.z() - half);
    const Eigen::Vector3d v010(center.x() - half, center.y() + half, center.z() - half);
    const Eigen::Vector3d v110(center.x() + half, center.y() + half, center.z() - half);
    const Eigen::Vector3d v001(center.x() - half, center.y() - half, center.z() + half);
    const Eigen::Vector3d v101(center.x() + half, center.y() - half, center.z() + half);
    const Eigen::Vector3d v011(center.x() - half, center.y() + half, center.z() + half);
    const Eigen::Vector3d v111(center.x() + half, center.y() + half, center.z() + half);

    if (!has_block(gx - 1, gy, gz)) add_quad(v000, v001, v011, v010, color);  // -X
    if (!has_block(gx + 1, gy, gz)) add_quad(v100, v110, v111, v101, color);  // +X
    if (!has_block(gx, gy - 1, gz)) add_quad(v000, v100, v101, v001, color);  // -Y
    if (!has_block(gx, gy + 1, gz)) add_quad(v010, v011, v111, v110, color);  // +Y
    if (!has_block(gx, gy, gz - 1)) add_quad(v000, v010, v110, v100, color);  // -Z
    if (!has_block(gx, gy, gz + 1)) add_quad(v001, v101, v111, v011, color);  // +Z
  }

  occupied_map_pub_->publish(mk);
}


void SDFMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_->publish(cloud_msg);
}

void SDFMap::publishDepth()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_->publish(cloud_msg);
}

void SDFMap::publishUpdateRange()
{
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = node_->now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);
}

void SDFMap::publishESDF()
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut =
    md_.local_bound_min_ -
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut =
    md_.local_bound_max_ +
    Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_->publish(cloud_msg);

  // ROS_INFO("pub esdf");
}


void SDFMap::publishESDF3D()
{
  if (!md_.has_odom_ || !md_.has_cloud_) return;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  Eigen::Vector3i min_cut;
  Eigen::Vector3i max_cut;

  if (mp_.esdf_3d_local_only_) {
    min_cut = md_.local_bound_min_ - Eigen::Vector3i(
      mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    max_cut = md_.local_bound_max_ + Eigen::Vector3i(
      mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  } else {
    min_cut = mp_.map_min_idx_;
    max_cut = mp_.map_max_idx_;
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  const double min_dist = mp_.esdf_3d_min_distance_;
  const double max_dist = mp_.esdf_3d_max_distance_;
  const int stride = std::max(1, mp_.esdf_3d_stride_);

  for (int x = min_cut(0); x <= max_cut(0); x += stride)
    for (int y = min_cut(1); y <= max_cut(1); y += stride)
      for (int z = min_cut(2); z <= max_cut(2); z += stride) {
        const int idx = toAddress(x, y, z);
        double dist = md_.distance_buffer_all_[idx];

        // distance_buffer_all_ keeps 10000 for voxels that have not been updated by ESDF
        if (dist > 9999.0) continue;

        if (!mp_.esdf_3d_publish_negative_distance_ && dist < 0.0) continue;

        double vis_dist = std::abs(dist);
        if (vis_dist > max_dist) continue;
        vis_dist = std::max(vis_dist, min_dist);

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        pt.intensity = (vis_dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  esdf_3d_pub_->publish(cloud_msg);
}


void SDFMap::publishESDFDistance()
{
  if (!md_.has_odom_ || !md_.has_cloud_) return;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  Eigen::Vector3i min_cut;
  Eigen::Vector3i max_cut;

  if (mp_.esdf_3d_local_only_) {
    min_cut = md_.local_bound_min_ - Eigen::Vector3i(
      mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    max_cut = md_.local_bound_max_ + Eigen::Vector3i(
      mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  } else {
    min_cut = mp_.map_min_idx_;
    max_cut = mp_.map_max_idx_;
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  const int stride = std::max(1, mp_.esdf_3d_stride_);

  for (int x = min_cut(0); x <= max_cut(0); x += stride)
    for (int y = min_cut(1); y <= max_cut(1); y += stride)
      for (int z = min_cut(2); z <= max_cut(2); z += stride) {
        const int idx = toAddress(x, y, z);
        const double dist = md_.distance_buffer_all_[idx];

        // distance_buffer_all_ keeps 10000 for voxels that have not been updated by ESDF
        if (dist > 9999.0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        pt.intensity = static_cast<float>(dist);  // signed distance in meters, not normalized
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  esdf_distance_pub_->publish(cloud_msg);
}

void SDFMap::getSliceESDF(
  const double height, const double res, const Eigen::Vector4d & range,
  vector<Eigen::Vector3d> & slice, vector<Eigen::Vector3d> & grad, int sign)
{
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {
      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist()
{
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
        }
      }
}

bool SDFMap::odomValid()
{
  return md_.has_odom_;
}

bool SDFMap::hasDepthObservation()
{
  return md_.has_first_depth_;
}

double SDFMap::getResolution()
{
  return mp_.resolution_;
}

Eigen::Vector3d SDFMap::getOrigin()
{
  return mp_.map_origin_;
}

int SDFMap::getVoxelNum()
{
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d & ori, Eigen::Vector3d & size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(
  const Eigen::Vector3d & pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d & diff)
{
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(
  sensor_msgs::msg::Image::ConstSharedPtr img, nav_msgs::msg::Odometry::ConstSharedPtr odom)
{

  /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(
    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z);
  if (md_.camera_q_.norm() < 1e-6) md_.camera_q_ = Eigen::Quaterniond::Identity();
  md_.camera_q_.normalize();

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.has_cloud_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void SDFMap::depthCallback(sensor_msgs::msg::Image::ConstSharedPtr img)
{
  if (!md_.has_odom_) return;

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (isInMap(md_.camera_pos_)) {
    md_.has_cloud_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}


void SDFMap::poseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(
    pose->pose.orientation.w, pose->pose.orientation.x,
    pose->pose.orientation.y, pose->pose.orientation.z);
  if (md_.camera_q_.norm() < 1e-6) md_.camera_q_ = Eigen::Quaterniond::Identity();
  md_.camera_q_.normalize();
  md_.has_odom_ = isInMap(md_.camera_pos_);
}


// SDFMap

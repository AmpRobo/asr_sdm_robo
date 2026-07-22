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
#include "binary_map_io.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <rmw/qos_profiles.h>

namespace
{

constexpr double kGridComparisonTolerance = 1e-6;

bool nearlyEqual(const double lhs, const double rhs)
{
  const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
  return std::abs(lhs - rhs) <= kGridComparisonTolerance * scale;
}

bool isTargetVoxelCenter(
  const Eigen::Vector3d & center, const Eigen::Vector3d & target_origin,
  const double target_resolution)
{
  const Eigen::Array3d voxel_coordinate =
    (center - target_origin).array() / target_resolution - 0.5;
  return (voxel_coordinate - voxel_coordinate.round()).abs().maxCoeff() <=
         kGridComparisonTolerance;
}

bool boxesOverlapWithVolume(
  const Eigen::Vector3d & first_min, const Eigen::Vector3d & first_max,
  const Eigen::Vector3d & second_min, const Eigen::Vector3d & second_max,
  const double tolerance)
{
  const Eigen::Array3d overlap =
    first_max.cwiseMin(second_max).array() -
    first_min.cwiseMax(second_min).array();
  return (overlap > tolerance).all();
}

}  // namespace

void ESDFMap::initMap(const std::shared_ptr<rclcpp::Node> & node)
{
  node_ = node;

  double x_size, y_size, z_size;
  node_->declare_parameter("esdf_map.resolution", -1.0);
  node_->declare_parameter("esdf_map.map_size_x", -1.0);
  node_->declare_parameter("esdf_map.map_size_y", -1.0);
  node_->declare_parameter("esdf_map.map_size_z", -1.0);
  node_->declare_parameter("esdf_map.local_update_range_x", -1.0);
  node_->declare_parameter("esdf_map.local_update_range_y", -1.0);
  node_->declare_parameter("esdf_map.local_update_range_z", -1.0);
  node_->declare_parameter("esdf_map.obstacles_inflation", -1.0);
  node_->declare_parameter("esdf_map.fx", -1.0);
  node_->declare_parameter("esdf_map.fy", -1.0);
  node_->declare_parameter("esdf_map.cx", -1.0);
  node_->declare_parameter("esdf_map.cy", -1.0);
  node_->declare_parameter("esdf_map.use_depth_filter", true);
  node_->declare_parameter("esdf_map.depth_filter_tolerance", -1.0);
  node_->declare_parameter("esdf_map.depth_filter_maxdist", -1.0);
  node_->declare_parameter("esdf_map.depth_filter_mindist", -1.0);
  node_->declare_parameter("esdf_map.depth_filter_margin", -1);
  node_->declare_parameter("esdf_map.k_depth_scaling_factor", -1.0);
  node_->declare_parameter("esdf_map.skip_pixel", -1);
  node_->declare_parameter("esdf_map.p_hit", 0.70);
  node_->declare_parameter("esdf_map.p_miss", 0.35);
  node_->declare_parameter("esdf_map.p_min", 0.12);
  node_->declare_parameter("esdf_map.p_max", 0.97);
  node_->declare_parameter("esdf_map.p_occ", 0.80);
  node_->declare_parameter("esdf_map.min_ray_length", -0.1);
  node_->declare_parameter("esdf_map.max_ray_length", -0.1);
  node_->declare_parameter("esdf_map.virtual_ceil_height", -0.1);
  node_->declare_parameter("esdf_map.show_occ_time", false);
  node_->declare_parameter("esdf_map.show_esdf_time", false);
  node_->declare_parameter("esdf_map.local_bound_inflate", 1.0);
  node_->declare_parameter("esdf_map.local_map_margin", 1);
  node_->declare_parameter("esdf_map.ground_height", 1.0);

  node_->declare_parameter(
    "esdf_map.depth_topic", string("/sensing/camera/realsense/depth"));
  node_->declare_parameter(
    "esdf_map.odom_topic", string("/localization/vins/odometry"));
  node_->declare_parameter(
    "esdf_map.cloud_topic", string("/localization/vins/point_cloud"));
  node_->declare_parameter("esdf_map.enable_depth_odom", true);
  node_->declare_parameter("esdf_map.enable_pointcloud_odom", true);
  node_->declare_parameter("esdf_map.preload_map_directory", string("maps"));
  node_->declare_parameter("esdf_map.preload_occupancy_filename", string("occupancy.bin"));
  node_->declare_parameter("esdf_map.preload_esdf_filename", string("esdf.bin"));
  node_->declare_parameter("esdf_map.preload_source_resolution", -1.0);

  mp_.resolution_ = node_->get_parameter("esdf_map.resolution").as_double();
  x_size = node_->get_parameter("esdf_map.map_size_x").as_double();
  y_size = node_->get_parameter("esdf_map.map_size_y").as_double();
  z_size = node_->get_parameter("esdf_map.map_size_z").as_double();
  mp_.local_update_range_(0) = node_->get_parameter("esdf_map.local_update_range_x").as_double();
  mp_.local_update_range_(1) = node_->get_parameter("esdf_map.local_update_range_y").as_double();
  mp_.local_update_range_(2) = node_->get_parameter("esdf_map.local_update_range_z").as_double();
  mp_.obstacles_inflation_ = node_->get_parameter("esdf_map.obstacles_inflation").as_double();
  mp_.fx_ = node_->get_parameter("esdf_map.fx").as_double();
  mp_.fy_ = node_->get_parameter("esdf_map.fy").as_double();
  mp_.cx_ = node_->get_parameter("esdf_map.cx").as_double();
  mp_.cy_ = node_->get_parameter("esdf_map.cy").as_double();
  mp_.use_depth_filter_ = node_->get_parameter("esdf_map.use_depth_filter").as_bool();
  mp_.depth_filter_tolerance_ = node_->get_parameter("esdf_map.depth_filter_tolerance").as_double();
  mp_.depth_filter_maxdist_ = node_->get_parameter("esdf_map.depth_filter_maxdist").as_double();
  mp_.depth_filter_mindist_ = node_->get_parameter("esdf_map.depth_filter_mindist").as_double();
  mp_.depth_filter_margin_ = node_->get_parameter("esdf_map.depth_filter_margin").as_int();
  mp_.k_depth_scaling_factor_ = node_->get_parameter("esdf_map.k_depth_scaling_factor").as_double();
  mp_.skip_pixel_ = node_->get_parameter("esdf_map.skip_pixel").as_int();
  mp_.p_hit_ = node_->get_parameter("esdf_map.p_hit").as_double();
  mp_.p_miss_ = node_->get_parameter("esdf_map.p_miss").as_double();
  mp_.p_min_ = node_->get_parameter("esdf_map.p_min").as_double();
  mp_.p_max_ = node_->get_parameter("esdf_map.p_max").as_double();
  mp_.p_occ_ = node_->get_parameter("esdf_map.p_occ").as_double();
  mp_.min_ray_length_ = node_->get_parameter("esdf_map.min_ray_length").as_double();
  mp_.max_ray_length_ = node_->get_parameter("esdf_map.max_ray_length").as_double();
  mp_.virtual_ceil_height_ = node_->get_parameter("esdf_map.virtual_ceil_height").as_double();
  mp_.show_occ_time_ = node_->get_parameter("esdf_map.show_occ_time").as_bool();
  mp_.show_esdf_time_ = node_->get_parameter("esdf_map.show_esdf_time").as_bool();
  mp_.local_bound_inflate_ = node_->get_parameter("esdf_map.local_bound_inflate").as_double();
  mp_.local_map_margin_ = node_->get_parameter("esdf_map.local_map_margin").as_int();
  mp_.ground_height_ = node_->get_parameter("esdf_map.ground_height").as_double();
  mp_.depth_topic_ = node_->get_parameter("esdf_map.depth_topic").as_string();
  mp_.odom_topic_ = node_->get_parameter("esdf_map.odom_topic").as_string();
  mp_.cloud_topic_ = node_->get_parameter("esdf_map.cloud_topic").as_string();
  mp_.enable_depth_odom_ = node_->get_parameter("esdf_map.enable_depth_odom").as_bool();
  mp_.enable_pointcloud_odom_ =
    node_->get_parameter("esdf_map.enable_pointcloud_odom").as_bool();
  mp_.preload_map_directory_ = node_->get_parameter("esdf_map.preload_map_directory").as_string();
  mp_.preload_occupancy_filename_ =
    node_->get_parameter("esdf_map.preload_occupancy_filename").as_string();
  mp_.preload_esdf_filename_ =
    node_->get_parameter("esdf_map.preload_esdf_filename").as_string();
  mp_.preload_source_resolution_ =
    node_->get_parameter("esdf_map.preload_source_resolution").as_double();

  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0) {
    throw std::invalid_argument("esdf_map.resolution must be finite and positive.");
  }

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

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  int buffer_size =
    mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

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

  const auto update_period = std::chrono::milliseconds(50);

  if (mp_.enable_depth_odom_) {
    rmw_qos_profile_t depth_qos = rmw_qos_profile_sensor_data;
    depth_qos.depth = 50;
    rmw_qos_profile_t odom_qos = rmw_qos_profile_sensor_data;
    odom_qos.depth = 100;

    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_sub_->subscribe(node_.get(), mp_.depth_topic_, depth_qos);

    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    odom_sub_->subscribe(node_.get(), mp_.odom_topic_, odom_qos);
    sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
      SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
    sync_image_odom_->registerCallback(
      std::bind(&ESDFMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

    occ_timer_ = node_->create_wall_timer(
      update_period, std::bind(&ESDFMap::updateOccupancyCallback, this));
  }

  if (mp_.enable_pointcloud_odom_) {
    auto independent_qos = rclcpp::SensorDataQoS();
    independent_qos.keep_last(10);
    indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(
      mp_.cloud_topic_, independent_qos,
      std::bind(&ESDFMap::cloudCallback, this, std::placeholders::_1));
    indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      mp_.odom_topic_, independent_qos,
      std::bind(&ESDFMap::odomCallback, this, std::placeholders::_1));
  }

  if (mp_.enable_depth_odom_ || mp_.enable_pointcloud_odom_) {
    esdf_timer_ = node_->create_wall_timer(
      update_period, std::bind(&ESDFMap::updateESDFCallback, this));
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "Live map inputs are disabled; running in preload-only mode.");
  }

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  loadPreloadedMaps();

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());
}

bool ESDFMap::loadPreloadedMaps()
{
  if (mp_.preload_map_directory_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "esdf_map preload disabled (empty preload_map_directory).");
    return false;
  }

  std::filesystem::path directory(mp_.preload_map_directory_);
  if (!directory.is_absolute()) {
    try {
      directory = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("asr_sdm_esdf_map")) / directory;
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to resolve package share directory for preload maps: %s",
        exception.what());
    }
  }

  const std::string occupancy_path =
    (directory / mp_.preload_occupancy_filename_).string();
  const std::string esdf_path = (directory / mp_.preload_esdf_filename_).string();

  RCLCPP_INFO(
    node_->get_logger(), "Preloading map files: occupancy=%s, esdf=%s",
    occupancy_path.c_str(), esdf_path.c_str());

  md_.local_bound_min_ = mp_.map_max_idx_;
  md_.local_bound_max_ = mp_.map_min_idx_;
  preloaded_occupancy_grid_matches_target_ = false;

  std::string occupancy_status;
  const bool occupancy_loaded = loadOccupancyBinary(occupancy_path, occupancy_status);
  if (occupancy_loaded) {
    RCLCPP_INFO(node_->get_logger(), "%s", occupancy_status.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "%s", occupancy_status.c_str());
  }

  bool esdf_available = false;
  if (occupancy_loaded && !preloaded_occupancy_grid_matches_target_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Source occupancy grid differs from the target grid. The preloaded ESDF file is ignored "
      "and the ESDF is rebuilt at target resolution %.6f m.",
      mp_.resolution_);
    rebuildEsdfFromOccupancy();
    esdf_available = true;
  } else {
    std::string esdf_status;
    esdf_available = loadEsdfBinary(esdf_path, esdf_status);
    if (esdf_available) {
      RCLCPP_INFO(node_->get_logger(), "%s", esdf_status.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", esdf_status.c_str());
      if (occupancy_loaded) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Rebuilding ESDF from the loaded occupancy map at target resolution %.6f m.",
          mp_.resolution_);
        rebuildEsdfFromOccupancy();
        esdf_available = true;
      }
    }
  }

  if (!occupancy_loaded && !esdf_available) {
    md_.local_bound_min_ = Eigen::Vector3i::Zero();
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    return false;
  }

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  return true;
}

bool ESDFMap::setPreloadedOccupiedVoxel(
  const Eigen::Vector3i & target_id, std::size_t & inserted_target_voxels)
{
  if (!isInMap(target_id)) {
    return false;
  }

  const int address = toAddress(target_id);
  if (md_.occupancy_buffer_inflate_[address] != 1) {
    ++inserted_target_voxels;
  }
  md_.occupancy_buffer_[address] = mp_.clamp_max_log_;
  md_.occupancy_buffer_inflate_[address] = 1;
  md_.local_bound_min_ = md_.local_bound_min_.cwiseMin(target_id);
  md_.local_bound_max_ = md_.local_bound_max_.cwiseMax(target_id);
  return true;
}

bool ESDFMap::insertPreloadedSourceVoxel(
  const Eigen::Vector3d & source_center, const double source_resolution,
  std::size_t & inserted_target_voxels)
{
  const double source_half_size = 0.5 * source_resolution;
  const Eigen::Vector3d source_min =
    source_center - Eigen::Vector3d::Constant(source_half_size);
  const Eigen::Vector3d source_max =
    source_center + Eigen::Vector3d::Constant(source_half_size);
  const Eigen::Vector3d target_grid_max =
    mp_.map_origin_ + mp_.map_voxel_num_.cast<double>() * mp_.resolution_;
  const double overlap_tolerance =
    std::max(1e-9, std::min(source_resolution, mp_.resolution_) * 1e-6);

  if (
    (source_max.array() <= mp_.map_origin_.array() + overlap_tolerance).any() ||
    (source_min.array() >= target_grid_max.array() - overlap_tolerance).any())
  {
    return false;
  }

  Eigen::Vector3i candidate_min;
  Eigen::Vector3i candidate_max;
  for (int axis = 0; axis < 3; ++axis) {
    candidate_min(axis) = static_cast<int>(std::floor(
      (source_min(axis) - mp_.map_origin_(axis)) / mp_.resolution_));
    candidate_max(axis) = static_cast<int>(std::floor(
      (source_max(axis) - overlap_tolerance - mp_.map_origin_(axis)) /
      mp_.resolution_));
  }
  candidate_min = candidate_min.cwiseMax(mp_.map_min_idx_);
  candidate_max = candidate_max.cwiseMin(mp_.map_max_idx_);

  bool inserted = false;
  for (int x = candidate_min.x(); x <= candidate_max.x(); ++x) {
    for (int y = candidate_min.y(); y <= candidate_max.y(); ++y) {
      for (int z = candidate_min.z(); z <= candidate_max.z(); ++z) {
        const Eigen::Vector3i target_id(x, y, z);
        const Eigen::Vector3d target_min =
          mp_.map_origin_ + target_id.cast<double>() * mp_.resolution_;
        const Eigen::Vector3d target_max =
          target_min + Eigen::Vector3d::Constant(mp_.resolution_);

        if (!boxesOverlapWithVolume(
              source_min, source_max, target_min, target_max, overlap_tolerance))
        {
          continue;
        }

        inserted = setPreloadedOccupiedVoxel(target_id, inserted_target_voxels) || inserted;
      }
    }
  }
  return inserted;
}

bool ESDFMap::loadOccupancyBinary(const std::string & path, std::string & status)
{
  asr_sdm_esdf_map::binary_map::OccupancyData source_map;
  std::string read_error;
  if (!asr_sdm_esdf_map::binary_map::readOccupancy(path, source_map, read_error)) {
    status = "esdf_map: " + read_error;
    return false;
  }

  const double source_resolution = mp_.preload_source_resolution_;
  if (!std::isfinite(source_resolution) || source_resolution <= 0.0) {
    status =
      "esdf_map: set esdf_map.preload_source_resolution to the voxel size used "
      "when occupancy.bin was saved: " + path;
    return false;
  }

  const bool same_resolution = nearlyEqual(source_resolution, mp_.resolution_);
  const bool centers_aligned = std::all_of(
    source_map.occupied_centers.begin(), source_map.occupied_centers.end(),
    [&](const Eigen::Vector3d & center) {
      return isTargetVoxelCenter(center, mp_.map_origin_, mp_.resolution_);
    });
  preloaded_occupancy_grid_matches_target_ = same_resolution && centers_aligned;

  std::size_t source_records_loaded = 0;
  std::size_t target_voxels_inserted = 0;
  std::size_t out_of_map_records = 0;

  for (const Eigen::Vector3d & source_center : source_map.occupied_centers) {
    bool inserted = false;
    if (preloaded_occupancy_grid_matches_target_) {
      Eigen::Vector3i target_id;
      posToIndex(source_center, target_id);
      inserted = setPreloadedOccupiedVoxel(target_id, target_voxels_inserted);
    } else {
      inserted = insertPreloadedSourceVoxel(
        source_center, source_resolution, target_voxels_inserted);
    }

    if (inserted) {
      ++source_records_loaded;
    } else {
      ++out_of_map_records;
    }
  }

  const std::string conversion_mode =
    preloaded_occupancy_grid_matches_target_ ? "direct" : "conservative_aabb";
  status =
    "esdf_map: loaded occupancy binary frame=" + source_map.frame_id +
    ", source_records=" + std::to_string(source_map.record_count) +
    ", loaded_source_records=" + std::to_string(source_records_loaded) +
    ", inserted_target_voxels=" + std::to_string(target_voxels_inserted) +
    ", invalid_records=" + std::to_string(source_map.invalid_records) +
    ", out_of_map_records=" + std::to_string(out_of_map_records) +
    ", source_resolution=" + std::to_string(source_resolution) +
    ", target_resolution=" + std::to_string(mp_.resolution_) +
    ", conversion=" + conversion_mode + " from " + path;
  return source_records_loaded > 0;
}

bool ESDFMap::loadEsdfBinary(const std::string & path, std::string & status)
{
  asr_sdm_esdf_map::binary_map::EsdfData source_map;
  std::string read_error;
  if (!asr_sdm_esdf_map::binary_map::readEsdf(path, source_map, read_error)) {
    status = "esdf_map: " + read_error;
    return false;
  }

  const double source_resolution = mp_.preload_source_resolution_;
  if (!std::isfinite(source_resolution) || source_resolution <= 0.0) {
    status =
      "esdf_map: set esdf_map.preload_source_resolution to the voxel size used "
      "when esdf.bin was saved: " + path;
    return false;
  }

  const bool source_grid_matches_target =
    nearlyEqual(source_resolution, mp_.resolution_) &&
    std::all_of(
      source_map.samples.begin(), source_map.samples.end(),
      [&](const asr_sdm_esdf_map::binary_map::EsdfSample & sample) {
        return isTargetVoxelCenter(sample.center, mp_.map_origin_, mp_.resolution_);
      });
  if (!source_grid_matches_target) {
    status =
      "esdf_map: ignored preloaded ESDF because its source grid does not match the target grid "
      "(source_resolution=" + std::to_string(source_resolution) +
      ", target_resolution=" + std::to_string(mp_.resolution_) + "): " + path;
    return false;
  }

  std::size_t loaded = 0;
  std::size_t out_of_map_records = 0;
  for (const auto & sample : source_map.samples) {
    Eigen::Vector3i target_id;
    posToIndex(sample.center, target_id);
    if (!isInMap(target_id)) {
      ++out_of_map_records;
      continue;
    }

    const int address = toAddress(target_id);
    md_.distance_buffer_all_[address] = sample.distance;
    md_.distance_buffer_[address] = std::max(0.0, sample.distance);
    md_.local_bound_min_ = md_.local_bound_min_.cwiseMin(target_id);
    md_.local_bound_max_ = md_.local_bound_max_.cwiseMax(target_id);
    ++loaded;
  }

  status =
    "esdf_map: loaded ESDF binary frame=" + source_map.frame_id +
    ", source_records=" + std::to_string(source_map.record_count) +
    ", loaded_records=" + std::to_string(loaded) +
    ", invalid_records=" + std::to_string(source_map.invalid_records) +
    ", out_of_map_records=" + std::to_string(out_of_map_records) +
    ", resolution=" + std::to_string(source_resolution) + " from " + path;
  return loaded > 0;
}

void ESDFMap::rebuildEsdfFromOccupancy()
{
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  updateESDF3d();
  md_.esdf_need_update_ = false;
}

void ESDFMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void ESDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
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
void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
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

void ESDFMap::updateESDF3d()
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


int ESDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;
  if (md_.count_hit_and_miss_[idx_ctns] == 1) md_.cache_voxel_.push(id);
  if (occ == 1) md_.count_hit_[idx_ctns] += 1;
  return idx_ctns;
}

void ESDFMap::projectDepthImage()
{
  md_.proj_points_cnt = 0;
  uint16_t * row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  double depth;
  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);
      for (int u = 0; u < cols; u++) {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;
        proj_pt = camera_r * proj_pt + md_.camera_pos_;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  } else {
    if (!md_.has_first_depth_) {
      md_.has_first_depth_ = true;
    } else {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;
      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_;
           v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;
        for (int u = mp_.depth_filter_margin_;
             u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_) {
          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;
          pt_world = camera_r * pt_cur + md_.camera_pos_;
          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;
            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(
                    md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                    pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            } else {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void ESDFMap::raycastProcess()
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

Eigen::Vector3d ESDFMap::closetPointInMap(
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

void ESDFMap::clearAndInflateLocalMap()
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


void ESDFMap::updateOccupancyCallback()
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

void ESDFMap::updateESDFCallback()
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





void ESDFMap::depthOdomCallback(
  sensor_msgs::msg::Image::ConstSharedPtr img,
  nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(
    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void ESDFMap::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (md_.has_first_depth_) return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.has_odom_ = true;
}

void ESDFMap::cloudCallback(sensor_msgs::msg::PointCloud::ConstSharedPtr msg)
{
  if (!md_.has_odom_) return;
  if (msg->points.empty()) return;
  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

  resetBuffer(
    md_.camera_pos_ - mp_.local_update_range_,
    md_.camera_pos_ + mp_.local_update_range_);

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

  for (size_t i = 0; i < msg->points.size(); ++i) {
    const auto & pt = msg->points[i];
    p3d(0) = pt.x;
    p3d(1) = pt.y;
    p3d(2) = pt.z;

    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) &&
        fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2)) {
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
            md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
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

  md_.esdf_need_update_ = true;
}

void ESDFMap::getSliceESDF(
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

void ESDFMap::checkDist()
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

bool ESDFMap::odomValid()
{
  return md_.has_odom_;
}

bool ESDFMap::hasDepthObservation()
{
  return md_.has_first_depth_;
}

double ESDFMap::getResolution()
{
  return mp_.resolution_;
}

Eigen::Vector3d ESDFMap::getOrigin()
{
  return mp_.map_origin_;
}

int ESDFMap::getVoxelNum()
{
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void ESDFMap::getRegion(Eigen::Vector3d & ori, Eigen::Vector3d & size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void ESDFMap::getSurroundPts(
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

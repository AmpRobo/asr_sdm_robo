#include <asr_sdm_esdf_map/esdf_map.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

class ESDFMapTestNode : public rclcpp::Node
{
public:
  ESDFMapTestNode()
  : Node("esdf_map")
  {
    declare_parameter("test.frame_id", std::string("world"));
    declare_parameter("test.visualization_truncate_height", 3.0);
    declare_parameter("test.esdf_max_valid_distance", 1000.0);
    declare_parameter("test.publish_period", 0.5);
    declare_parameter("test.odometry_path_min_distance", 0.05);
    declare_parameter("test.odometry_path_max_poses", 5000);
  }

  void initialize()
  {
    esdf_map_ = std::make_shared<ESDFMap>();
    esdf_map_->initMap(shared_from_this());

    loadInputParameters();
    initializeRawCloudInputs();

    // Map outputs are latched so RViz can start after a preload-only test and
    // still receive the latest static map.
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    raw_occupancy_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/map/esdf_map/cloud", map_qos);
    inflated_occupancy_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/map/esdf_map/occupancy_inflate", map_qos);
    esdf_distance_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map/esdf_map/esdf_distance", map_qos);
    esdf_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/map/esdf_map/esdf", map_qos);
    occupied_map_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("/map/esdf_map/occupied_map", map_qos);

    // Publish odometry using standard ROS message types. The path is transient
    // local so a late-starting RViz receives the latest bounded history; the
    // current pose remains a normal live stream.
    auto odometry_path_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    odometry_path_qos.reliable();
    odometry_path_qos.transient_local();
    odometry_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/map/esdf_map/odometry_path", odometry_path_qos);

    auto odometry_pose_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    odometry_pose_qos.reliable();
    odometry_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/map/esdf_map/odometry_pose", odometry_pose_qos);

    auto odometry_qos = rclcpp::SensorDataQoS();
    odometry_qos.keep_last(10);
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odometry_qos,
      std::bind(
        &ESDFMapTestNode::odometryCallback, this,
        std::placeholders::_1));

    publishMaps();

    double period = get_parameter("test.publish_period").as_double();
    if (!std::isfinite(period) || period <= 0.0) {
      RCLCPP_WARN(get_logger(), "test.publish_period must be positive; using 0.5 s.");
      period = 0.5;
    }
    vis_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&ESDFMapTestNode::publishMaps, this));
  }

private:
  using DepthOdomSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, nav_msgs::msg::Odometry>;

  void loadInputParameters()
  {
    enable_depth_odom_ = get_parameter("esdf_map.enable_depth_odom").as_bool();
    enable_pointcloud_odom_ = get_parameter("esdf_map.enable_pointcloud_odom").as_bool();
    depth_topic_ = get_parameter("esdf_map.depth_topic").as_string();
    odom_topic_ = get_parameter("esdf_map.odom_topic").as_string();
    cloud_topic_ = get_parameter("esdf_map.cloud_topic").as_string();

    local_update_range_.x() =
      get_parameter("esdf_map.local_update_range_x").as_double();
    local_update_range_.y() =
      get_parameter("esdf_map.local_update_range_y").as_double();
    local_update_range_.z() =
      get_parameter("esdf_map.local_update_range_z").as_double();

    fx_ = get_parameter("esdf_map.fx").as_double();
    fy_ = get_parameter("esdf_map.fy").as_double();
    cx_ = get_parameter("esdf_map.cx").as_double();
    cy_ = get_parameter("esdf_map.cy").as_double();
    use_depth_filter_ = get_parameter("esdf_map.use_depth_filter").as_bool();
    depth_filter_maxdist_ = get_parameter("esdf_map.depth_filter_maxdist").as_double();
    depth_filter_mindist_ = get_parameter("esdf_map.depth_filter_mindist").as_double();
    depth_filter_margin_ =
      static_cast<int>(get_parameter("esdf_map.depth_filter_margin").as_int());
    depth_scaling_factor_ =
      get_parameter("esdf_map.k_depth_scaling_factor").as_double();
    skip_pixel_ = static_cast<int>(get_parameter("esdf_map.skip_pixel").as_int());
    max_ray_length_ = get_parameter("esdf_map.max_ray_length").as_double();

    odometry_path_min_distance_ =
      get_parameter("test.odometry_path_min_distance").as_double();
    const std::int64_t odometry_path_max_poses =
      get_parameter("test.odometry_path_max_poses").as_int();
    if (!std::isfinite(odometry_path_min_distance_) || odometry_path_min_distance_ < 0.0) {
      throw std::runtime_error(
              "test.odometry_path_min_distance must be finite and non-negative.");
    }
    if (odometry_path_max_poses < 3) {
      throw std::runtime_error("test.odometry_path_max_poses must be at least 3.");
    }
    odometry_path_max_poses_ = static_cast<std::size_t>(odometry_path_max_poses);

    if (
      enable_pointcloud_odom_ &&
      (!local_update_range_.allFinite() || (local_update_range_.array() <= 0.0).any()))
    {
      throw std::runtime_error(
              "Point-cloud visualization requires positive local_update_range values.");
    }
    if (enable_depth_odom_) {
      if (fx_ <= 0.0 || fy_ <= 0.0 || depth_scaling_factor_ <= 0.0) {
        throw std::runtime_error(
                "Depth visualization requires positive fx, fy, and k_depth_scaling_factor.");
      }
      if (skip_pixel_ <= 0 || depth_filter_margin_ < 0) {
        throw std::runtime_error(
                "Depth visualization requires skip_pixel > 0 and depth_filter_margin >= 0.");
      }
      if (
        use_depth_filter_ &&
        (depth_filter_mindist_ <= 0.0 || depth_filter_maxdist_ <= depth_filter_mindist_))
      {
        throw std::runtime_error(
                "Depth visualization requires 0 < depth_filter_mindist < "
                "depth_filter_maxdist.");
      }
    }
  }

  void initializeRawCloudInputs()
  {
    // These subscriptions are test-only collectors for /map/esdf_map/cloud. They
    // deliberately do not inspect the map buffers, so occupancy.bin can never
    // leak into the raw live-observation topic.
    if (enable_pointcloud_odom_) {
      auto sensor_qos = rclcpp::SensorDataQoS();
      sensor_qos.keep_last(10);
      raw_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud>(
        cloud_topic_, sensor_qos,
        std::bind(&ESDFMapTestNode::pointCloudCallback, this, std::placeholders::_1));
      raw_cloud_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos,
        std::bind(&ESDFMapTestNode::pointCloudOdomCallback, this, std::placeholders::_1));
    }

    if (enable_depth_odom_) {
      rmw_qos_profile_t depth_qos = rmw_qos_profile_sensor_data;
      depth_qos.depth = 50;
      rmw_qos_profile_t odom_qos = rmw_qos_profile_sensor_data;
      odom_qos.depth = 100;

      raw_depth_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
      raw_depth_odom_sub_ =
        std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
      raw_depth_sub_->subscribe(this, depth_topic_, depth_qos);
      raw_depth_odom_sub_->subscribe(this, odom_topic_, odom_qos);
      raw_depth_odom_sync_ =
        std::make_shared<message_filters::Synchronizer<DepthOdomSyncPolicy>>(
        DepthOdomSyncPolicy(100), *raw_depth_sub_, *raw_depth_odom_sub_);
      raw_depth_odom_sync_->registerCallback(
        std::bind(
          &ESDFMapTestNode::depthOdomCallback, this,
          std::placeholders::_1, std::placeholders::_2));
    }
  }

  template <typename PointT>
  void publishCloud(
    pcl::PointCloud<PointT> & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    publisher->publish(msg);
  }

  static std_msgs::msg::ColorRGBA heightColor(
    const double z, const double min_z, const double max_z)
  {
    const double denominator = std::max(max_z - min_z, 1e-6);
    const double value = std::clamp((z - min_z) / denominator, 0.0, 1.0);

    // Match the guidance planner's height-color palette.
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(
      std::clamp(1.5 - std::abs(4.0 * value - 3.0), 0.0, 1.0));
    color.g = static_cast<float>(
      std::clamp(1.5 - std::abs(4.0 * value - 2.0), 0.0, 1.0));
    color.b = static_cast<float>(
      std::clamp(1.5 - std::abs(4.0 * value - 1.0), 0.0, 1.0));
    color.a = 1.0F;
    return color;
  }

  static geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d & position)
  {
    geometry_msgs::msg::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    return point;
  }

  void odometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const Eigen::Vector3d position(
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z);
    Eigen::Quaterniond orientation(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);

    if (!position.allFinite() || !orientation.coeffs().allFinite()) {
      return;
    }
    if (orientation.norm() < 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring odometry visualization with an invalid zero-norm quaternion.");
      return;
    }
    orientation.normalize();

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header = odom->header;
    if (current_pose.header.frame_id.empty()) {
      current_pose.header.frame_id = get_parameter("test.frame_id").as_string();
    }
    current_pose.pose = odom->pose.pose;
    current_pose.pose.orientation.x = orientation.x();
    current_pose.pose.orientation.y = orientation.y();
    current_pose.pose.orientation.z = orientation.z();
    current_pose.pose.orientation.w = orientation.w();
    odometry_pose_pub_->publish(current_pose);

    bool reset_path =
      !odometry_path_.header.frame_id.empty() &&
      odometry_path_.header.frame_id != current_pose.header.frame_id;
    if (!odometry_path_.poses.empty()) {
      const rclcpp::Time current_stamp(current_pose.header.stamp);
      const rclcpp::Time last_stamp(odometry_path_.poses.back().header.stamp);
      reset_path = reset_path || current_stamp < last_stamp;
    }
    if (reset_path) {
      odometry_path_.poses.clear();
    }

    odometry_path_.header = current_pose.header;
    bool append_pose = odometry_path_.poses.empty();
    if (!append_pose) {
      const auto & last_position = odometry_path_.poses.back().pose.position;
      const Eigen::Vector3d previous_position(
        last_position.x, last_position.y, last_position.z);
      append_pose =
        (position - previous_position).norm() >= odometry_path_min_distance_;
    }

    if (!append_pose) {
      return;
    }

    if (odometry_path_.poses.size() >= odometry_path_max_poses_) {
      // Preserve the complete route while bounding memory and DDS message size:
      // older history is progressively decimated instead of being discarded.
      std::vector<geometry_msgs::msg::PoseStamped> compacted_path;
      compacted_path.reserve(odometry_path_.poses.size() / 2 + 1);
      for (std::size_t index = 0; index < odometry_path_.poses.size(); index += 2) {
        compacted_path.push_back(std::move(odometry_path_.poses[index]));
      }
      if ((odometry_path_.poses.size() - 1) % 2 != 0) {
        compacted_path.push_back(std::move(odometry_path_.poses.back()));
      }
      odometry_path_.poses = std::move(compacted_path);
    }

    odometry_path_.poses.push_back(std::move(current_pose));
    odometry_path_pub_->publish(odometry_path_);
  }

  void setRawOccupiedVoxels(
    const std::vector<Eigen::Vector3d> & observations,
    const std::string & source)
  {
    std::unordered_set<int> occupied_addresses;
    occupied_addresses.reserve(observations.size());

    pcl::PointCloud<pcl::PointXYZ> raw;
    raw.reserve(observations.size());
    for (const Eigen::Vector3d & observation : observations) {
      if (!observation.allFinite() || !esdf_map_->isInMap(observation)) {
        continue;
      }

      Eigen::Vector3i id;
      esdf_map_->posToIndex(observation, id);
      const int address = esdf_map_->toAddress(id);
      if (!occupied_addresses.insert(address).second) {
        continue;
      }

      Eigen::Vector3d center;
      esdf_map_->indexToPos(id, center);
      raw.emplace_back(center.x(), center.y(), center.z());
    }

    raw_occupancy_ = std::move(raw);
    raw_occupancy_source_ = source;
  }

  void pointCloudOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    pointcloud_camera_position_ = Eigen::Vector3d(
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z);
    has_pointcloud_odom_ = pointcloud_camera_position_.allFinite();
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    point_cloud.reserve(msg->points.size());
    for (const auto & point : msg->points) {
      point_cloud.emplace_back(point.x, point.y, point.z);
    }

    auto point_cloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(point_cloud, *point_cloud2);
    point_cloud2->header = msg->header;
    pointCloud2Callback(point_cloud2);
  }

  void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!has_pointcloud_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring %s until the first odometry message arrives on %s.",
        cloud_topic_.c_str(), odom_topic_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::vector<Eigen::Vector3d> observations;
    observations.reserve(cloud.size());
    for (const auto & point : cloud.points) {
      const Eigen::Vector3d position(point.x, point.y, point.z);
      if (!position.allFinite()) {
        continue;
      }
      const Eigen::Vector3d delta = position - pointcloud_camera_position_;
      if (
        std::abs(delta.x()) < local_update_range_.x() &&
        std::abs(delta.y()) < local_update_range_.y() &&
        std::abs(delta.z()) < local_update_range_.z())
      {
        observations.push_back(position);
      }
    }
    setRawOccupiedVoxels(observations, cloud_topic_);
  }

  void depthOdomCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr image,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    const Eigen::Vector3d camera_position(
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z);
    Eigen::Quaterniond camera_orientation(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);

    if (!camera_position.allFinite() || !camera_orientation.coeffs().allFinite()) {
      return;
    }
    if (!esdf_map_->isInMap(camera_position)) {
      setRawOccupiedVoxels({}, depth_topic_);
      return;
    }
    if (camera_orientation.norm() < 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring depth frame with an invalid zero-norm odometry quaternion.");
      return;
    }
    camera_orientation.normalize();

    cv_bridge::CvImageConstPtr cv_image;
    try {
      cv_image = cv_bridge::toCvShare(image);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Failed to decode depth image: %s", error.what());
      return;
    }

    const cv::Mat & depth = cv_image->image;
    if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Unsupported depth image type %d; expected 16UC1 or 32FC1.", depth.type());
      return;
    }

    // Match Fast-Planner's filtered depth chain: the first frame initializes
    // temporal state and is not inserted into the occupancy map.
    if (use_depth_filter_ && !has_first_depth_frame_) {
      has_first_depth_frame_ = true;
      setRawOccupiedVoxels({}, depth_topic_);
      return;
    }

    const int margin = use_depth_filter_ ? depth_filter_margin_ : 0;
    const int step = use_depth_filter_ ? skip_pixel_ : 1;
    if (depth.rows <= 2 * margin || depth.cols <= 2 * margin) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Depth image is smaller than the configured filter margins.");
      return;
    }

    std::vector<Eigen::Vector3d> observations;
    observations.reserve(
      static_cast<std::size_t>((depth.rows - 2 * margin + step - 1) / step) *
      static_cast<std::size_t>((depth.cols - 2 * margin + step - 1) / step));

    const Eigen::Matrix3d camera_rotation = camera_orientation.toRotationMatrix();
    for (int v = margin; v < depth.rows - margin; v += step) {
      for (int u = margin; u < depth.cols - margin; u += step) {
        double distance = 0.0;
        if (depth.type() == CV_16UC1) {
          distance = static_cast<double>(depth.at<std::uint16_t>(v, u)) /
            depth_scaling_factor_;
        } else {
          distance = static_cast<double>(depth.at<float>(v, u));
        }

        if (!std::isfinite(distance) || distance <= 0.0) {
          continue;
        }
        if (
          use_depth_filter_ &&
          (distance < depth_filter_mindist_ || distance > depth_filter_maxdist_))
        {
          continue;
        }
        // Endpoints beyond max_ray_length are free ray endpoints in the map,
        // not raw occupied voxels.
        if (max_ray_length_ > 0.0 && distance > max_ray_length_) {
          continue;
        }

        const Eigen::Vector3d camera_point(
          (static_cast<double>(u) - cx_) * distance / fx_,
          (static_cast<double>(v) - cy_) * distance / fy_,
          distance);
        observations.push_back(camera_rotation * camera_point + camera_position);
      }
    }
    setRawOccupiedVoxels(observations, depth_topic_);
  }

  void publishMaps()
  {
    if (!esdf_map_) {
      return;
    }

    Eigen::Vector3d origin;
    Eigen::Vector3d size;
    esdf_map_->getRegion(origin, size);
    const double resolution = esdf_map_->getResolution();
    const int nx = static_cast<int>(std::ceil(size.x() / resolution));
    const int ny = static_cast<int>(std::ceil(size.y() / resolution));
    const int nz = static_cast<int>(std::ceil(size.z() / resolution));

    const std::string frame_id = get_parameter("test.frame_id").as_string();
    const double truncate_height =
      get_parameter("test.visualization_truncate_height").as_double();
    const double ground_height = get_parameter("esdf_map.ground_height").as_double();
    const bool truncate_occupied_marker =
      std::isfinite(truncate_height) && truncate_height >= origin.z();
    const double esdf_max_valid_distance =
      get_parameter("test.esdf_max_valid_distance").as_double();
    const rclcpp::Time stamp = now();

    pcl::PointCloud<pcl::PointXYZ> inflated_occupancy;
    pcl::PointCloud<pcl::PointXYZI> esdf_3d;

    visualization_msgs::msg::Marker occupied_map;
    occupied_map.header.stamp = stamp;
    occupied_map.header.frame_id = frame_id;
    occupied_map.ns = "occupied_map";
    occupied_map.id = 0;
    // Fast-Planner-style voxel obstacle visualization: one CUBE_LIST point per
    // occupied voxel. The cube edge length comes directly from the map
    // resolution, so live observations and preloaded .bin maps use identical
    // geometry without a separate visualization-resolution parameter.
    occupied_map.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupied_map.action = visualization_msgs::msg::Marker::ADD;
    occupied_map.pose.orientation.w = 1.0;
    occupied_map.scale.x = resolution;
    occupied_map.scale.y = resolution;
    occupied_map.scale.z = resolution;
    occupied_map.color.r = 1.0F;
    occupied_map.color.g = 1.0F;
    occupied_map.color.b = 1.0F;
    occupied_map.color.a = 1.0F;

    double min_esdf_distance = std::numeric_limits<double>::infinity();
    double max_esdf_distance = -std::numeric_limits<double>::infinity();
    const double marker_min_z = ground_height;
    const double marker_max_z = truncate_occupied_marker ?
      std::max(marker_min_z + 1e-6, truncate_height) :
      std::max(marker_min_z + 1e-6, origin.z() + size.z());

    for (int x = 0; x < nx; ++x) {
      for (int y = 0; y < ny; ++y) {
        for (int z = 0; z < nz; ++z) {
          const Eigen::Vector3i id(x, y, z);
          Eigen::Vector3d position;
          esdf_map_->indexToPos(id, position);

          if (esdf_map_->getInflateOccupancy(id) == 1) {
            inflated_occupancy.emplace_back(position.x(), position.y(), position.z());

            if (!truncate_occupied_marker || position.z() <= truncate_height) {
              occupied_map.points.push_back(pointMsg(position));
              occupied_map.colors.push_back(
                heightColor(position.z(), marker_min_z, marker_max_z));
            }
          }

          const double distance = esdf_map_->getDistance(id);
          if (
            std::isfinite(distance) && esdf_max_valid_distance > 0.0 &&
            std::abs(distance) < esdf_max_valid_distance)
          {
            pcl::PointXYZI point;
            point.x = static_cast<float>(position.x());
            point.y = static_cast<float>(position.y());
            point.z = static_cast<float>(position.z());
            point.intensity = static_cast<float>(distance);
            esdf_3d.push_back(point);
            min_esdf_distance = std::min(min_esdf_distance, distance);
            max_esdf_distance = std::max(max_esdf_distance, distance);
          }
        }
      }
    }

    // Publish the unmodified signed distance field before reusing the same
    // cloud for RViz normalization. Both messages retain the same voxel centers,
    // timestamp, and frame_id, so a saver can pair them with the occupancy
    // snapshot without another full-map traversal or cloud copy.
    publishCloud(esdf_3d, esdf_distance_pub_, stamp, frame_id);

    // Normalize the ESDF intensity to [0, 1]. XYZ remains the true voxel
    // center; only the color channel is normalized for deterministic RViz
    // rendering. This applies to preloaded esdf.bin and subsequent live maps.
    if (!esdf_3d.empty()) {
      const double distance_range = max_esdf_distance - min_esdf_distance;
      if (distance_range > 1e-9) {
        for (pcl::PointXYZI & point : esdf_3d.points) {
          point.intensity = static_cast<float>(
            std::clamp(
              (static_cast<double>(point.intensity) - min_esdf_distance) / distance_range,
              0.0, 1.0));
        }
      } else {
        for (pcl::PointXYZI & point : esdf_3d.points) {
          point.intensity = 0.5F;
        }
      }
    }

    if (occupied_map.points.empty()) {
      // Reusing the same namespace/id with DELETE clears stale cubes in RViz.
      occupied_map.action = visualization_msgs::msg::Marker::DELETE;
    }

    publishCloud(raw_occupancy_, raw_occupancy_pub_, stamp, frame_id);
    publishCloud(inflated_occupancy, inflated_occupancy_pub_, stamp, frame_id);
    publishCloud(esdf_3d, esdf_pub_, stamp, frame_id);
    occupied_map_pub_->publish(occupied_map);

    if (!reported_initial_map_) {
      reported_initial_map_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Test outputs: raw_live_voxels=%zu (source=%s; preload excluded), "
        "inflated_voxels=%zu, occupied_map_voxels=%zu, esdf_voxels=%zu, "
        "raw_esdf_range=[%.3f, %.3f] m, "
        "esdf_distance_intensity=metres, esdf_visualization_intensity=[0, 1], "
        "marker_voxel_resolution=%.3f m (from ESDFMap), "
        "region_origin=(%.3f, %.3f, %.3f), region_size=(%.3f, %.3f, %.3f)",
        raw_occupancy_.size(),
        raw_occupancy_source_.empty() ? "no live input" : raw_occupancy_source_.c_str(),
        inflated_occupancy.size(), occupied_map.points.size(), esdf_3d.size(),
        esdf_3d.empty() ? 0.0 : min_esdf_distance,
        esdf_3d.empty() ? 0.0 : max_esdf_distance,
        resolution, origin.x(), origin.y(), origin.z(), size.x(), size.y(), size.z());

      if (inflated_occupancy.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "No inflated occupied voxels are available. Check live inputs or the "
          "occupancy.bin path/header/map bounds diagnostics.");
      }
      if (occupied_map.points.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "The occupied_map CUBE_LIST is empty although the publisher is active.");
      }
      if (esdf_3d.empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "No valid ESDF voxels are available. Check live inputs or the "
          "esdf.bin path/header/map bounds diagnostics.");
      }
      if (!enable_depth_odom_ && !enable_pointcloud_odom_) {
        RCLCPP_INFO(
          get_logger(),
          "/map/esdf_map/cloud is intentionally empty in preload-only mode; binary occupancy "
          "is published only on /map/esdf_map/occupancy_inflate and /map/esdf_map/occupied_map.");
      }
    }
  }

  ESDFMap::Ptr esdf_map_;

  bool enable_depth_odom_{false};
  bool enable_pointcloud_odom_{false};
  std::string depth_topic_;
  std::string odom_topic_;
  std::string cloud_topic_;
  Eigen::Vector3d local_update_range_{Eigen::Vector3d::Zero()};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool use_depth_filter_{true};
  double depth_filter_maxdist_{0.0};
  double depth_filter_mindist_{0.0};
  int depth_filter_margin_{0};
  double depth_scaling_factor_{0.0};
  int skip_pixel_{1};
  double max_ray_length_{0.0};

  pcl::PointCloud<pcl::PointXYZ> raw_occupancy_;
  std::string raw_occupancy_source_;
  Eigen::Vector3d pointcloud_camera_position_{Eigen::Vector3d::Zero()};
  bool has_pointcloud_odom_{false};
  bool has_first_depth_frame_{false};

  nav_msgs::msg::Path odometry_path_;
  double odometry_path_min_distance_{0.05};
  std::size_t odometry_path_max_poses_{5000};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr raw_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_cloud_odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> raw_depth_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> raw_depth_odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<DepthOdomSyncPolicy>> raw_depth_odom_sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odometry_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_pose_pub_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  bool reported_initial_map_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESDFMapTestNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

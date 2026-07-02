#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class VioMarkerAdapter : public rclcpp::Node
{
public:
  VioMarkerAdapter()
  : Node("vio_marker_adapter")
  {
    /* VIO keyframe_point (PointCloud)  ->  visualization_msgs/Marker (ESDF vio_points)
     *
     * ESDF vioPointsCallback expects:
     *   - action = Marker::ADD
     *   - ns     = "pts"  (matches esdf_map.vio_points_ns_filter)
     *   - type   = Marker::POINTS
     *   - pose   = identity (ESDF only reads pose to detect coordinate frame)
     *   - points = world-coordinate 3-D positions, one per input point
     *
     * The input PointCloud.header.frame_id tells ESDF which frame the points are in;
     * ESDF vioPointsCallback checks isInMap(camera_pos) using the pose received
     * separately on the vio_pose topic.
     */
    sub_keyframe_point_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/vins_estimator/keyframe_point",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&VioMarkerAdapter::keyframePointCallback, this, std::placeholders::_1));

    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/localization/video_inertial_odom/points",
      rclcpp::QoS(rclcpp::KeepLast(10)));

    /* Forward VIO odometry as PoseWithCovarianceStamped so ESDF vioPoseCallback
     * can derive camera pose.  VINS already publishes Covariance in
     * nav_msgs/Odometry.pose.covariance (26-element fixed-order array).
     * nav_msgs/Odometry.header.frame_id is "world" — match ESDF vio_pose_topic.
     */
    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vins_estimator/odometry",
      rclcpp::QoS(rclcpp::KeepLast(50)),
      std::bind(&VioMarkerAdapter::odomCallback, this, std::placeholders::_1));

    pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/video_inertial_odom/pose",
      rclcpp::QoS(rclcpp::KeepLast(50)));

    RCLCPP_INFO(this->get_logger(),
      "[VioMarkerAdapter] started: keyframe_point->Marker + odometry->PoseWithCovarianceStamped");
  }

private:
  void keyframePointCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns     = "pts";
    marker.id     = 0;
    marker.type   = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // ESDF uses pose only to determine the reference frame; it reads camera_pos
    // from the separate vio_pose topic, so we use identity pose here.
    marker.pose.position.x    = 0.0;
    marker.pose.position.y    = 0.0;
    marker.pose.position.z    = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Input PointCloud stores world 3-D positions in its points[] array.
    // Copy them directly into the Marker — no hand-packed PointCloud2 fields.
    marker.points.reserve(msg->points.size());
    for (const auto & pt : msg->points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      marker.points.push_back(p);
    }

    // Point scale: ESDF only cares about positions, not rendering size.
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    // White points with full alpha.
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    pub_marker_->publish(marker);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
    pose_cov.header = msg->header;
    pose_cov.pose.pose    = msg->pose.pose;
    pose_cov.pose.covariance = msg->pose.covariance;
    pub_pose_cov_->publish(pose_cov);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_keyframe_point_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr                pub_marker_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    sub_odometry_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  pub_pose_cov_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioMarkerAdapter>());
  rclcpp::shutdown();
  return 0;
}

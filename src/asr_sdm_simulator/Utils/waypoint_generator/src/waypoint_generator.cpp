#include <iostream>
#include <cassert>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::msg::Odometry odom;
nav_msgs::msg::Path waypoints;

// series waypoint needed
std::deque<nav_msgs::msg::Path> waypointSegments;
rclcpp::Time trigged_time;

static double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
}

static geometry_msgs::msg::Quaternion yawToQuatMsg(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}

void load_seg(int segid, const rclcpp::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d.") % segid);
    double yaw;
    double time_from_start;
    RCLCPP_INFO(node->get_logger(), "Getting segment %d", segid);
    assert(node->get_parameter(seg_str + "yaw", yaw));
    assert((yaw > -3.1499999) && (yaw < 3.14999999));
    assert(node->get_parameter(seg_str + "time_from_start", time_from_start));
    assert(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    assert(node->get_parameter(seg_str + "x", ptx));
    assert(node->get_parameter(seg_str + "y", pty));
    assert(node->get_parameter(seg_str + "z", ptz));

    assert(ptx.size());
    assert(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

    double baseyaw = quatToYaw(odom.pose.pose.orientation);

    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::msg::PoseStamped pt;
        pt.pose.orientation = yawToQuatMsg(baseyaw + yaw);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}

void load_waypoints(const rclcpp::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    assert(node->get_parameter("segment_cnt", seg_cnt));
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(i, time_base);
        if (i > 0) {
            assert(rclcpp::Time(waypointSegments[i - 1].header.stamp) <
                   rclcpp::Time(waypointSegments[i].header.stamp));
        }
    }
    RCLCPP_INFO(node->get_logger(), "Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = node->now();
    pub1->publish(waypoints);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    // pub2->publish(waypoints);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::msg::Path wp_vis = waypoints;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = node->now();

    {
        geometry_msgs::msg::Pose init_pose;
        init_pose = odom.pose.pose;
        poseArray.poses.push_back(init_pose);
    }

    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::msg::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    pub2->publish(poseArray);
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    is_odom_ready = true;
    odom = *msg;

    if (waypointSegments.size()) {
        rclcpp::Time expected_time = waypointSegments.front().header.stamp;
        if (rclcpp::Time(odom.header.stamp) >= expected_time) {
            waypoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                          pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                          pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                          pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                          pose_stamped.pose.orientation.z << std::endl;
            }
            RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
}

void goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
/*    if (!is_odom_ready) {
        RCLCPP_ERROR(node->get_logger(), "[waypoint_generator] No odom!");
        return;
    }*/

    trigged_time = node->now(); //odom.header.stamp;

    node->get_parameter_or("waypoint_type", waypoint_type, string("manual"));

    if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z > -0.1) {
            // if height > 0, it's a valid goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            RCLCPP_WARN(node->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        if (msg->pose.position.z > 0) {
            // if height > 0, it's a normal goal;
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                double yaw = quatToYaw(odom.pose.pose.orientation);
                pt.pose.orientation = yawToQuatMsg(yaw);
            }
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            // if 0 > height > -1.0, remove last goal;
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // if -1.0 > height, end of input
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    (void)msg;
    if (!is_odom_ready) {
        RCLCPP_ERROR(node->get_logger(), "[waypoint_generator] No odom!");
        return;
    }

    RCLCPP_WARN(node->get_logger(), "[waypoint_generator] Trigger!");
    trigged_time = odom.header.stamp;
    assert(trigged_time > rclcpp::Time(0, 0, trigged_time.get_clock_type()));

    node->get_parameter_or("waypoint_type", waypoint_type, string("manual"));

    RCLCPP_ERROR_STREAM(node->get_logger(), "Pattern " << waypoint_type << " generated!");
    if (waypoint_type == string("free")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(trigged_time);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("waypoint_generator", options);

    trigged_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    node->get_parameter_or("waypoint_type", waypoint_type, string("manual"));

    auto sub1 = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, odom_callback);
    auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 10, goal_callback);
    auto sub3 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "traj_start_trigger", 10, traj_start_trigger_callback);
    pub1 = node->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
    pub2 = node->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

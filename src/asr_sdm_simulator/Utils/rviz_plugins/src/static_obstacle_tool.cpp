/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"

#include "static_obstacle_tool.h"

namespace rviz_plugins
{

StaticObstacleTool::StaticObstacleTool()
{
  shortcut_key_ = 'o';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/simulator/planning_simulator/add_static_obstacle",
    "The topic on which to publish static obstacle placement poses.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

void StaticObstacleTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("Static Obstacle");
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
}

void StaticObstacleTool::updateTopic()
{
  pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 1);
}

void StaticObstacleTool::onPoseSet(double x, double y, double z, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  geometry_msgs::msg::PoseStamped obstacle;
  obstacle.header.stamp = nh_->now();
  obstacle.header.frame_id = fixed_frame;
  obstacle.pose.position.x = x;
  obstacle.pose.position.y = y;
  obstacle.pose.position.z = z;
  obstacle.pose.orientation = tf2::toMsg(quat);

  RCLCPP_INFO(
    nh_->get_logger(),
    "Static obstacle: Frame:%s, Position(%.3f, %.3f, %.3f)",
    fixed_frame.c_str(), obstacle.pose.position.x, obstacle.pose.position.y,
    obstacle.pose.position.z);
  pub_->publish(obstacle);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::StaticObstacleTool, rviz_common::Tool)

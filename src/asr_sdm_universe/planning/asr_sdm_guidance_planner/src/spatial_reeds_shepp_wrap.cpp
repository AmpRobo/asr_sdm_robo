// Copyright (c) Amphibious Robotics.
// Spatial Reeds-Shepp guidance path interface implementation.

#include <asr_sdm_guidance_planner/spatial_reeds_shepp_wrap.h>

#include <shortest_curve_path/spatial_reeds_shepp.hpp>

#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace amprobo
{

void SpatialReedsShepp::setParam(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;
  node_->declare_parameter("spatial_reeds_shepp.yaw_radius", 1.0);
  node_->declare_parameter("spatial_reeds_shepp.pitch_radius", 1.0);
  node_->declare_parameter("spatial_reeds_shepp.sample_ds", 0.1);
  node_->declare_parameter("spatial_reeds_shepp.margin", 0.2);
  node_->declare_parameter("spatial_reeds_shepp.position_tol", 0.05);
  yaw_radius_ = node_->get_parameter("spatial_reeds_shepp.yaw_radius").as_double();
  pitch_radius_ = node_->get_parameter("spatial_reeds_shepp.pitch_radius").as_double();
  sample_ds_ = node_->get_parameter("spatial_reeds_shepp.sample_ds").as_double();
  margin_ = node_->get_parameter("spatial_reeds_shepp.margin").as_double();
  position_tol_ = node_->get_parameter("spatial_reeds_shepp.position_tol").as_double();

  cout << "spatial reeds-shepp yaw radius:" << yaw_radius_ << endl;
  cout << "spatial reeds-shepp pitch radius:" << pitch_radius_ << endl;
}

void SpatialReedsShepp::init()
{
  yaw_radius_ = std::max(yaw_radius_, 1.0e-6);
  pitch_radius_ = std::max(pitch_radius_, 1.0e-6);
  sample_ds_ = std::max(sample_ds_, 1.0e-3);

  if (edt_environment_) {
    edt_environment_->getMapRegion(origin_, map_size_3d_);
    map_max_ = origin_ + map_size_3d_;
    has_map_ = true;
    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;
    cout << "map max: " << map_max_.transpose() << endl;
  }
}

void SpatialReedsShepp::setEnvironment(const EDTEnvironment::Ptr & env)
{
  this->edt_environment_ = env;
}

void SpatialReedsShepp::reset()
{
  path_.clear();
  yaw_path_.clear();
  pitch_path_.clear();
  path_length_ = 0.0;
  position_error_ = 0.0;
  has_path_ = false;
}

int SpatialReedsShepp::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
  double start_yaw = 0.0;
  double start_pitch = 0.0;
  headingFromDelta(end_pt - start_pt, start_yaw, start_pitch);
  return search(start_pt, start_yaw, start_pitch, end_pt, start_yaw, start_pitch);
}

int SpatialReedsShepp::search(
  Eigen::Vector3d start_pt, double start_yaw, double start_pitch, Eigen::Vector3d end_pt,
  double end_yaw, double end_pitch)
{
  reset();

  shortest_curve_path::SpatialReedsShepp planner(yaw_radius_, pitch_radius_);
  shortest_curve_path::Pose3d start;
  shortest_curve_path::Pose3d goal;
  start.position = start_pt;
  start.yaw = start_yaw;
  start.pitch = start_pitch;
  goal.position = end_pt;
  goal.yaw = end_yaw;
  goal.pitch = end_pitch;

  const shortest_curve_path::Path3d curve = planner.plan(start, goal);
  position_error_ = curve.position_error;
  path_length_ = curve.length;

  if (curve.empty() || !curve.feasible(position_tol_)) {
    cout << "spatial reeds-shepp: no feasible curve, err=" << position_error_ << endl;
    return NO_PATH;
  }

  const std::vector<shortest_curve_path::Pose3d> samples =
    planner.discretize(start, curve, sample_ds_);
  path_.reserve(samples.size());
  yaw_path_.reserve(samples.size());
  pitch_path_.reserve(samples.size());

  for (const auto & pose : samples) {
    if (!collisionFree(pose.position)) {
      cout << "spatial reeds-shepp: collision on sampled curve" << endl;
      reset();
      return NO_PATH;
    }
    path_.push_back(pose.position);
    yaw_path_.push_back(pose.yaw);
    pitch_path_.push_back(pose.pitch);
  }

  has_path_ = true;
  return REACH_END;
}

std::vector<Eigen::Vector3d> SpatialReedsShepp::getPath()
{
  return path_;
}

std::vector<double> SpatialReedsShepp::getYawPath()
{
  return yaw_path_;
}

std::vector<double> SpatialReedsShepp::getPitchPath()
{
  return pitch_path_;
}

bool SpatialReedsShepp::headingFromDelta(
  const Eigen::Vector3d & delta, double & yaw, double & pitch) const
{
  const double sxy = std::hypot(delta.x(), delta.y());
  const double range = delta.norm();
  if (range < 1.0e-6) {
    yaw = 0.0;
    pitch = 0.0;
    return false;
  }
  yaw = std::atan2(delta.y(), delta.x());
  pitch = std::atan2(-delta.z(), std::max(sxy, 1.0e-9));
  return true;
}

bool SpatialReedsShepp::inMap(const Eigen::Vector3d & pt) const
{
  if (!has_map_) {
    return true;
  }
  return (pt.array() > origin_.array()).all() && (pt.array() < map_max_.array()).all();
}

bool SpatialReedsShepp::collisionFree(const Eigen::Vector3d & pt) const
{
  if (!inMap(pt)) {
    return false;
  }
  if (!edt_environment_) {
    return true;
  }
  Eigen::Vector3d query = pt;
  return edt_environment_->evaluateCoarseEDT(query, -1.0) > margin_;
}

}  // namespace amprobo

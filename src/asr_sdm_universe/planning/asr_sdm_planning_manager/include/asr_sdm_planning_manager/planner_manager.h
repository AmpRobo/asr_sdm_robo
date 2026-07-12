// Copyright (c) Amphibious Robotics.
// High-level planning manager interface.

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <asr_sdm_esdf_map/edt_environment.hpp>
#include <asr_sdm_planning_manager/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>

#include <asr_sdm_local_path_modifier/astar.h>
#include <asr_sdm_local_path_modifier/topo_prm.h>
#include <asr_sdm_trajectory_optimizer/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

namespace amprobo
{

// Planning Manager
// Key algorithms of mapping and planning are called

class PlanningManager
{
  // SECTION stable
public:
  PlanningManager();
  ~PlanningManager();

  /* main planning interface */
  bool planGlobalTraj(const Eigen::Vector3d & start_pos);
  bool topoReplan(bool collide);

  void planYaw(const Eigen::Vector3d & start_yaw);

  void initPlanModules(const std::shared_ptr<rclcpp::Node> & nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d> & waypoints);

  bool checkTrajCollision(double & distance);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;

private:
  /* ROS node handle (for time and logging) */
  std::shared_ptr<rclcpp::Node> node_;

  /* main planning algorithms & modules */
  ESDFMap::Ptr esdf_map_;

  unique_ptr<Astar> geo_path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

  /* global trajectory helpers */
  vector<Eigen::Vector3d> buildGlobalWaypoints(const Eigen::Vector3d & start_pos);
  PolynomialTraj fitGlobalMinSnapTraj(const vector<Eigen::Vector3d> & points);
  void initLocalTrajFromGlobal(const rclcpp::Time & time_now);

  // topology guided optimization

  void findCollisionRange(
    vector<Eigen::Vector3d> & colli_start, vector<Eigen::Vector3d> & colli_end,
    vector<Eigen::Vector3d> & start_pts, vector<Eigen::Vector3d> & end_pts);

  void optimizeTopoBspline(
    double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double & dt, double & duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double & dt);

  void selectBestTraj(fast_planner::NonUniformBspline & traj);
  void refineTraj(fast_planner::NonUniformBspline & best_traj, double & time_inc);
  void reparamBspline(
    fast_planner::NonUniformBspline & bspline, double ratio, Eigen::MatrixXd & ctrl_pts, double & dt,
    double & time_inc);

  // heading planning
  void calcNextYaw(const double & last_yaw, double & yaw);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<PlanningManager> Ptr;

  // !SECTION
};
}  // namespace amprobo

#endif
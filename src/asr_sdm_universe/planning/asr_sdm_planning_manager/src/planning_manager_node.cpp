// Copyright (c) Amphibious Robotics.
// ROS 2 planning manager node.

#include <asr_sdm_planning_manager/backward.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <asr_sdm_planning_manager/planner_manager.h>
#include <asr_sdm_planning_manager/topo_replan_fsm.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
namespace backward
{
backward::SignalHandling sh;
}

namespace amprobo
{

// SECTION interfaces for setup and query

PlanningManager::PlanningManager()
{
}

PlanningManager::~PlanningManager()
{
  RCLCPP_INFO(rclcpp::get_logger("PlanningManager"), "des manager");
}

void PlanningManager::initPlanModules(const std::shared_ptr<rclcpp::Node> & nh)
{
  node_ = nh;

  /* read algorithm parameters */
  node_->declare_parameter("manager.max_vel", -1.0);
  node_->declare_parameter("manager.max_acc", -1.0);
  node_->declare_parameter("manager.max_jerk", -1.0);
  node_->declare_parameter("manager.dynamic_environment", -1);
  node_->declare_parameter("manager.clearance_threshold", -1.0);
  node_->declare_parameter("manager.local_segment_length", -1.0);
  node_->declare_parameter("manager.control_points_distance", -1.0);
  pp_.max_vel_ = node_->get_parameter("manager.max_vel").as_double();
  pp_.max_acc_ = node_->get_parameter("manager.max_acc").as_double();
  pp_.max_jerk_ = node_->get_parameter("manager.max_jerk").as_double();
  pp_.dynamic_ = node_->get_parameter("manager.dynamic_environment").as_int();
  pp_.clearance_ = node_->get_parameter("manager.clearance_threshold").as_double();
  pp_.local_traj_len_ = node_->get_parameter("manager.local_segment_length").as_double();
  pp_.ctrl_pt_dist = node_->get_parameter("manager.control_points_distance").as_double();

  node_->declare_parameter("manager.use_geometric_path", false);
  node_->declare_parameter("manager.use_topo_path", false);
  node_->declare_parameter("manager.use_optimization", false);
  bool use_geometric_path = node_->get_parameter("manager.use_geometric_path").as_bool();
  bool use_topo_path = node_->get_parameter("manager.use_topo_path").as_bool();
  bool use_optimization = node_->get_parameter("manager.use_optimization").as_bool();

  local_data_.traj_id_ = 0;
  esdf_map_.reset(new ESDFMap);
  esdf_map_->initMap(node_);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(esdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(node_);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(node_);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(node_);
  }
}

void PlanningManager::setGlobalWaypoints(vector<Eigen::Vector3d> & waypoints)
{
  plan_data_.global_waypoints_ = waypoints;
}

bool PlanningManager::checkTrajCollision(double & distance)
{
  double t_now = (node_->now() - local_data_.start_time_).seconds();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION topological replanning

bool PlanningManager::planGlobalTraj(const Eigen::Vector3d & start_pos)
{
  // Clear any previous topological search results before building a new global reference.
  plan_data_.clearTopoPaths();

  // Densify waypoints, fit a min-snap polynomial, then truncate the first local segment.
  vector<Eigen::Vector3d> points = buildGlobalWaypoints(start_pos);
  PolynomialTraj global_traj = fitGlobalMinSnapTraj(points);
  auto time_now = node_->now();
  global_data_.setGlobalTraj(global_traj, time_now);

  initLocalTrajFromGlobal(time_now);
  updateTrajInfo();
  return true;
}

vector<Eigen::Vector3d> PlanningManager::buildGlobalWaypoints(const Eigen::Vector3d & start_pos)
{
  // Start from configured global waypoints and prepend the current start position.
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) RCLCPP_WARN(node_->get_logger(), "no global waypoints!");

  points.insert(points.begin(), start_pos);

  // Insert intermediate points when consecutive waypoints are farther than dist_thresh.
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
          points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  // minSnapTraj needs at least 3 waypoints; insert a midpoint if only two remain.
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  return inter_points;
}

PolynomialTraj PlanningManager::fitGlobalMinSnapTraj(const vector<Eigen::Vector3d> & points)
{
  // Convert waypoints to a position matrix and allocate segment times by distance / max_vel.
  int pt_num = static_cast<int>(points.size());
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  // Slow down the first and last segments for smoother start/stop.
  time(0) *= 2.0;
  time(0) = std::max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = std::max(1.0, time(time.rows() - 1));

  return minSnapTraj(pos, zero, zero, zero, zero, time);
}

void PlanningManager::initLocalTrajFromGlobal(const rclcpp::Time & time_now)
{
  // Truncate a local B-spline segment from the global polynomial for immediate execution.
  double dt, duration;
  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  fast_planner::NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  RCLCPP_INFO(node_->get_logger(), "global trajectory generated.");
}

bool PlanningManager::topoReplan(bool collide)
{
  rclcpp::Time t1, t2;

  /* truncate a new local segment for replanning */
  rclcpp::Time time_now = node_->now();
  double t_now = (time_now - global_data_.global_start_time_).seconds();
  double local_traj_dt, local_traj_duration;
  double time_inc = 0.0;

  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  fast_planner::NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      RCLCPP_WARN(node_->get_logger(), "Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      fast_planner::NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      RCLCPP_INFO(node_->get_logger(), "[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr> graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(
        colli_start.front(), colli_end.back(), start_pts, end_pts, graph, raw_paths, filtered_paths,
        select_paths);

      if (select_paths.size() == 0) {
        RCLCPP_WARN(node_->get_logger(), "No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      RCLCPP_INFO(node_->get_logger(), "[Optimize]: ---------");
      t1 = node_->now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (size_t i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(
          &PlanningManager::optimizeTopoBspline, this, t_now, local_traj_duration, select_paths[i],
          int(i));
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (size_t i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (node_->now() - t1).seconds();
      RCLCPP_INFO_STREAM(node_->get_logger(), "[planner]: optimization time: " << t_opt);
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(
        local_data_.position_traj_, t_now, local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void PlanningManager::selectBestTraj(fast_planner::NonUniformBspline & traj)
{
  // sort by jerk
  vector<fast_planner::NonUniformBspline> & trajs = plan_data_.topo_traj_pos2_;
  sort(
    trajs.begin(), trajs.end(),
    [&](fast_planner::NonUniformBspline & tj1, fast_planner::NonUniformBspline & tj2) {
      return tj1.getJerk() < tj2.getJerk();
    });
  traj = trajs[0];
}

void PlanningManager::refineTraj(fast_planner::NonUniformBspline & best_traj, double & time_inc)
{
  rclcpp::Time t1 = node_->now();
  time_inc = 0.0;
  double dt, t_inc;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  RCLCPP_INFO_STREAM(node_->get_logger(), "ratio: " << ratio);
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = fast_planner::NonUniformBspline(ctrl_pts, 3, dt);
  RCLCPP_WARN_STREAM(
    node_->get_logger(),
    "[Refine]: cost " << (node_->now() - t1).seconds() << " seconds, time change is: " << time_inc);
}

void PlanningManager::updateTrajInfo()
{
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void PlanningManager::reparamBspline(
  fast_planner::NonUniformBspline & bspline, double ratio, Eigen::MatrixXd & ctrl_pts, double & dt,
  double & time_inc)
{
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, plan_data_.local_start_end_derivative_, ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void PlanningManager::optimizeTopoBspline(
  double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id)
{
  rclcpp::Time t1;
  double tm1, tm2, tm3;

  t1 = node_->now();

  // parameterize B-spline according to the length of guide path
  int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (int(guide_pt.size()) != int(ctrl_pts.rows()) - 6) {
    RCLCPP_WARN(node_->get_logger(), "what guide");
  }

  tm1 = (node_->now() - t1).seconds();
  t1 = node_->now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
    ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = fast_planner::NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (node_->now() - t1).seconds();
  t1 = node_->now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
    opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = fast_planner::NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (node_->now() - t1).seconds();
  RCLCPP_INFO(
    node_->get_logger(), "optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd PlanningManager::reparamLocalTraj(double start_t, double & dt, double & duration)
{
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(
    start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set, start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd PlanningManager::reparamLocalTraj(
  double start_t, double duration, int seg_num, double & dt)
{
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  fast_planner::NonUniformBspline::parameterizeToBspline(
    dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void PlanningManager::findCollisionRange(
  vector<Eigen::Vector3d> & colli_start, vector<Eigen::Vector3d> & colli_end,
  vector<Eigen::Vector3d> & start_pts, vector<Eigen::Vector3d> & end_pts)
{
  bool last_safe = true, safe;
  double t_m, t_mp;
  fast_planner::NonUniformBspline * initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e = t_mp;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {
    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int sn = ceil((t_s - t_m) / dt);
  dt = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void PlanningManager::planYaw(const Eigen::Vector3d & start_yaw)
{
  RCLCPP_INFO(node_->get_logger(), "plan yaw");
  auto t1 = node_->now();
  // calculate waypoints of heading

  auto & pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
    1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (size_t i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  RCLCPP_INFO_STREAM(node_->get_logger(), "plan heading: " << (node_->now() - t1).seconds());
}

void PlanningManager::calcNextYaw(const double & last_yaw, double & yaw)
{
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace amprobo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("planning_manager_node");

  amprobo::TopoReplanFSM topo_replan;
  topo_replan.init(nh);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Use a multi-threaded executor so the FSM timers and subscription callbacks
  // can run concurrently across multiple threads.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

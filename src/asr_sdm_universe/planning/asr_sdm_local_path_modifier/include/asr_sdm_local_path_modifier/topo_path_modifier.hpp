// Copyright (c) Amphibious Robotics.
// Topological local path modifier interface.

#ifndef TOPO_PATH_MODIFIER_HPP_
#define TOPO_PATH_MODIFIER_HPP_

#include <Eigen/Core>
#include <map_query_interface.hpp>

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace amprobo
{

struct VirtualObstacle
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.35;
};

struct CollisionRegionHint
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.35;
};

struct CollisionChecker
{
  std::function<bool(const Eigen::Vector3d & point)> pointInCollision;
  std::function<bool(const Eigen::Vector3d & a, const Eigen::Vector3d & b)> segmentInCollision;
  std::function<double(const Eigen::Vector3d & point)> clearance;
  std::vector<CollisionRegionHint> region_hints;
};

struct TopoModifierOptions
{
  double obstacle_radius = 0.35;
  double collision_clearance = 0.10;
  double collision_check_step = 0.05;
  double local_window_padding = 1.20;
  double detour_margin = 0.35;
  double max_connection_length = 8.0;
  double waypoint_spacing = 0.20;
  int max_sample_num = 160;
  int random_seed = 7;
  int shortcut_iterations = 4;
  int block_extend_segments = 2;
  bool deterministic_sampling = true;

  // Parameters corresponding to Fast-Planner TopologyPRM.
  int max_raw_paths = 32;
  int reserve_num = 6;
  double ratio_to_short = 1.45;
  int topo_equiv_sample_num = 48;
  double topo_equiv_triangle_step = 0.25;
  double smoothness_weight = 0.20;
  double clearance_weight = 0.08;
  bool use_guard_connector_graph = true;
  bool allow_dense_graph_fallback = true;
  bool allow_direct_start_goal_edge = false;
  bool use_triangle_topo_check = false;
};

struct TopoModifierResult
{
  bool success = false;
  bool input_in_collision = false;
  bool modified = false;
  std::string message;
  // Backward-compatible best path. In new manager code, prefer candidate_paths.
  std::vector<Eigen::Vector3d> path;
  // Fast-Planner TopologyPRM-style selected topo paths. Each path is a complete path
  // from the original input start to goal, ready for upper-level B-spline / optimizer loops.
  std::vector<std::vector<Eigen::Vector3d>> candidate_paths;
};

class TopoPathModifier
{
public:
  TopoPathModifier() = default;

  void setOptions(const TopoModifierOptions & options);
  const TopoModifierOptions & options() const { return options_; }

  // Core C++ library interface: manager should provide a collision checker or MapQueryInterface.
  TopoModifierResult modify(
    const std::vector<Eigen::Vector3d> & input_path, const CollisionChecker & checker);

  TopoModifierResult modify(
    const std::vector<Eigen::Vector3d> & input_path,
    const MapQueryInterface & map);

  bool pathCollides(
    const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker,
    int * first_segment = nullptr, int * last_segment = nullptr) const;

  bool pathCollides(
    const std::vector<Eigen::Vector3d> & path, const MapQueryInterface & map,
    int * first_segment = nullptr, int * last_segment = nullptr) const;

  // Test compatibility wrapper: RViz virtual obstacles are converted to CollisionChecker.
  TopoModifierResult modify(
    const std::vector<Eigen::Vector3d> & input_path,
    const std::vector<VirtualObstacle> & obstacles);

  bool pathCollides(
    const std::vector<Eigen::Vector3d> & path, const std::vector<VirtualObstacle> & obstacles,
    int * first_segment = nullptr, int * last_segment = nullptr) const;

private:
  struct GraphEdge
  {
    int to = -1;
    double cost = 0.0;
  };

  enum class GraphNodeType { Guard, Connector, Sample };

  struct GraphNode
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    GraphNodeType type = GraphNodeType::Sample;
    int id = -1;
  };

  CollisionChecker makeVirtualObstacleChecker(const std::vector<VirtualObstacle> & obstacles) const;
  CollisionChecker makeMapCollisionChecker(const MapQueryInterface & map) const;

  bool pointInCollision(const Eigen::Vector3d & point, const CollisionChecker & checker) const;
  bool lineVisib(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const CollisionChecker & checker) const;
  bool segmentCollides(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const CollisionChecker & checker) const;
  double pointClearance(const Eigen::Vector3d & point, const CollisionChecker & checker) const;

  bool pointInsideAnyVirtualObstacle(
    const Eigen::Vector3d & point, const std::vector<VirtualObstacle> & obstacles) const;

  bool segmentCollidesWithVirtualObstacles(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b,
    const std::vector<VirtualObstacle> & obstacles) const;

  double pointSegmentDistance(
    const Eigen::Vector3d & point, const Eigen::Vector3d & a, const Eigen::Vector3d & b) const;

  std::vector<Eigen::Vector3d> buildTopoDetour(
    const std::vector<Eigen::Vector3d> & input_path, const CollisionChecker & checker,
    int first_blocked_segment, int last_blocked_segment, int & start_index, int & end_index,
    std::vector<std::vector<Eigen::Vector3d>> & candidate_paths, std::string & message);

  std::vector<std::vector<Eigen::Vector3d>> buildFullCandidatePaths(
    const std::vector<Eigen::Vector3d> & input_path, int start_index, int end_index,
    const std::vector<std::vector<Eigen::Vector3d>> & local_candidate_paths,
    const CollisionChecker & checker) const;

  std::vector<Eigen::Vector3d> buildGuardConnectorGraphPath(
    const std::vector<Eigen::Vector3d> & sample_pool, const CollisionChecker & checker,
    std::vector<std::vector<Eigen::Vector3d>> & selected_paths, std::string & message) const;

  std::vector<Eigen::Vector3d> buildDenseGraphPath(
    const std::vector<Eigen::Vector3d> & nodes, const CollisionChecker & checker,
    std::vector<std::vector<Eigen::Vector3d>> & selected_paths, std::string & message) const;

  std::vector<int> findVisibGuard(
    const Eigen::Vector3d & point, const std::vector<GraphNode> & nodes,
    const CollisionChecker & checker) const;

  bool needConnection(
    int guard_a, int guard_b, const Eigen::Vector3d & connector, std::vector<GraphNode> & nodes,
    std::vector<std::vector<GraphEdge>> & graph, const CollisionChecker & checker) const;

  void addUndirectedEdge(
    int a, int b, const std::vector<GraphNode> & nodes,
    std::vector<std::vector<GraphEdge>> & graph) const;

  bool containsEdge(const std::vector<GraphEdge> & edges, int target) const;

  std::vector<std::vector<Eigen::Vector3d>> searchPaths(
    const std::vector<GraphNode> & nodes, const std::vector<std::vector<GraphEdge>> & graph) const;

  void depthFirstSearch(
    int current, double current_length, double length_limit, const std::vector<GraphNode> & nodes,
    const std::vector<std::vector<GraphEdge>> & graph, std::vector<int> & path_ids,
    std::vector<bool> & visited, std::vector<std::vector<Eigen::Vector3d>> & raw_paths) const;

  double shortestGraphDistance(const std::vector<std::vector<GraphEdge>> & graph) const;

  std::vector<Eigen::Vector3d> shortestGraphPath(
    const std::vector<Eigen::Vector3d> & nodes, const std::vector<std::vector<GraphEdge>> & graph,
    std::vector<std::vector<Eigen::Vector3d>> & candidate_paths) const;

  std::vector<std::vector<Eigen::Vector3d>> shortcutPaths(
    const std::vector<std::vector<Eigen::Vector3d>> & paths,
    const CollisionChecker & checker) const;

  std::vector<std::vector<Eigen::Vector3d>> pruneEquivalent(
    const std::vector<std::vector<Eigen::Vector3d>> & paths,
    const CollisionChecker & checker) const;

  std::vector<std::vector<Eigen::Vector3d>> selectShortPaths(
    const std::vector<std::vector<Eigen::Vector3d>> & paths,
    const CollisionChecker & checker) const;

  bool sameTopoPath(
    const std::vector<Eigen::Vector3d> & path1, const std::vector<Eigen::Vector3d> & path2,
    const CollisionChecker & checker) const;

  bool triangleVisible(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
    const CollisionChecker & checker) const;

  double pathScore(
    const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const;

  double clearancePenalty(
    const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const;

  double minClearanceOnPath(
    const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const;

  double pathLength(const std::vector<Eigen::Vector3d> & path) const;

  std::vector<Eigen::Vector3d> discretizePath(
    const std::vector<Eigen::Vector3d> & path, int point_num) const;

  std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d> & path) const;

  std::vector<Eigen::Vector3d> shortcutPath(
    const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const;

  std::vector<Eigen::Vector3d> densifyPath(const std::vector<Eigen::Vector3d> & path) const;
  void appendUnique(std::vector<Eigen::Vector3d> & path, const Eigen::Vector3d & point) const;

  TopoModifierOptions options_;
};

}  // namespace amprobo

#endif  // TOPO_PATH_MODIFIER_HPP_

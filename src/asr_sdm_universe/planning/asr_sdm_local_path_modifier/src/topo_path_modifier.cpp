// Copyright (c) Amphibious Robotics.
// Topological local path modifier implementation.

#include <Eigen/Geometry>
#include <asr_sdm_local_path_modifier/topo_path_modifier.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace amprobo
{
namespace
{
constexpr double kEpsilon = 1.0e-6;

Eigen::Vector3d safeNormalized(const Eigen::Vector3d & v, const Eigen::Vector3d & fallback)
{
  if (v.norm() < kEpsilon) {
    return fallback.normalized();
  }
  return v.normalized();
}

std::uint64_t edgeKey(const int a, const int b)
{
  const auto lo = static_cast<std::uint32_t>(std::min(a, b));
  const auto hi = static_cast<std::uint32_t>(std::max(a, b));
  return (static_cast<std::uint64_t>(lo) << 32U) | static_cast<std::uint64_t>(hi);
}
}  // namespace

TopoPathModifier::TopoPathModifier()
: rng_(static_cast<std::mt19937::result_type>(options_.random_seed))
{
}

void TopoPathModifier::setOptions(const TopoModifierOptions & options)
{
  options_ = options;
  options_.collision_check_step = std::max(1.0e-3, options_.collision_check_step);
  options_.waypoint_spacing = std::max(1.0e-3, options_.waypoint_spacing);
  options_.max_sample_num = std::max(0, options_.max_sample_num);
  options_.max_raw_paths = std::max(1, options_.max_raw_paths);
  options_.reserve_num = std::max(1, options_.reserve_num);
  options_.ratio_to_short = std::max(1.0, options_.ratio_to_short);
  rng_.seed(static_cast<std::mt19937::result_type>(options_.random_seed));
}

TopoModifierResult TopoPathModifier::modify(
  const std::vector<Eigen::Vector3d> & input_path, const std::vector<VirtualObstacle> & obstacles)
{
  return modify(input_path, makeVirtualObstacleChecker(obstacles));
}

TopoModifierResult TopoPathModifier::modify(
  const std::vector<Eigen::Vector3d> & input_path, const MapQueryInterface & map)
{
  return modify(input_path, makeMapCollisionChecker(map));
}

TopoModifierResult TopoPathModifier::modify(
  const std::vector<Eigen::Vector3d> & input_path, const CollisionChecker & checker)
{
  TopoModifierResult result;

  if (input_path.size() < 2U) {
    result.success = false;
    result.path = input_path;
    result.message = "Input path has fewer than 2 points.";
    return result;
  }

  int first_blocked_segment = -1;
  int last_blocked_segment = -1;
  result.input_in_collision =
    pathCollides(input_path, checker, &first_blocked_segment, &last_blocked_segment);

  if (!result.input_in_collision) {
    result.success = true;
    result.modified = false;
    result.path = densifyPath(input_path);
    result.candidate_paths = {result.path};
    result.message =
      "Input path is collision-free. Emitted the original path as one topo candidate.";
    return result;
  }

  int start_index = 0;
  int end_index = static_cast<int>(input_path.size()) - 1;
  std::string topo_message;
  std::vector<std::vector<Eigen::Vector3d>> local_candidate_paths;
  std::vector<Eigen::Vector3d> local_detour = buildTopoDetour(
    input_path, checker, first_blocked_segment, last_blocked_segment, start_index, end_index,
    local_candidate_paths, topo_message);

  if (local_detour.size() < 2U) {
    result.success = false;
    result.modified = false;
    result.path = input_path;
    result.message =
      "Input path is in collision, but topology-based local modification failed. " + topo_message;
    return result;
  }

  if (local_candidate_paths.empty()) {
    local_candidate_paths.push_back(local_detour);
  }

  std::vector<std::vector<Eigen::Vector3d>> full_candidate_paths =
    buildFullCandidatePaths(input_path, start_index, end_index, local_candidate_paths, checker);

  if (full_candidate_paths.empty()) {
    result.success = false;
    result.modified = false;
    result.path = input_path;
    std::ostringstream oss;
    oss << "TopologyPRM generated local candidates, but no complete candidate path passed "
           "collision check. "
        << topo_message;
    result.message = oss.str();
    return result;
  }

  result.success = true;
  result.modified = true;
  result.candidate_paths = full_candidate_paths;
  result.path = result.candidate_paths.front();
  std::ostringstream oss;
  oss << "Generated " << result.candidate_paths.size()
      << " complete topo candidate paths with Fast-Planner TopologyPRM-style guard/connector graph "
      << "around blocked segments " << first_blocked_segment << "-" << last_blocked_segment << ". "
      << topo_message;
  result.message = oss.str();
  return result;
}

bool TopoPathModifier::pathCollides(
  const std::vector<Eigen::Vector3d> & path, const std::vector<VirtualObstacle> & obstacles,
  int * first_segment, int * last_segment) const
{
  return pathCollides(path, makeVirtualObstacleChecker(obstacles), first_segment, last_segment);
}

bool TopoPathModifier::pathCollides(
  const std::vector<Eigen::Vector3d> & path, const MapQueryInterface & map,
  int * first_segment, int * last_segment) const
{
  return pathCollides(path, makeMapCollisionChecker(map), first_segment, last_segment);
}

bool TopoPathModifier::pathCollides(
  const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker, int * first_segment,
  int * last_segment) const
{
  bool collision = false;
  int first = -1;
  int last = -1;

  if (path.size() < 2U) {
    if (first_segment) {
      *first_segment = -1;
    }
    if (last_segment) {
      *last_segment = -1;
    }
    return false;
  }

  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    if (segmentCollides(path[i], path[i + 1U], checker)) {
      if (!collision) {
        first = static_cast<int>(i);
      }
      last = static_cast<int>(i);
      collision = true;
    }
  }

  if (first_segment) {
    *first_segment = first;
  }
  if (last_segment) {
    *last_segment = last;
  }
  return collision;
}

std::vector<std::vector<Eigen::Vector3d>> TopoPathModifier::buildFullCandidatePaths(
  const std::vector<Eigen::Vector3d> & input_path, const int start_index, const int end_index,
  const std::vector<std::vector<Eigen::Vector3d>> & local_candidate_paths,
  const CollisionChecker & checker) const
{
  std::vector<std::vector<Eigen::Vector3d>> full_candidate_paths;
  if (input_path.size() < 2U || local_candidate_paths.empty()) {
    return full_candidate_paths;
  }

  for (const auto & local_path : local_candidate_paths) {
    if (local_path.size() < 2U) {
      continue;
    }

    std::vector<Eigen::Vector3d> merged;
    merged.reserve(input_path.size() + local_path.size());
    for (int i = 0; i < start_index; ++i) {
      appendUnique(merged, input_path[static_cast<std::size_t>(i)]);
    }
    for (const auto & point : local_path) {
      appendUnique(merged, point);
    }
    for (std::size_t i = static_cast<std::size_t>(end_index + 1); i < input_path.size(); ++i) {
      appendUnique(merged, input_path[i]);
    }

    if (merged.size() < 2U) {
      continue;
    }

    std::vector<Eigen::Vector3d> dense = densifyPath(merged);
    if (pathCollides(dense, checker)) {
      continue;
    }

    bool duplicate = false;
    for (const auto & existing : full_candidate_paths) {
      if (sameTopoPath(dense, existing, checker)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      full_candidate_paths.push_back(std::move(dense));
    }
  }

  std::sort(
    full_candidate_paths.begin(), full_candidate_paths.end(),
    [&](const auto & a, const auto & b) { return pathLength(a) < pathLength(b); });

  return full_candidate_paths;
}

CollisionChecker TopoPathModifier::makeVirtualObstacleChecker(
  const std::vector<VirtualObstacle> & obstacles) const
{
  CollisionChecker checker;
  checker.region_hints.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    checker.region_hints.push_back(CollisionRegionHint{obstacle.center, obstacle.radius});
  }

  checker.pointInCollision = [this, obstacles](const Eigen::Vector3d & point) {
    return pointInsideAnyVirtualObstacle(point, obstacles);
  };
  checker.segmentInCollision = [this, obstacles](
                                 const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    return segmentCollidesWithVirtualObstacles(a, b, obstacles);
  };
  checker.clearance = [this, obstacles](const Eigen::Vector3d & point) {
    if (obstacles.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double min_clearance = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : obstacles) {
      const double inflated_radius = obstacle.radius + options_.collision_clearance;
      min_clearance = std::min(min_clearance, (point - obstacle.center).norm() - inflated_radius);
    }
    return min_clearance;
  };
  return checker;
}

CollisionChecker TopoPathModifier::makeMapCollisionChecker(
  const MapQueryInterface & map) const
{
  CollisionChecker checker;

  checker.pointInCollision = [this, &map](const Eigen::Vector3d & point) {
    return !map.isFree(point, options_.collision_clearance);
  };

  checker.segmentInCollision = [this, &map](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    return !map.segmentIsFree(a, b, options_.collision_check_step, options_.collision_clearance);
  };

  checker.clearance = [&map](const Eigen::Vector3d & point) { return map.distance(point); };

  return checker;
}

bool TopoPathModifier::pointInCollision(
  const Eigen::Vector3d & point, const CollisionChecker & checker) const
{
  if (checker.pointInCollision) {
    return checker.pointInCollision(point);
  }
  const double clearance = pointClearance(point, checker);
  return std::isfinite(clearance) && clearance <= options_.collision_clearance;
}

bool TopoPathModifier::lineVisib(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, const CollisionChecker & checker) const
{
  if (checker.segmentInCollision) {
    return !checker.segmentInCollision(a, b);
  }

  const double length = (b - a).norm();
  const int steps =
    std::max(1, static_cast<int>(std::ceil(length / options_.collision_check_step)));
  for (int i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    if (pointInCollision(a + ratio * (b - a), checker)) {
      return false;
    }
  }
  return true;
}

bool TopoPathModifier::segmentCollides(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, const CollisionChecker & checker) const
{
  return !lineVisib(a, b, checker);
}

double TopoPathModifier::pointClearance(
  const Eigen::Vector3d & point, const CollisionChecker & checker) const
{
  if (checker.clearance) {
    return checker.clearance(point);
  }
  return std::numeric_limits<double>::infinity();
}

bool TopoPathModifier::pointInsideAnyVirtualObstacle(
  const Eigen::Vector3d & point, const std::vector<VirtualObstacle> & obstacles) const
{
  for (const auto & obstacle : obstacles) {
    const double inflated_radius = obstacle.radius + options_.collision_clearance;
    if ((point - obstacle.center).norm() <= inflated_radius) {
      return true;
    }
  }
  return false;
}

bool TopoPathModifier::segmentCollidesWithVirtualObstacles(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  const std::vector<VirtualObstacle> & obstacles) const
{
  for (const auto & obstacle : obstacles) {
    const double inflated_radius = obstacle.radius + options_.collision_clearance;
    if (pointSegmentDistance(obstacle.center, a, b) <= inflated_radius) {
      return true;
    }
  }
  return false;
}

double TopoPathModifier::pointSegmentDistance(
  const Eigen::Vector3d & point, const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
{
  const Eigen::Vector3d ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom < kEpsilon) {
    return (point - a).norm();
  }
  const double t = std::clamp((point - a).dot(ab) / denom, 0.0, 1.0);
  return (point - (a + t * ab)).norm();
}

std::vector<Eigen::Vector3d> TopoPathModifier::buildTopoDetour(
  const std::vector<Eigen::Vector3d> & input_path, const CollisionChecker & checker,
  const int first_blocked_segment, const int last_blocked_segment, int & start_index,
  int & end_index, std::vector<std::vector<Eigen::Vector3d>> & candidate_paths,
  std::string & message)
{
  start_index = std::max(0, first_blocked_segment - options_.block_extend_segments);
  end_index = std::min(
    static_cast<int>(input_path.size()) - 1,
    last_blocked_segment + 1 + options_.block_extend_segments);

  while (start_index > 0 &&
         pointInCollision(input_path[static_cast<std::size_t>(start_index)], checker)) {
    --start_index;
  }
  while (end_index + 1 < static_cast<int>(input_path.size()) &&
         pointInCollision(input_path[static_cast<std::size_t>(end_index)], checker)) {
    ++end_index;
  }

  const Eigen::Vector3d start = input_path[static_cast<std::size_t>(start_index)];
  const Eigen::Vector3d goal = input_path[static_cast<std::size_t>(end_index)];

  if (pointInCollision(start, checker) || pointInCollision(goal, checker)) {
    message = "The selected local replanning endpoints are inside collision.";
    return {};
  }

  std::vector<Eigen::Vector3d> sample_pool;
  sample_pool.reserve(static_cast<std::size_t>(options_.max_sample_num) + input_path.size() + 64U);
  sample_pool.push_back(start);
  sample_pool.push_back(goal);

  Eigen::Vector3d dir = safeNormalized(goal - start, Eigen::Vector3d::UnitX());
  Eigen::Vector3d axis1 = dir.cross(Eigen::Vector3d::UnitZ());
  if (axis1.norm() < 1.0e-3) {
    axis1 = dir.cross(Eigen::Vector3d::UnitY());
  }
  axis1.normalize();
  Eigen::Vector3d axis2 = safeNormalized(dir.cross(axis1), Eigen::Vector3d::UnitZ());

  std::vector<CollisionRegionHint> local_hints;
  for (const auto & hint : checker.region_hints) {
    const double influence =
      hint.radius + options_.collision_clearance + options_.local_window_padding;
    if (pointSegmentDistance(hint.center, start, goal) <= influence) {
      local_hints.push_back(hint);
    }
  }

  const int hint_begin = std::max(0, first_blocked_segment);
  const int hint_end = std::min(static_cast<int>(input_path.size()) - 2, last_blocked_segment);
  for (int seg = hint_begin; seg <= hint_end; ++seg) {
    const Eigen::Vector3d a = input_path[static_cast<std::size_t>(seg)];
    const Eigen::Vector3d b = input_path[static_cast<std::size_t>(seg + 1)];
    const double length = (b - a).norm();
    const int steps =
      std::max(1, static_cast<int>(std::ceil(length / options_.collision_check_step)));
    for (int i = 0; i <= steps; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      const Eigen::Vector3d p = a + ratio * (b - a);
      if (pointInCollision(p, checker)) {
        local_hints.push_back(
          CollisionRegionHint{p, std::max(options_.detour_margin, options_.collision_clearance)});
        break;
      }
    }
  }

  if (local_hints.empty()) {
    const Eigen::Vector3d mid =
      0.5 * (input_path[static_cast<std::size_t>(first_blocked_segment)] +
             input_path[static_cast<std::size_t>(last_blocked_segment + 1)]);
    local_hints.push_back(CollisionRegionHint{mid, options_.local_window_padding});
  }

  const auto addNode = [&](const Eigen::Vector3d & p) {
    if (pointInCollision(p, checker)) {
      return;
    }
    for (const auto & n : sample_pool) {
      if ((n - p).norm() < 0.5 * std::max(options_.waypoint_spacing, 1.0e-3)) {
        return;
      }
    }
    sample_pool.push_back(p);
  };

  for (int i = start_index + 1; i < end_index; ++i) {
    addNode(input_path[static_cast<std::size_t>(i)]);
  }

  const std::vector<Eigen::Vector3d> detour_dirs = {
    axis1,
    -axis1,
    axis2,
    -axis2,
    safeNormalized(axis1 + axis2, axis1),
    safeNormalized(axis1 - axis2, axis1),
    safeNormalized(-axis1 + axis2, axis2),
    safeNormalized(-axis1 - axis2, -axis1)};

  for (const auto & hint : local_hints) {
    const double detour_radius =
      hint.radius + options_.collision_clearance + options_.detour_margin;
    for (const auto & detour_dir : detour_dirs) {
      addNode(hint.center + detour_radius * detour_dir);
    }
  }

  Eigen::Vector3d lower = start.cwiseMin(goal);
  Eigen::Vector3d upper = start.cwiseMax(goal);
  for (int i = start_index; i <= end_index; ++i) {
    const Eigen::Vector3d p = input_path[static_cast<std::size_t>(i)];
    lower = lower.cwiseMin(p);
    upper = upper.cwiseMax(p);
  }
  for (const auto & hint : local_hints) {
    const double pad = hint.radius + options_.collision_clearance + options_.local_window_padding;
    lower = lower.cwiseMin(hint.center - Eigen::Vector3d::Constant(pad));
    upper = upper.cwiseMax(hint.center + Eigen::Vector3d::Constant(pad));
  }
  lower -= Eigen::Vector3d::Constant(options_.local_window_padding);
  upper += Eigen::Vector3d::Constant(options_.local_window_padding);

  std::uniform_real_distribution<double> x_dist(lower.x(), upper.x());
  std::uniform_real_distribution<double> y_dist(lower.y(), upper.y());
  std::uniform_real_distribution<double> z_dist(lower.z(), upper.z());

  if (options_.deterministic_sampling) {
    rng_.seed(static_cast<std::mt19937::result_type>(options_.random_seed));
  }

  for (int i = 0; i < options_.max_sample_num; ++i) {
    addNode(Eigen::Vector3d(x_dist(rng_), y_dist(rng_), z_dist(rng_)));
  }

  std::string graph_message;
  std::vector<Eigen::Vector3d> best_path;
  if (options_.use_guard_connector_graph) {
    best_path = buildGuardConnectorGraphPath(sample_pool, checker, candidate_paths, graph_message);
  }

  if (best_path.size() < 2U && options_.allow_dense_graph_fallback) {
    std::vector<std::vector<Eigen::Vector3d>> dense_candidates;
    std::string dense_message;
    best_path = buildDenseGraphPath(sample_pool, checker, dense_candidates, dense_message);
    if (!dense_candidates.empty()) {
      candidate_paths = dense_candidates;
    }
    graph_message += " | dense_fallback: " + dense_message;
  }

  if (best_path.size() < 2U) {
    std::ostringstream oss;
    oss << "No visible topology graph connection found. samples=" << sample_pool.size()
        << ", local_hints=" << local_hints.size() << ". " << graph_message;
    message = oss.str();
    return {};
  }

  best_path = densifyPath(best_path);

  std::ostringstream oss;
  oss << "samples=" << sample_pool.size() << ", local_hints=" << local_hints.size()
      << ", selected_topo_paths=" << candidate_paths.size() << ", start_index=" << start_index
      << ", end_index=" << end_index << ". " << graph_message;
  message = oss.str();
  return best_path;
}

std::vector<Eigen::Vector3d> TopoPathModifier::buildGuardConnectorGraphPath(
  const std::vector<Eigen::Vector3d> & sample_pool, const CollisionChecker & checker,
  std::vector<std::vector<Eigen::Vector3d>> & selected_paths, std::string & message) const
{
  selected_paths.clear();
  if (sample_pool.size() < 2U) {
    message = "guard_connector_graph: fewer than 2 samples.";
    return {};
  }

  std::vector<GraphNode> graph_nodes;
  graph_nodes.reserve(sample_pool.size());
  graph_nodes.push_back(GraphNode{sample_pool[0], GraphNodeType::Guard, 0});
  graph_nodes.push_back(GraphNode{sample_pool[1], GraphNodeType::Guard, 1});
  std::vector<std::vector<GraphEdge>> graph(2U);

  for (std::size_t sample_id = 2U; sample_id < sample_pool.size(); ++sample_id) {
    const Eigen::Vector3d & sample = sample_pool[sample_id];
    if (pointInCollision(sample, checker)) {
      continue;
    }

    const std::vector<int> visible_guards = findVisibGuard(sample, graph_nodes, checker);
    if (visible_guards.empty()) {
      const int id = static_cast<int>(graph_nodes.size());
      graph_nodes.push_back(GraphNode{sample, GraphNodeType::Guard, id});
      graph.emplace_back();
      continue;
    }

    if (visible_guards.size() == 2U) {
      const int guard_a = visible_guards[0];
      const int guard_b = visible_guards[1];
      if (!needConnection(guard_a, guard_b, sample, graph_nodes, graph, checker)) {
        continue;
      }
      const int connector_id = static_cast<int>(graph_nodes.size());
      graph_nodes.push_back(GraphNode{sample, GraphNodeType::Connector, connector_id});
      graph.emplace_back();
      addUndirectedEdge(connector_id, guard_a, graph_nodes, graph);
      addUndirectedEdge(connector_id, guard_b, graph_nodes, graph);
    }
  }

  if (
    options_.allow_direct_start_goal_edge && !containsEdge(graph[0], 1) &&
    lineVisib(graph_nodes[0].pos, graph_nodes[1].pos, checker)) {
    addUndirectedEdge(0, 1, graph_nodes, graph);
  }

  std::vector<std::vector<Eigen::Vector3d>> raw_paths = searchPaths(graph_nodes, graph);
  if (raw_paths.empty()) {
    std::ostringstream oss;
    oss << "guard_connector_graph: nodes=" << graph_nodes.size() << ", raw_paths=0.";
    message = oss.str();
    return {};
  }

  std::vector<std::vector<Eigen::Vector3d>> short_paths = shortcutPaths(raw_paths, checker);
  std::vector<std::vector<Eigen::Vector3d>> filtered_paths = pruneEquivalent(short_paths, checker);
  selected_paths = selectShortPaths(filtered_paths, checker);
  if (selected_paths.empty()) {
    selected_paths = filtered_paths;
  }
  if (selected_paths.empty()) {
    selected_paths = short_paths;
  }

  auto best_it = std::min_element(
    selected_paths.begin(), selected_paths.end(),
    [&](const auto & a, const auto & b) { return pathLength(a) < pathLength(b); });

  std::ostringstream oss;
  oss << "guard_connector_graph: nodes=" << graph_nodes.size() << ", raw_paths=" << raw_paths.size()
      << ", topo_clusters=" << filtered_paths.size() << ", selected=" << selected_paths.size();
  if (best_it != selected_paths.end()) {
    oss << ", best_length=" << pathLength(*best_it)
        << ", best_score=" << pathScore(*best_it, checker);
  }
  message = oss.str();

  return best_it == selected_paths.end() ? std::vector<Eigen::Vector3d>{} : *best_it;
}

std::vector<Eigen::Vector3d> TopoPathModifier::buildDenseGraphPath(
  const std::vector<Eigen::Vector3d> & dense_nodes, const CollisionChecker & checker,
  std::vector<std::vector<Eigen::Vector3d>> & selected_paths, std::string & message) const
{
  selected_paths.clear();
  if (dense_nodes.size() < 2U) {
    message = "dense_graph: fewer than 2 samples.";
    return {};
  }

  std::vector<GraphNode> graph_nodes;
  graph_nodes.reserve(dense_nodes.size());
  for (std::size_t i = 0; i < dense_nodes.size(); ++i) {
    graph_nodes.push_back(GraphNode{dense_nodes[i], GraphNodeType::Sample, static_cast<int>(i)});
  }
  graph_nodes[0].type = GraphNodeType::Guard;
  graph_nodes[1].type = GraphNodeType::Guard;

  std::vector<std::vector<GraphEdge>> graph(graph_nodes.size());
  std::unordered_set<std::uint64_t> blocked_edges;

  const auto addEdgeIfVisible = [&](const int i, const int j) {
    const double distance =
      (graph_nodes[static_cast<std::size_t>(i)].pos - graph_nodes[static_cast<std::size_t>(j)].pos)
        .norm();
    if (distance > options_.max_connection_length) {
      return;
    }
    if (!lineVisib(
          graph_nodes[static_cast<std::size_t>(i)].pos,
          graph_nodes[static_cast<std::size_t>(j)].pos, checker)) {
      blocked_edges.insert(edgeKey(i, j));
      return;
    }
    graph[static_cast<std::size_t>(i)].push_back(GraphEdge{j, distance});
    graph[static_cast<std::size_t>(j)].push_back(GraphEdge{i, distance});
  };

  for (std::size_t i = 0; i < graph_nodes.size(); ++i) {
    if (pointInCollision(graph_nodes[i].pos, checker)) {
      continue;
    }
    for (std::size_t j = i + 1U; j < graph_nodes.size(); ++j) {
      if (pointInCollision(graph_nodes[j].pos, checker)) {
        continue;
      }
      addEdgeIfVisible(static_cast<int>(i), static_cast<int>(j));
    }
  }

  std::vector<std::vector<Eigen::Vector3d>> raw_paths = searchPaths(graph_nodes, graph);
  if (raw_paths.empty()) {
    std::vector<Eigen::Vector3d> shortest = shortestGraphPath(dense_nodes, graph, selected_paths);
    std::ostringstream oss;
    oss << "dense_graph: nodes=" << graph_nodes.size() << ", blocked_edges=" << blocked_edges.size()
        << ", dijkstra_path=" << (shortest.size() >= 2U ? 1 : 0);
    message = oss.str();
    return shortcutPath(shortest, checker);
  }

  std::vector<std::vector<Eigen::Vector3d>> short_paths = shortcutPaths(raw_paths, checker);
  std::vector<std::vector<Eigen::Vector3d>> filtered_paths = pruneEquivalent(short_paths, checker);
  selected_paths = selectShortPaths(filtered_paths, checker);
  if (selected_paths.empty()) {
    selected_paths = filtered_paths;
  }
  auto best_it = std::min_element(
    selected_paths.begin(), selected_paths.end(),
    [&](const auto & a, const auto & b) { return pathLength(a) < pathLength(b); });

  std::ostringstream oss;
  oss << "dense_graph: nodes=" << graph_nodes.size() << ", blocked_edges=" << blocked_edges.size()
      << ", raw_paths=" << raw_paths.size() << ", topo_clusters=" << filtered_paths.size()
      << ", selected=" << selected_paths.size();
  message = oss.str();

  return best_it == selected_paths.end() ? std::vector<Eigen::Vector3d>{}
                                         : shortcutPath(*best_it, checker);
}

std::vector<int> TopoPathModifier::findVisibGuard(
  const Eigen::Vector3d & point, const std::vector<GraphNode> & graph_nodes,
  const CollisionChecker & checker) const
{
  std::vector<int> visible_guards;
  int visib_num = 0;
  for (const auto & node : graph_nodes) {
    if (node.type != GraphNodeType::Guard) {
      continue;
    }
    if ((node.pos - point).norm() > options_.max_connection_length) {
      continue;
    }
    if (lineVisib(point, node.pos, checker)) {
      visible_guards.push_back(node.id);
      ++visib_num;
      if (visib_num > 2) {
        break;
      }
    }
  }
  return visible_guards;
}

bool TopoPathModifier::needConnection(
  const int guard_a, const int guard_b, const Eigen::Vector3d & connector,
  std::vector<GraphNode> & graph_nodes, std::vector<std::vector<GraphEdge>> & graph,
  const CollisionChecker & checker) const
{
  if (guard_a == guard_b) {
    return false;
  }
  if (
    !lineVisib(graph_nodes[static_cast<std::size_t>(guard_a)].pos, connector, checker) ||
    !lineVisib(graph_nodes[static_cast<std::size_t>(guard_b)].pos, connector, checker)) {
    return false;
  }

  const std::vector<Eigen::Vector3d> new_path = {
    graph_nodes[static_cast<std::size_t>(guard_a)].pos, connector,
    graph_nodes[static_cast<std::size_t>(guard_b)].pos};
  const double new_length = pathLength(new_path);

  const auto refreshConnectorCosts = [&](const int connector_id) {
    for (auto & edge : graph[static_cast<std::size_t>(connector_id)]) {
      edge.cost = (graph_nodes[static_cast<std::size_t>(connector_id)].pos -
                   graph_nodes[static_cast<std::size_t>(edge.to)].pos)
                    .norm();
      for (auto & back_edge : graph[static_cast<std::size_t>(edge.to)]) {
        if (back_edge.to == connector_id) {
          back_edge.cost = edge.cost;
          break;
        }
      }
    }
  };

  for (const auto & edge_a : graph[static_cast<std::size_t>(guard_a)]) {
    const int old_connector = edge_a.to;
    if (old_connector < 0 || static_cast<std::size_t>(old_connector) >= graph_nodes.size()) {
      continue;
    }
    if (graph_nodes[static_cast<std::size_t>(old_connector)].type != GraphNodeType::Connector) {
      continue;
    }
    if (!containsEdge(graph[static_cast<std::size_t>(old_connector)], guard_b)) {
      continue;
    }

    const std::vector<Eigen::Vector3d> old_path = {
      graph_nodes[static_cast<std::size_t>(guard_a)].pos,
      graph_nodes[static_cast<std::size_t>(old_connector)].pos,
      graph_nodes[static_cast<std::size_t>(guard_b)].pos};
    if (!sameTopoPath(new_path, old_path, checker)) {
      continue;
    }

    if (new_length + 1.0e-6 < pathLength(old_path)) {
      graph_nodes[static_cast<std::size_t>(old_connector)].pos = connector;
      refreshConnectorCosts(old_connector);
    }
    return false;
  }

  return true;
}

bool TopoPathModifier::containsEdge(const std::vector<GraphEdge> & edges, const int target) const
{
  return std::any_of(
    edges.begin(), edges.end(), [target](const GraphEdge & edge) { return edge.to == target; });
}

void TopoPathModifier::addUndirectedEdge(
  const int a, const int b, const std::vector<GraphNode> & graph_nodes,
  std::vector<std::vector<GraphEdge>> & graph) const
{
  if (
    a == b || a < 0 || b < 0 || static_cast<std::size_t>(a) >= graph_nodes.size() ||
    static_cast<std::size_t>(b) >= graph_nodes.size()) {
    return;
  }
  if (containsEdge(graph[static_cast<std::size_t>(a)], b)) {
    return;
  }
  const double distance =
    (graph_nodes[static_cast<std::size_t>(a)].pos - graph_nodes[static_cast<std::size_t>(b)].pos)
      .norm();
  graph[static_cast<std::size_t>(a)].push_back(GraphEdge{b, distance});
  graph[static_cast<std::size_t>(b)].push_back(GraphEdge{a, distance});
}

std::vector<std::vector<Eigen::Vector3d>> TopoPathModifier::searchPaths(
  const std::vector<GraphNode> & graph_nodes,
  const std::vector<std::vector<GraphEdge>> & graph) const
{
  std::vector<std::vector<Eigen::Vector3d>> raw_paths;
  if (graph_nodes.size() < 2U || graph.size() != graph_nodes.size()) {
    return raw_paths;
  }

  const double shortest = shortestGraphDistance(graph);
  if (!std::isfinite(shortest)) {
    return raw_paths;
  }
  const double length_limit =
    shortest * std::max(1.05, options_.ratio_to_short * 2.0) + options_.max_connection_length;

  std::vector<int> path_ids;
  std::vector<bool> visited(graph_nodes.size(), false);
  path_ids.push_back(0);
  visited[0] = true;
  depthFirstSearch(0, 0.0, length_limit, graph_nodes, graph, path_ids, visited, raw_paths);

  std::sort(raw_paths.begin(), raw_paths.end(), [&](const auto & a, const auto & b) {
    if (a.size() != b.size()) {
      return a.size() < b.size();
    }
    return pathLength(a) < pathLength(b);
  });
  if (raw_paths.size() > static_cast<std::size_t>(std::max(1, options_.max_raw_paths))) {
    raw_paths.resize(static_cast<std::size_t>(options_.max_raw_paths));
  }
  return raw_paths;
}

void TopoPathModifier::depthFirstSearch(
  const int current, const double current_length, const double length_limit,
  const std::vector<GraphNode> & graph_nodes, const std::vector<std::vector<GraphEdge>> & graph,
  std::vector<int> & path_ids, std::vector<bool> & visited,
  std::vector<std::vector<Eigen::Vector3d>> & raw_paths) const
{
  if (raw_paths.size() >= static_cast<std::size_t>(std::max(1, options_.max_raw_paths) * 4)) {
    return;
  }
  if (current == 1) {
    std::vector<Eigen::Vector3d> path;
    path.reserve(path_ids.size());
    for (const int id : path_ids) {
      path.push_back(graph_nodes[static_cast<std::size_t>(id)].pos);
    }
    raw_paths.push_back(path);
    return;
  }

  std::vector<GraphEdge> neighbors = graph[static_cast<std::size_t>(current)];
  std::sort(neighbors.begin(), neighbors.end(), [&](const GraphEdge & a, const GraphEdge & b) {
    const double ha = (graph_nodes[static_cast<std::size_t>(a.to)].pos - graph_nodes[1].pos).norm();
    const double hb = (graph_nodes[static_cast<std::size_t>(b.to)].pos - graph_nodes[1].pos).norm();
    return current_length + a.cost + ha < current_length + b.cost + hb;
  });

  for (const auto & edge : neighbors) {
    if (edge.to < 0 || static_cast<std::size_t>(edge.to) >= graph_nodes.size()) {
      continue;
    }
    if (visited[static_cast<std::size_t>(edge.to)]) {
      continue;
    }
    const double next_length = current_length + edge.cost;
    const double optimistic =
      next_length +
      (graph_nodes[static_cast<std::size_t>(edge.to)].pos - graph_nodes[1].pos).norm();
    if (optimistic > length_limit) {
      continue;
    }
    visited[static_cast<std::size_t>(edge.to)] = true;
    path_ids.push_back(edge.to);
    depthFirstSearch(
      edge.to, next_length, length_limit, graph_nodes, graph, path_ids, visited, raw_paths);
    path_ids.pop_back();
    visited[static_cast<std::size_t>(edge.to)] = false;
  }
}

double TopoPathModifier::shortestGraphDistance(
  const std::vector<std::vector<GraphEdge>> & graph) const
{
  if (graph.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::vector<double> dist(graph.size(), std::numeric_limits<double>::infinity());
  dist[0] = 0.0;
  open.push({0.0, 0});

  while (!open.empty()) {
    const auto [cost, u] = open.top();
    open.pop();
    if (cost > dist[static_cast<std::size_t>(u)] + 1.0e-6) {
      continue;
    }
    if (u == 1) {
      return cost;
    }
    for (const auto & edge : graph[static_cast<std::size_t>(u)]) {
      const double next_cost = cost + edge.cost;
      if (next_cost + 1.0e-6 < dist[static_cast<std::size_t>(edge.to)]) {
        dist[static_cast<std::size_t>(edge.to)] = next_cost;
        open.push({next_cost, edge.to});
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

std::vector<Eigen::Vector3d> TopoPathModifier::shortestGraphPath(
  const std::vector<Eigen::Vector3d> & nodes, const std::vector<std::vector<GraphEdge>> & graph,
  std::vector<std::vector<Eigen::Vector3d>> & candidate_paths) const
{
  if (nodes.size() < 2U || graph.size() != nodes.size()) {
    return {};
  }

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::vector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(nodes.size(), -1);
  dist[0] = 0.0;
  open.push({0.0, 0});

  while (!open.empty()) {
    const auto [cost, u] = open.top();
    open.pop();
    if (cost > dist[static_cast<std::size_t>(u)] + 1.0e-6) {
      continue;
    }
    if (u == 1) {
      break;
    }
    for (const auto & edge : graph[static_cast<std::size_t>(u)]) {
      const double next_cost = cost + edge.cost;
      if (next_cost + 1.0e-6 < dist[static_cast<std::size_t>(edge.to)]) {
        dist[static_cast<std::size_t>(edge.to)] = next_cost;
        parent[static_cast<std::size_t>(edge.to)] = u;
        open.push({next_cost, edge.to});
      }
    }
  }

  if (!std::isfinite(dist[1])) {
    return {};
  }

  std::vector<int> ids;
  for (int v = 1; v >= 0; v = parent[static_cast<std::size_t>(v)]) {
    ids.push_back(v);
    if (v == 0) {
      break;
    }
  }
  if (ids.empty() || ids.back() != 0) {
    return {};
  }
  std::reverse(ids.begin(), ids.end());

  std::vector<Eigen::Vector3d> path;
  path.reserve(ids.size());
  for (const int id : ids) {
    path.push_back(nodes[static_cast<std::size_t>(id)]);
  }
  candidate_paths.push_back(path);
  return path;
}

std::vector<std::vector<Eigen::Vector3d>> TopoPathModifier::shortcutPaths(
  const std::vector<std::vector<Eigen::Vector3d>> & paths, const CollisionChecker & checker) const
{
  std::vector<std::vector<Eigen::Vector3d>> short_paths;
  short_paths.reserve(paths.size());
  for (const auto & path : paths) {
    const auto shortened = shortcutPath(path, checker);
    if (shortened.size() >= 2U && !pathCollides(shortened, checker)) {
      short_paths.push_back(shortened);
    }
  }
  return short_paths;
}

std::vector<std::vector<Eigen::Vector3d>> TopoPathModifier::pruneEquivalent(
  const std::vector<std::vector<Eigen::Vector3d>> & paths, const CollisionChecker & checker) const
{
  std::vector<std::vector<Eigen::Vector3d>> pruned_paths;
  if (paths.empty()) {
    return pruned_paths;
  }

  std::vector<int> exist_path_ids;
  exist_path_ids.push_back(0);
  for (std::size_t i = 1U; i < paths.size(); ++i) {
    bool new_path = true;
    for (const int id : exist_path_ids) {
      if (sameTopoPath(paths[i], paths[static_cast<std::size_t>(id)], checker)) {
        new_path = false;
        break;
      }
    }
    if (new_path) {
      exist_path_ids.push_back(static_cast<int>(i));
    }
  }

  pruned_paths.reserve(exist_path_ids.size());
  for (const int id : exist_path_ids) {
    pruned_paths.push_back(paths[static_cast<std::size_t>(id)]);
  }
  return pruned_paths;
}

std::vector<std::vector<Eigen::Vector3d>> TopoPathModifier::selectShortPaths(
  const std::vector<std::vector<Eigen::Vector3d>> & paths, const CollisionChecker & checker) const
{
  (void)checker;
  std::vector<std::vector<Eigen::Vector3d>> short_paths;
  if (paths.empty()) {
    return short_paths;
  }

  std::vector<std::vector<Eigen::Vector3d>> candidates = paths;
  double min_len = std::numeric_limits<double>::infinity();
  for (int i = 0; i < options_.reserve_num && !candidates.empty(); ++i) {
    const auto shortest_it = std::min_element(
      candidates.begin(), candidates.end(),
      [&](const auto & a, const auto & b) { return pathLength(a) < pathLength(b); });
    if (shortest_it == candidates.end()) {
      break;
    }
    const double length = pathLength(*shortest_it);
    if (i == 0) {
      min_len = length;
      short_paths.push_back(*shortest_it);
      candidates.erase(shortest_it);
      continue;
    }
    if (length / std::max(min_len, kEpsilon) < options_.ratio_to_short) {
      short_paths.push_back(*shortest_it);
      candidates.erase(shortest_it);
    } else {
      break;
    }
  }
  return short_paths;
}

bool TopoPathModifier::sameTopoPath(
  const std::vector<Eigen::Vector3d> & path1, const std::vector<Eigen::Vector3d> & path2,
  const CollisionChecker & checker) const
{
  if (path1.size() < 2U || path2.size() < 2U) {
    return false;
  }

  const double len1 = pathLength(path1);
  const double len2 = pathLength(path2);
  const double max_len = std::max(len1, len2);
  const int point_num = std::max(
    4, std::max(
         options_.topo_equiv_sample_num,
         static_cast<int>(std::ceil(max_len / options_.collision_check_step))));
  const std::vector<Eigen::Vector3d> p1 = discretizePath(path1, point_num);
  const std::vector<Eigen::Vector3d> p2 = discretizePath(path2, point_num);
  if (p1.size() != p2.size() || p1.size() < 2U) {
    return false;
  }

  for (std::size_t i = 0; i < p1.size(); ++i) {
    if (!lineVisib(p1[i], p2[i], checker)) {
      return false;
    }
  }

  if (!options_.use_triangle_topo_check) {
    return true;
  }

  for (std::size_t i = 0; i + 1U < p1.size(); ++i) {
    if (
      !triangleVisible(p1[i], p1[i + 1U], p2[i], checker) ||
      !triangleVisible(p1[i + 1U], p2[i + 1U], p2[i], checker)) {
      return false;
    }
  }
  return true;
}

bool TopoPathModifier::triangleVisible(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
  const CollisionChecker & checker) const
{
  if (!lineVisib(a, b, checker) || !lineVisib(b, c, checker) || !lineVisib(c, a, checker)) {
    return false;
  }

  const double max_edge = std::max({(a - b).norm(), (b - c).norm(), (c - a).norm()});
  const int steps = std::max(
    2, static_cast<int>(std::ceil(max_edge / std::max(0.05, options_.topo_equiv_triangle_step))));
  for (int i = 0; i <= steps; ++i) {
    for (int j = 0; j <= steps - i; ++j) {
      const double u = static_cast<double>(i) / static_cast<double>(steps);
      const double v = static_cast<double>(j) / static_cast<double>(steps);
      const double w = 1.0 - u - v;
      const Eigen::Vector3d p = u * a + v * b + w * c;
      if (pointInCollision(p, checker)) {
        return false;
      }
    }
  }
  return true;
}

double TopoPathModifier::pathScore(
  const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const
{
  if (path.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  double smoothness = 0.0;
  for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
    const Eigen::Vector3d v1 = path[i] - path[i - 1U];
    const Eigen::Vector3d v2 = path[i + 1U] - path[i];
    if (v1.norm() < kEpsilon || v2.norm() < kEpsilon) {
      continue;
    }
    const double cos_angle = std::clamp(v1.normalized().dot(v2.normalized()), -1.0, 1.0);
    const double angle = std::acos(cos_angle);
    smoothness += angle * angle;
  }

  return pathLength(path) + options_.smoothness_weight * smoothness +
         options_.clearance_weight * clearancePenalty(path, checker);
}

double TopoPathModifier::clearancePenalty(
  const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const
{
  if (path.empty()) {
    return 0.0;
  }

  const std::vector<Eigen::Vector3d> sampled =
    discretizePath(path, std::max(8, options_.topo_equiv_sample_num / 2));
  double penalty = 0.0;
  for (const auto & point : sampled) {
    const double clearance = pointClearance(point, checker);
    if (!std::isfinite(clearance)) {
      continue;
    }
    const double clamped = std::max(0.02, clearance);
    penalty += 1.0 / clamped;
  }
  return sampled.empty() ? 0.0 : penalty / static_cast<double>(sampled.size());
}

double TopoPathModifier::minClearanceOnPath(
  const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const
{
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_clearance = std::numeric_limits<double>::infinity();
  const std::vector<Eigen::Vector3d> sampled =
    discretizePath(path, std::max(8, options_.topo_equiv_sample_num / 2));
  for (const auto & point : sampled) {
    min_clearance = std::min(min_clearance, pointClearance(point, checker));
  }
  return min_clearance;
}

double TopoPathModifier::pathLength(const std::vector<Eigen::Vector3d> & path) const
{
  double length = 0.0;
  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    length += (path[i + 1U] - path[i]).norm();
  }
  return length;
}

std::vector<Eigen::Vector3d> TopoPathModifier::discretizePath(
  const std::vector<Eigen::Vector3d> & path, const int point_num) const
{
  if (path.empty()) {
    return {};
  }
  if (path.size() == 1U || point_num <= 1) {
    return {path.front()};
  }

  const double total_length = pathLength(path);
  if (total_length < kEpsilon) {
    return std::vector<Eigen::Vector3d>(static_cast<std::size_t>(point_num), path.front());
  }

  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t i = 1U; i < path.size(); ++i) {
    cumulative[i] = cumulative[i - 1U] + (path[i] - path[i - 1U]).norm();
  }

  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(static_cast<std::size_t>(point_num));
  std::size_t seg = 0;
  for (int i = 0; i < point_num; ++i) {
    const double target =
      total_length * static_cast<double>(i) / static_cast<double>(point_num - 1);
    while (seg + 1U < cumulative.size() && cumulative[seg + 1U] < target) {
      ++seg;
    }
    if (seg + 1U >= path.size()) {
      sampled.push_back(path.back());
      continue;
    }
    const double seg_len = cumulative[seg + 1U] - cumulative[seg];
    if (seg_len < kEpsilon) {
      sampled.push_back(path[seg]);
      continue;
    }
    const double ratio = (target - cumulative[seg]) / seg_len;
    sampled.push_back(path[seg] + ratio * (path[seg + 1U] - path[seg]));
  }
  return sampled;
}

std::vector<Eigen::Vector3d> TopoPathModifier::discretizePath(
  const std::vector<Eigen::Vector3d> & path) const
{
  if (path.size() < 2U) {
    return path;
  }

  std::vector<Eigen::Vector3d> sampled;
  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    const Eigen::Vector3d a = path[i];
    const Eigen::Vector3d b = path[i + 1U];
    const double length = (b - a).norm();
    const int steps =
      std::max(1, static_cast<int>(std::ceil(length / options_.collision_check_step)));
    for (int s = 0; s <= steps; ++s) {
      if (!sampled.empty() && s == 0) {
        continue;
      }
      const double ratio = static_cast<double>(s) / static_cast<double>(steps);
      sampled.push_back(a + ratio * (b - a));
    }
  }
  return sampled;
}

std::vector<Eigen::Vector3d> TopoPathModifier::shortcutPath(
  const std::vector<Eigen::Vector3d> & path, const CollisionChecker & checker) const
{
  if (path.size() <= 2U) {
    return path;
  }

  std::vector<Eigen::Vector3d> current = path;
  for (int iter = 0; iter < std::max(1, options_.shortcut_iterations); ++iter) {
    std::vector<Eigen::Vector3d> shortened;
    std::size_t i = 0;
    shortened.push_back(current.front());
    while (i + 1U < current.size()) {
      std::size_t best = i + 1U;
      for (std::size_t j = current.size() - 1U; j > i + 1U; --j) {
        if (lineVisib(current[i], current[j], checker)) {
          best = j;
          break;
        }
      }
      shortened.push_back(current[best]);
      i = best;
    }

    if (shortened.size() >= current.size()) {
      break;
    }
    current = shortened;
  }
  return current;
}

std::vector<Eigen::Vector3d> TopoPathModifier::densifyPath(
  const std::vector<Eigen::Vector3d> & path) const
{
  if (path.size() < 2U || options_.waypoint_spacing <= 1.0e-6) {
    return path;
  }

  std::vector<Eigen::Vector3d> dense;
  dense.reserve(path.size() * 2U);
  dense.push_back(path.front());
  for (std::size_t i = 0; i + 1U < path.size(); ++i) {
    const Eigen::Vector3d a = path[i];
    const Eigen::Vector3d b = path[i + 1U];
    const double len = (b - a).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(len / options_.waypoint_spacing)));
    for (int s = 1; s <= steps; ++s) {
      dense.push_back(a + (static_cast<double>(s) / static_cast<double>(steps)) * (b - a));
    }
  }
  return dense;
}

void TopoPathModifier::appendUnique(
  std::vector<Eigen::Vector3d> & path, const Eigen::Vector3d & point) const
{
  if (path.empty() || (path.back() - point).norm() > 1.0e-6) {
    path.push_back(point);
  }
}

}  // namespace amprobo

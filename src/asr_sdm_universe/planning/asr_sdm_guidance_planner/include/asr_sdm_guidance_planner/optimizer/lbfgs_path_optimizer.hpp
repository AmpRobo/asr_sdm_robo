#ifndef ASR_SDM_GUIDANCE_PLANNER_OPTIMIZER_LBFGS_PATH_OPTIMIZER_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_OPTIMIZER_LBFGS_PATH_OPTIMIZER_HPP_

#include <asr_sdm_guidance_planner/map/voxel_esdf_map.hpp>
#include <asr_sdm_guidance_planner/planner/sphere_corridor.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace asr_sdm_guidance_planner
{

struct LbfgsPathOptimizerOptions
{
  bool enabled = true;
  int max_iterations = 120;
  int max_control_points = 80;
  double epsilon = 1.0e-4;
  double smooth_weight = 1.0;
  double length_weight = 0.05;
  double obstacle_weight = 12.0;
  double guide_weight = 0.08;
  double safe_distance = 0.20;
  double validity_check_step = 0.05;
  double extra_clearance = 0.0;
  double corridor_weight = 60.0;
};

struct OptimizerResult
{
  bool success = false;
  bool path_safe = false;
  std::string message;
  std::vector<Eigen::Vector3d> path;
};

class LbfgsPathOptimizer
{
public:
  explicit LbfgsPathOptimizer(const LbfgsPathOptimizerOptions & options = LbfgsPathOptimizerOptions());

  void setOptions(const LbfgsPathOptimizerOptions & options) { options_ = options; }
  const LbfgsPathOptimizerOptions & options() const { return options_; }

  OptimizerResult optimize(
    const std::vector<Eigen::Vector3d> & raw_path,
    const VoxelEsdfMap & map,
    const std::vector<CorridorSphere> * corridor = nullptr) const;

private:
  LbfgsPathOptimizerOptions options_;

  std::vector<Eigen::Vector3d> selectControlPoints(const std::vector<Eigen::Vector3d> & raw_path) const;
};

}  // namespace asr_sdm_guidance_planner

#endif  // ASR_SDM_GUIDANCE_PLANNER_OPTIMIZER_LBFGS_PATH_OPTIMIZER_HPP_

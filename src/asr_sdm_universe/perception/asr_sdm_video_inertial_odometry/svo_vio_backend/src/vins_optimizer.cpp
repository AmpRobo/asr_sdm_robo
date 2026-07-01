#include "svo_vio_backend/vins_optimizer.h"
#include "svo_vio_backend/vins_types.h"
#include <ceres/ceres.h>

namespace svo_vio_backend {

VinsOptimizer::VinsOptimizer()
{
  options_.max_num_iterations = 10;
  options_.minimizer_progress_to_stdout = false;
  options_.num_threads = 4;
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.trust_region_strategy_type = ceres::DOGLEG;
}

VinsOptimizer::~VinsOptimizer()
{
}

void VinsOptimizer::setMaxIterations(int max_iterations)
{
  options_.max_num_iterations = max_iterations;
}

void VinsOptimizer::setSolverTimeLimit(double time_limit)
{
  options_.max_solver_time_in_seconds = time_limit;
}

void VinsOptimizer::setNumThreads(int num_threads)
{
  options_.num_threads = num_threads;
}

bool VinsOptimizer::solve(ceres::Problem& problem)
{
  ceres::Solve(options_, &problem, &summary_);
  return summary_.IsSolutionUsable();
}

int ParameterBlockManager::addParameterBlock(double* data, int size,
                                           ceres::Manifold* manifold)
{
  ParameterBlock block;
  block.data = data;
  block.size = size;
  block.manifold = manifold;
  blocks_.push_back(block);
  return static_cast<int>(blocks_.size()) - 1;
}

double* ParameterBlockManager::getParameterBlock(int index)
{
  if (index >= 0 && index < static_cast<int>(blocks_.size())) {
    return blocks_[index].data;
  }
  return nullptr;
}

int ParameterBlockManager::getParameterBlockSize(int index) const
{
  if (index >= 0 && index < static_cast<int>(blocks_.size())) {
    return blocks_[index].size;
  }
  return 0;
}

std::vector<double*> ParameterBlockManager::getParameterBlockPointers()
{
  std::vector<double*> ptrs;
  for (const auto& block : blocks_) {
    ptrs.push_back(block.data);
  }
  return ptrs;
}

}  // namespace svo_vio_backend

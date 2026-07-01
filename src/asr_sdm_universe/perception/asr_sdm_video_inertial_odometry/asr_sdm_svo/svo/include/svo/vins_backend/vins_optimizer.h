#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace svo {

class VinsOptimizer
{
public:
  VinsOptimizer();
  ~VinsOptimizer();

  // Configure solver options
  void setMaxIterations(int max_iterations);
  void setSolverTimeLimit(double time_limit);
  void setNumThreads(int num_threads);

  // Solve the optimization problem
  bool solve(ceres::Problem& problem);

  // Get solver summary
  const ceres::Solver::Summary& getSummary() const { return summary_; }

private:
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;
};

/**
 * @brief Parameter block index manager
 * 
 * Manages the mapping between state variables and Ceres parameter blocks.
 */
class ParameterBlockManager
{
public:
  // Add a parameter block
  int addParameterBlock(double* data, int size, 
                        ceres::LocalParameterization* local_param = nullptr);
  
  // Get parameter block data
  double* getParameterBlock(int index) { return blocks_[index].data; }
  int getParameterBlockSize(int index) const { return blocks_[index].size; }

  // Build parameter block pointer array for Ceres
  std::vector<double*> getParameterBlockPointers();

private:
  struct ParameterBlock {
    double* data;
    int size;
    ceres::LocalParameterization* local_param;
  };
  std::vector<ParameterBlock> blocks_;
};

}  // namespace svo

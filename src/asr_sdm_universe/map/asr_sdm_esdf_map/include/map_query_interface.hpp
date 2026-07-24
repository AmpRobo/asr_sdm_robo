#ifndef ASR_SDM_ESDF_MAP_MAP_QUERY_INTERFACE_HPP_
#define ASR_SDM_ESDF_MAP_MAP_QUERY_INTERFACE_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <vector>

namespace amprobo
{

struct GridIndex
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const GridIndex & other) const
  {
    return !(*this == other);
  }
};

class MapQueryInterface
{
public:
  virtual ~MapQueryInterface() = default;

  virtual bool isReady() const = 0;
  virtual bool hasOccupancyLayer() const = 0;
  virtual bool hasDistanceField() const = 0;

  virtual bool isInMap(const Eigen::Vector3d & position) const = 0;
  virtual bool isInMap(const GridIndex & index) const = 0;
  virtual bool worldToIndex(const Eigen::Vector3d & position, GridIndex & index) const = 0;
  virtual Eigen::Vector3d indexToWorld(const GridIndex & index) const = 0;

  virtual int toAddress(const GridIndex & index) const = 0;
  virtual bool addressToIndex(int address, GridIndex & index) const = 0;
  virtual int voxelCount() const = 0;

  virtual const Eigen::Vector3d & origin() const = 0;
  virtual const Eigen::Vector3d & size() const = 0;
  virtual double resolution() const = 0;

  virtual bool isOccupied(const GridIndex & index) const = 0;
  virtual bool isOccupied(const Eigen::Vector3d & position) const = 0;
  virtual bool isFree(const GridIndex & index, double extra_clearance = 0.0) const = 0;
  virtual bool isFree(const Eigen::Vector3d & position, double extra_clearance = 0.0) const = 0;

  virtual bool findNearestFree(
    const GridIndex & seed,
    int max_radius_voxels,
    GridIndex & free_index) const = 0;

  virtual double distance(const GridIndex & index) const = 0;
  virtual double distance(const Eigen::Vector3d & position) const = 0;
  virtual Eigen::Vector3d gradient(const Eigen::Vector3d & position) const = 0;

  virtual bool segmentIsFree(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b,
    double step,
    double extra_clearance) const
  {
    const double length = (b - a).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(step, 1.0e-3))));

    for (int i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      const Eigen::Vector3d p = (1.0 - ratio) * a + ratio * b;
      if (!isFree(p, extra_clearance)) {
        return false;
      }
    }
    return true;
  }

  virtual bool pathIsFree(
    const std::vector<Eigen::Vector3d> & path,
    double step,
    double extra_clearance) const
  {
    if (path.empty()) {
      return false;
    }

    for (std::size_t i = 1; i < path.size(); ++i) {
      if (!segmentIsFree(path[i - 1], path[i], step, extra_clearance)) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace amprobo

#endif  // ASR_SDM_ESDF_MAP_MAP_QUERY_INTERFACE_HPP_

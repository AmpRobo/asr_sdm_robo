#ifndef ASR_SDM_ESDF_MAP__BINARY_MAP_IO_HPP_
#define ASR_SDM_ESDF_MAP__BINARY_MAP_IO_HPP_

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace asr_sdm_esdf_map::binary_map
{

using Vector3dList =
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

struct OccupancyData
{
  std::string frame_id;
  uint64_t record_count{0};
  Vector3dList occupied_centers;
  std::size_t invalid_records{0};
};

struct EsdfSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double distance{0.0};
};

using EsdfSampleList =
  std::vector<EsdfSample, Eigen::aligned_allocator<EsdfSample>>;

struct EsdfData
{
  std::string frame_id;
  uint64_t record_count{0};
  EsdfSampleList samples;
  std::size_t invalid_records{0};
};

bool readOccupancy(
  const std::string & path, OccupancyData & data, std::string & error);

bool readEsdf(
  const std::string & path, EsdfData & data, std::string & error);

}  // namespace asr_sdm_esdf_map::binary_map

#endif  // ASR_SDM_ESDF_MAP__BINARY_MAP_IO_HPP_

#include "binary_map_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>

namespace asr_sdm_esdf_map::binary_map
{
namespace
{

constexpr char kOccupancyMagic[] = "ASR_OCC_BIN";
constexpr char kEsdfMagic[] = "ASR_ESDF_BIN";
constexpr uint32_t kBinaryVersion = 1U;
constexpr uint32_t kOccupancyMapType = 1U;
constexpr uint32_t kEsdfMapType = 2U;
constexpr uint32_t kMaxFrameIdLength = 4096U;
constexpr uint64_t kMaxOccupancyRecords = 20000000ULL;
constexpr uint64_t kMaxEsdfRecords = 200000000ULL;

struct BinaryHeader
{
  char magic[16];
  uint32_t version{0};
  uint32_t map_type{0};
  int64_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  uint32_t floats_per_record{0};
  uint64_t record_count{0};
  uint32_t frame_id_length{0};
};

bool magicMatches(const char magic[16], const char * expected)
{
  char expected_magic[16] = {};
  const std::size_t length = std::min(std::strlen(expected), sizeof(expected_magic));
  std::memcpy(expected_magic, expected, length);
  return std::memcmp(magic, expected_magic, sizeof(expected_magic)) == 0;
}

bool readHeader(
  std::ifstream & stream, const std::string & path, const char * expected_magic,
  const uint32_t expected_map_type, const uint32_t expected_floats_per_record,
  const uint64_t maximum_records, const std::string & map_name,
  BinaryHeader & header, std::string & frame_id, std::string & error)
{
  stream.read(header.magic, sizeof(header.magic));
  stream.read(reinterpret_cast<char *>(&header.version), sizeof(header.version));
  stream.read(reinterpret_cast<char *>(&header.map_type), sizeof(header.map_type));
  stream.read(reinterpret_cast<char *>(&header.stamp_sec), sizeof(header.stamp_sec));
  stream.read(
    reinterpret_cast<char *>(&header.stamp_nanosec), sizeof(header.stamp_nanosec));
  stream.read(
    reinterpret_cast<char *>(&header.floats_per_record), sizeof(header.floats_per_record));
  stream.read(reinterpret_cast<char *>(&header.record_count), sizeof(header.record_count));
  stream.read(
    reinterpret_cast<char *>(&header.frame_id_length), sizeof(header.frame_id_length));

  if (!stream.good()) {
    error = "invalid " + map_name + " binary header: " + path;
    return false;
  }

  if (!magicMatches(header.magic, expected_magic)) {
    error = "invalid " + map_name + " binary magic: " + path;
    return false;
  }

  if (header.version != kBinaryVersion) {
    error = "unsupported " + map_name + " binary version " +
      std::to_string(header.version) + ": " + path;
    return false;
  }

  if (
    header.map_type != expected_map_type ||
    header.floats_per_record != expected_floats_per_record)
  {
    error = "unsupported " + map_name + " binary layout: " + path;
    return false;
  }

  if (header.record_count > maximum_records) {
    error = "too many records in " + map_name + " binary: " +
      std::to_string(header.record_count);
    return false;
  }

  if (header.frame_id_length > kMaxFrameIdLength) {
    error = "invalid frame_id length in " + map_name + " binary: " + path;
    return false;
  }

  frame_id.assign(header.frame_id_length, '\0');
  if (header.frame_id_length > 0U) {
    stream.read(frame_id.data(), static_cast<std::streamsize>(header.frame_id_length));
  }
  if (!stream.good()) {
    error = "unexpected EOF while reading frame_id from " + map_name + " binary: " + path;
    return false;
  }

  return true;
}

bool writeHeader(
  std::ofstream & stream, const char * magic, const uint32_t map_type,
  const uint32_t floats_per_record, const uint64_t record_count,
  const std::string & frame_id, const std::string & path, const std::string & map_name,
  std::string & error)
{
  if (frame_id.size() > kMaxFrameIdLength) {
    error = "invalid frame_id length while writing " + map_name + " binary: " + path;
    return false;
  }

  BinaryHeader header{};
  const std::size_t magic_length = std::min(std::strlen(magic), sizeof(header.magic));
  std::memcpy(header.magic, magic, magic_length);
  header.version = kBinaryVersion;
  header.map_type = map_type;

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  header.stamp_sec = nanoseconds / 1000000000LL;
  header.stamp_nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  header.floats_per_record = floats_per_record;
  header.record_count = record_count;
  header.frame_id_length = static_cast<uint32_t>(frame_id.size());

  stream.write(header.magic, sizeof(header.magic));
  stream.write(reinterpret_cast<const char *>(&header.version), sizeof(header.version));
  stream.write(reinterpret_cast<const char *>(&header.map_type), sizeof(header.map_type));
  stream.write(reinterpret_cast<const char *>(&header.stamp_sec), sizeof(header.stamp_sec));
  stream.write(
    reinterpret_cast<const char *>(&header.stamp_nanosec), sizeof(header.stamp_nanosec));
  stream.write(
    reinterpret_cast<const char *>(&header.floats_per_record),
    sizeof(header.floats_per_record));
  stream.write(
    reinterpret_cast<const char *>(&header.record_count), sizeof(header.record_count));
  stream.write(
    reinterpret_cast<const char *>(&header.frame_id_length), sizeof(header.frame_id_length));
  if (!frame_id.empty()) {
    stream.write(frame_id.data(), static_cast<std::streamsize>(frame_id.size()));
  }

  if (!stream.good()) {
    error = "failed to write " + map_name + " binary header: " + path;
    return false;
  }
  return true;
}

}  // namespace

bool readOccupancy(
  const std::string & path, OccupancyData & data, std::string & error)
{
  data = OccupancyData{};

  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    error = "failed to open occupancy binary file: " + path;
    return false;
  }

  BinaryHeader header;
  if (!readHeader(
        stream, path, kOccupancyMagic, kOccupancyMapType, 3U,
        kMaxOccupancyRecords, "occupancy", header, data.frame_id, error))
  {
    return false;
  }

  data.record_count = header.record_count;
  data.occupied_centers.reserve(static_cast<std::size_t>(header.record_count));

  for (uint64_t index = 0; index < header.record_count; ++index) {
    float xyz[3];
    stream.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
    if (!stream.good()) {
      error = "unexpected EOF while reading occupancy binary: " + path;
      return false;
    }

    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
      ++data.invalid_records;
      continue;
    }

    data.occupied_centers.emplace_back(xyz[0], xyz[1], xyz[2]);
  }

  return true;
}

bool readEsdf(const std::string & path, EsdfData & data, std::string & error)
{
  data = EsdfData{};

  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    error = "failed to open ESDF binary file: " + path;
    return false;
  }

  BinaryHeader header;
  if (!readHeader(
        stream, path, kEsdfMagic, kEsdfMapType, 4U,
        kMaxEsdfRecords, "ESDF", header, data.frame_id, error))
  {
    return false;
  }

  data.record_count = header.record_count;
  data.samples.reserve(static_cast<std::size_t>(header.record_count));

  for (uint64_t index = 0; index < header.record_count; ++index) {
    float xyzi[4];
    stream.read(reinterpret_cast<char *>(xyzi), sizeof(xyzi));
    if (!stream.good()) {
      error = "unexpected EOF while reading ESDF binary: " + path;
      return false;
    }

    if (
      !std::isfinite(xyzi[0]) || !std::isfinite(xyzi[1]) ||
      !std::isfinite(xyzi[2]) || !std::isfinite(xyzi[3]))
    {
      ++data.invalid_records;
      continue;
    }

    EsdfSample sample;
    sample.center = Eigen::Vector3d(xyzi[0], xyzi[1], xyzi[2]);
    sample.distance = static_cast<double>(xyzi[3]);
    data.samples.push_back(sample);
  }

  return true;
}

bool writeOccupancy(
  const std::string & path, const OccupancyData & data, std::string & error)
{
  std::ofstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    error = "failed to open occupancy binary file for writing: " + path;
    return false;
  }

  const uint64_t record_count = static_cast<uint64_t>(data.occupied_centers.size());
  if (record_count > kMaxOccupancyRecords) {
    error = "too many occupancy records to write: " + std::to_string(record_count);
    return false;
  }

  if (!writeHeader(
        stream, kOccupancyMagic, kOccupancyMapType, 3U, record_count, data.frame_id, path,
        "occupancy", error))
  {
    return false;
  }

  for (const Eigen::Vector3d & center : data.occupied_centers) {
    const float xyz[3] = {
      static_cast<float>(center.x()),
      static_cast<float>(center.y()),
      static_cast<float>(center.z())};
    stream.write(reinterpret_cast<const char *>(xyz), sizeof(xyz));
    if (!stream.good()) {
      error = "failed while writing occupancy binary: " + path;
      return false;
    }
  }

  stream.flush();
  if (!stream.good()) {
    error = "failed to flush occupancy binary: " + path;
    return false;
  }
  return true;
}

bool writeEsdf(const std::string & path, const EsdfData & data, std::string & error)
{
  std::ofstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    error = "failed to open ESDF binary file for writing: " + path;
    return false;
  }

  const uint64_t record_count = static_cast<uint64_t>(data.samples.size());
  if (record_count > kMaxEsdfRecords) {
    error = "too many ESDF records to write: " + std::to_string(record_count);
    return false;
  }

  if (!writeHeader(
        stream, kEsdfMagic, kEsdfMapType, 4U, record_count, data.frame_id, path, "ESDF",
        error))
  {
    return false;
  }

  for (const EsdfSample & sample : data.samples) {
    const float xyzi[4] = {
      static_cast<float>(sample.center.x()),
      static_cast<float>(sample.center.y()),
      static_cast<float>(sample.center.z()),
      static_cast<float>(sample.distance)};
    stream.write(reinterpret_cast<const char *>(xyzi), sizeof(xyzi));
    if (!stream.good()) {
      error = "failed while writing ESDF binary: " + path;
      return false;
    }
  }

  stream.flush();
  if (!stream.good()) {
    error = "failed to flush ESDF binary: " + path;
    return false;
  }
  return true;
}

}  // namespace asr_sdm_esdf_map::binary_map

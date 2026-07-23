#ifndef ESDF_MAP_HPP_
#define ESDF_MAP_HPP_

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <asr_sdm_esdf_map/raycast.hpp>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_min_idx_, map_max_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;

  /* input topics and input mode */
  string depth_topic_, odom_topic_, cloud_topic_;
  bool enable_depth_odom_, enable_pointcloud_odom_;

  /* preloaded binary map files */
  string preload_map_directory_;
  string preload_occupancy_filename_;
  string preload_esdf_filename_;
  double preload_source_resolution_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* local map update and clear */
  double local_bound_inflate_;
  int local_map_margin_;

  /* visualization and computation time display */
  double virtual_ceil_height_, ground_height_;
  bool show_esdf_time_, show_occ_time_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion, esdf

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  vector<double> occupancy_buffer_;
  vector<char> occupancy_buffer_neg;
  vector<char> occupancy_buffer_inflate_;
  vector<double> distance_buffer_;
  vector<double> distance_buffer_neg_;
  vector<double> distance_buffer_all_;
  vector<double> tmp_buffer1_;
  vector<double> tmp_buffer2_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Quaterniond camera_q_, last_camera_q_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;

  // flags of map state

  bool occ_need_update_, local_updated_, esdf_need_update_;
  bool has_first_depth_;
  bool has_odom_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating ESDF

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ESDFMap
{
public:
  ESDFMap() = default;
  ~ESDFMap() = default;

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d & pos, Eigen::Vector3i & id);
  inline void indexToPos(const Eigen::Vector3i & id, Eigen::Vector3d & pos);
  inline int toAddress(const Eigen::Vector3i & id);
  inline int toAddress(int x, int y, int z);
  inline bool isInMap(const Eigen::Vector3d & pos);
  inline bool isInMap(const Eigen::Vector3i & idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  inline int getInflateOccupancy(const Eigen::Vector3i & id);

  inline void boundIndex(Eigen::Vector3i & id);
  inline bool isUnknown(const Eigen::Vector3i & id);
  inline bool isUnknown(const Eigen::Vector3d & pos);
  inline bool isKnownFree(const Eigen::Vector3i & id);
  inline bool isKnownOccupied(const Eigen::Vector3i & id);

  // distance field management
  inline double getDistance(const Eigen::Vector3d & pos);
  inline double getDistance(const Eigen::Vector3i & id);
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d & grad);
  void getSurroundPts(
    const Eigen::Vector3d & pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d & diff);

  void updateESDF3d();
  void getSliceESDF(
    const double height, const double res, const Eigen::Vector4d & range,
    vector<Eigen::Vector3d> & slice, vector<Eigen::Vector3d> & grad,
    int sign = 1);  // 1 pos, 2 neg, 3 combined
  void initMap(const std::shared_ptr<rclcpp::Node> & nh);

  void checkDist();
  bool hasDepthObservation();
  bool odomValid();

  // preloaded binary map (occupancy.bin / esdf.bin) loading
  bool loadPreloadedMaps();
  bool loadOccupancyBinary(const std::string & path, std::string & status);
  bool loadEsdfBinary(const std::string & path, std::string & status);
  void getRegion(Eigen::Vector3d & ori, Eigen::Vector3d & size);
  double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<ESDFMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // get depth image and camera pose
  void depthOdomCallback(
    sensor_msgs::msg::Image::ConstSharedPtr img,
    nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void pointCloudCallback(sensor_msgs::msg::PointCloud::ConstSharedPtr msg);
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & latest_cloud);
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom);

  // update occupancy by raycasting, and update ESDF
  void updateOccupancyCallback();
  void updateESDFCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();
  void rebuildEsdfFromOccupancy();
  bool setPreloadedOccupiedVoxel(
    const Eigen::Vector3i & target_id, std::size_t & inserted_target_voxels);
  bool insertPreloadedSourceVoxel(
    const Eigen::Vector3d & source_center, double source_resolution,
    std::size_t & inserted_target_voxels);

  inline void inflatePoint(const Eigen::Vector3i & pt, int step, vector<Eigen::Vector3i> & pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(
    const Eigen::Vector3d & pt, const Eigen::Vector3d & camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicyImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  shared_ptr<rclcpp::Node> node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  SynchronizerImageOdom sync_image_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;

  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr esdf_timer_;

  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;

  bool preloaded_occupancy_grid_matches_target_{false};
};

/* ============================== definition of inline function
 * ============================== */

inline int ESDFMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int ESDFMap::toAddress(const int x, const int y, const int z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void ESDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline double ESDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double ESDFMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

inline bool ESDFMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool ESDFMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool ESDFMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool ESDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline double ESDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * mp_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= mp_.resolution_inv_;

  return dist;
}

inline void ESDFMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void ESDFMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int ESDFMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int ESDFMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int ESDFMap::getInflateOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int ESDFMap::getOccupancy(Eigen::Vector3i id) {
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool ESDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool ESDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline void ESDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void ESDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void ESDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

#endif

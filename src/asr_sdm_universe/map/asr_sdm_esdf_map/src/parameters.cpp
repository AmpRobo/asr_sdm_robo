//
// Created by tommy on 5/2/19.
//

#include "asr_sdm_esdf_map/parameters.hpp"
void amp::Parameters::SetParameters(const rclcpp::Node::SharedPtr &node) {
  resolution_ = node->declare_parameter<double>("resolution", 0.1);
  visualize_every_n_updates_ = node->declare_parameter<int>("visualize_every_n_updates", 1);
  min_ray_length_ = node->declare_parameter<double>("min_ray_length", 0.5);
  max_ray_length_ = node->declare_parameter<double>("max_ray_length", 5.0);
  double slice_vis_level_tmp, vis_lower_bound_tmp, vis_upper_bound_tmp;

  slice_vis_max_dist_ = node->declare_parameter<double>("slice_vis_max_dist", 2.0);
  slice_vis_level_tmp = node->declare_parameter<double>("slice_vis_level", 5.0);
  vis_lower_bound_tmp = node->declare_parameter<double>("vis_lower_bound", -5.0);
  vis_upper_bound_tmp = node->declare_parameter<double>("vis_upper_bound", 10.0);
  slice_vis_level_ = (int) (slice_vis_level_tmp / resolution_);
  vis_lower_bound_ = (int) (vis_lower_bound_tmp / resolution_);
  vis_upper_bound_ = (int) (vis_upper_bound_tmp / resolution_);

  center_x_ = node->declare_parameter<double>("center_x", 322.477357419);
  center_y_ = node->declare_parameter<double>("center_y", 237.076346481);
  focal_x_ = node->declare_parameter<double>("focal_x", 384.458089392);
  focal_y_ = node->declare_parameter<double>("focal_y", 383.982755697);
  ray_cast_num_thread_ = node->declare_parameter<int>("ray_cast_num_thread", 0);
  double radius_x, radius_y, radius_z;
  radius_x = node->declare_parameter<double>("radius_x", 3.0);
  radius_y = node->declare_parameter<double>("radius_y", 3.0);
  radius_z = node->declare_parameter<double>("radius_z", 1.5);
  radius_ = Eigen::Vector3d(radius_x, radius_y, radius_z);

  global_map_ = node->declare_parameter<bool>("global_map", true);
  global_update_ = node->declare_parameter<bool>("global_update", true);
  global_vis_ = node->declare_parameter<bool>("global_vis", true);
  if (!global_map_)
    global_vis_ = global_update_ = false;

  use_depth_filter_ = node->declare_parameter<bool>("use_depth_filter", true);
  depth_filter_tolerance_ = node->declare_parameter<double>("depth_filter_tolerance", 0.1);
  depth_filter_max_dist_ = node->declare_parameter<double>("depth_filter_max_dist", 10.0);
  depth_filter_min_dist_ = node->declare_parameter<double>("depth_filter_min_dist", 0.1);
  depth_filter_margin_ = node->declare_parameter<int>("depth_filter_margin", 0);

#ifdef HASH_TABLE
  l_cornor_ << -100.f, -100.f, -100.f;
    r_cornor_ << 100.f, 100.f, 100.f;
    reserved_size_ = node->declare_parameter<int>("reserved_size", 1000000);
#else
  double lx, ly, lz;
  double rx, ry, rz;
  lx = node->declare_parameter<double>("lx", -20.0);
  ly = node->declare_parameter<double>("ly", -20.0);
  lz = node->declare_parameter<double>("lz", -5.0);
  rx = node->declare_parameter<double>("rx", 20.0);
  ry = node->declare_parameter<double>("ry", 20.0);
  rz = node->declare_parameter<double>("rz", 5.0);

  l_cornor_ << lx, ly, lz;
  r_cornor_ << rx, ry, rz;
  map_size_ = r_cornor_ - l_cornor_;
#endif

  update_esdf_every_n_sec_ = node->declare_parameter<double>("update_esdf_every_n_sec", 0.1);
//  T_B_C_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
//  T_D_B_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  // LADY_AND_COW
  T_B_C_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
  T_D_B_ << 0.971048, -0.120915, 0.206023, 0.00114049,
           0.15701, 0.973037, -0.168959, 0.0450936,
           -0.180038, 0.196415, 0.96385, 0.0430765,
           0.0, 0.0, 0.0, 1.0;

  // EuRoC
  //    T_B_C << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //            0.0, 0.0, 0.0, 1.0;
  //    T_D_B << 0.33638, -0.01749, 0.94156, 0.06901,
  //            -0.02078, -0.99972, -0.01114, -0.02781,
  //            0.94150, -0.01582, -0.33665, -0.12395,
  //            0.0, 0.0, 0.0, 1.0;
  //    T_D_B = T_D_B.inverse();

#ifdef PROBABILISTIC
  p_hit_ = node->declare_parameter<double>("p_hit", 0.70);
  p_miss_ = node->declare_parameter<double>("p_miss", 0.35);
  p_min_ = node->declare_parameter<double>("p_min", 0.12);
  p_max_ = node->declare_parameter<double>("p_max", 0.97);
  p_occ_ = node->declare_parameter<double>("p_occ", 0.80);
//    esdf_map_->SetParameters(p_hit_, p_miss_, p_min_, p_max_, p_occ_);
#endif
}
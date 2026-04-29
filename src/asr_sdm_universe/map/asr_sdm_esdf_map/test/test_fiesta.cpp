#include "asr_sdm_esdf_map/asr_sdm_esdf_map.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("asr_sdm_esdf_map");
  amp::Fiesta<sensor_msgs::msg::PointCloud2::ConstSharedPtr,
                 geometry_msgs::msg::TransformStamped::ConstSharedPtr> esdf_map(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

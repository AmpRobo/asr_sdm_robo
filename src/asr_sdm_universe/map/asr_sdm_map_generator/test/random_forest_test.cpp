#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_map_generator/random_forest_sensing.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<asr_sdm_map_generator::RandomForestSensing>());
  rclcpp::shutdown();
  return 0;
}

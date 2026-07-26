#ifndef ASR_SDM_VIDEO_ENHANCEMENT_ML__ASR_SDM_VIDEO_ENHANCEMENT_ML_NODE_HPP_
#define ASR_SDM_VIDEO_ENHANCEMENT_ML__ASR_SDM_VIDEO_ENHANCEMENT_ML_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tensorrt_common/engine.hpp>

namespace asr
{

class VideoEnhancementMlNode : public rclcpp::Node
{
public:
  explicit VideoEnhancementMlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  cv::Mat tensorToBgrImage(
    const std::vector<float> & output_data, int channels, int height, int width,
    int crop_width, int crop_height) const;
  void logStats(std::chrono::duration<double, std::milli> callback_ms,
    std::chrono::duration<double, std::milli> inference_ms);

  std::unique_ptr<Engine> engine_;
  int net_height_;
  int net_width_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

  std::string model_path_;
  std::string input_topic_;
  std::string output_topic_;
  bool normalize_output_;

  size_t frame_count_;
  double accumulated_callback_ms_;
  double accumulated_inference_ms_;
};

}  // namespace asr

#endif  // ASR_SDM_VIDEO_ENHANCEMENT_ML__ASR_SDM_VIDEO_ENHANCEMENT_ML_NODE_HPP_

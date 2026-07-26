#include "asr_sdm_video_enhancement_ml/asr_sdm_video_enhancement_ml_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace enc = sensor_msgs::image_encodings;
using std::placeholders::_1;

namespace asr
{

VideoEnhancementMlNode::VideoEnhancementMlNode(const rclcpp::NodeOptions & options)
: Node("asr_sdm_video_enhancement_ml_node", options),
  net_height_(0),
  net_width_(0),
  normalize_output_(true),
  frame_count_(0),
  accumulated_callback_ms_(0.0),
  accumulated_inference_ms_(0.0)
{
  model_path_ = declare_parameter<std::string>("model_path", "models/uieb_model.onnx");
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "/sensing/camera/camera0/image_raw/compressed");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/perception/video_enhancement/image_enhanced/compressed");
  normalize_output_ = declare_parameter<bool>("normalize_output", true);

  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error("ONNX model does not exist: " + model_path_);
  }

  Options engine_options;
  engine_options.precision = Precision::FP16;
  engine_options.optBatchSize = 1;
  engine_options.maxBatchSize = 1;
  engine_options.deviceIndex = 0;

  engine_ = std::make_unique<Engine>(engine_options);

  // UIE / FiveA+: normalize inputs to [0, 1]
  const std::array<float, 3> sub_vals{0.f, 0.f, 0.f};
  const std::array<float, 3> div_vals{1.f, 1.f, 1.f};
  constexpr bool normalize = true;

  if (!engine_->build(model_path_, sub_vals, div_vals, normalize)) {
    throw std::runtime_error("Failed to build TensorRT engine from: " + model_path_);
  }
  if (!engine_->loadNetwork(model_path_)) {
    throw std::runtime_error("Failed to load TensorRT engine from: " + model_path_);
  }

  const auto & input_dims = engine_->getInputDims();
  if (input_dims.empty()) {
    throw std::runtime_error("TensorRT engine has no input tensors");
  }
  // getInputDims() returns Dims3(C, H, W)
  net_height_ = input_dims[0].d[1] > 0 ? input_dims[0].d[1] : 256;
  net_width_ = input_dims[0].d[2] > 0 ? input_dims[0].d[2] : 256;

  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic_, rclcpp::QoS(10));
  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    input_topic_, rclcpp::SensorDataQoS(), std::bind(&VideoEnhancementMlNode::imageCallback, this, _1));

  RCLCPP_INFO(get_logger(), "Video enhancement ML node loaded %s (TensorRT %dx%d)",
    model_path_.c_str(), net_width_, net_height_);
  RCLCPP_INFO(get_logger(), "Subscribing %s, publishing %s", input_topic_.c_str(), output_topic_.c_str());
}

void VideoEnhancementMlNode::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", error.what());
    return;
  }

  if (cv_ptr->image.rows < 128 || cv_ptr->image.cols < 128) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Input image is too small: %dx%d; minimum is 128x128.",
      cv_ptr->image.cols, cv_ptr->image.rows);
    return;
  }

  const int orig_h = cv_ptr->image.rows;
  const int orig_w = cv_ptr->image.cols;
  // Match Engine::resizeKeepAspectRatioPadRightBottom scaling so we can remove letterbox padding.
  const float scale = std::min(
    static_cast<float>(net_width_) / static_cast<float>(orig_w),
    static_cast<float>(net_height_) / static_cast<float>(orig_h));
  const int unpad_w = static_cast<int>(scale * orig_w);
  const int unpad_h = static_cast<int>(scale * orig_h);

  cv::cuda::GpuMat gpu_rgb;
  gpu_rgb.upload(cv_ptr->image);
  cv::cuda::cvtColor(gpu_rgb, gpu_rgb, cv::COLOR_BGR2RGB);

  auto resized = Engine::resizeKeepAspectRatioPadRightBottom(
    gpu_rgb, static_cast<size_t>(net_height_), static_cast<size_t>(net_width_));

  // Input layout: [model_input][batch][GpuMat]
  std::vector<std::vector<cv::cuda::GpuMat>> inputs{{std::move(resized)}};
  std::vector<std::vector<std::vector<float>>> feature_vectors;

  const auto inference_start = std::chrono::steady_clock::now();
  if (!engine_->runInference(inputs, feature_vectors)) {
    RCLCPP_ERROR(get_logger(), "TensorRT inference failed");
    return;
  }
  const auto inference_end = std::chrono::steady_clock::now();

  if (feature_vectors.empty() || feature_vectors[0].empty()) {
    RCLCPP_ERROR(get_logger(), "TensorRT inference returned no outputs");
    return;
  }

  int out_c = 3;
  int out_h = net_height_;
  int out_w = net_width_;
  const auto & output_dims = engine_->getOutputDims();
  if (!output_dims.empty() && output_dims[0].nbDims >= 4) {
    out_c = output_dims[0].d[1] > 0 ? output_dims[0].d[1] : out_c;
    out_h = output_dims[0].d[2] > 0 ? output_dims[0].d[2] : out_h;
    out_w = output_dims[0].d[3] > 0 ? output_dims[0].d[3] : out_w;
  }

  const int crop_w = std::min(unpad_w, out_w);
  const int crop_h = std::min(unpad_h, out_h);
  cv::Mat cropped_bgr = tensorToBgrImage(
    feature_vectors[0][0], out_c, out_h, out_w, crop_w, crop_h);
  if (cropped_bgr.empty()) {
    return;
  }

  cv::Mat bgr_image;
  cv::resize(cropped_bgr, bgr_image, cv::Size(orig_w, orig_h), 0.0, 0.0, cv::INTER_LINEAR);

  auto output_msg = cv_bridge::CvImage(msg->header, enc::BGR8, bgr_image).toCompressedImageMsg();
  image_pub_->publish(*output_msg);

  const auto callback_end = std::chrono::steady_clock::now();
  logStats(
    std::chrono::duration<double, std::milli>(callback_end - callback_start),
    std::chrono::duration<double, std::milli>(inference_end - inference_start));
}

cv::Mat VideoEnhancementMlNode::tensorToBgrImage(
  const std::vector<float> & output_data, int channels, int height, int width,
  int crop_width, int crop_height) const
{
  if (channels != 3 || height <= 0 || width <= 0 || crop_width <= 0 || crop_height <= 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unexpected TensorRT output shape. Expected [3, H, W] with positive crop, got [%d, %d, %d] crop %dx%d.",
      channels, height, width, crop_width, crop_height);
    return cv::Mat();
  }

  crop_width = std::min(crop_width, width);
  crop_height = std::min(crop_height, height);

  const size_t plane_size = static_cast<size_t>(height) * static_cast<size_t>(width);
  const size_t tensor_size = 3 * plane_size;
  if (output_data.size() < tensor_size) {
    RCLCPP_ERROR(
      this->get_logger(),
      "TensorRT output too small: got %zu floats, need %zu for %dx%d RGB.",
      output_data.size(), tensor_size, width, height);
    return cv::Mat();
  }

  float min_value = 0.0F;
  float max_value = 1.0F;
  bool needs_normalize = false;
  if (normalize_output_) {
    min_value = std::numeric_limits<float>::infinity();
    max_value = -std::numeric_limits<float>::infinity();
    // Ignore letterbox padding when estimating the display range.
    for (int y = 0; y < crop_height; ++y) {
      for (int x = 0; x < crop_width; ++x) {
        const size_t offset = static_cast<size_t>(y * width + x);
        min_value = std::min(min_value, output_data[offset]);
        min_value = std::min(min_value, output_data[plane_size + offset]);
        min_value = std::min(min_value, output_data[2 * plane_size + offset]);
        max_value = std::max(max_value, output_data[offset]);
        max_value = std::max(max_value, output_data[plane_size + offset]);
        max_value = std::max(max_value, output_data[2 * plane_size + offset]);
      }
    }
    needs_normalize = (min_value < 0.0F || max_value > 1.0F);
  }

  const float denom = needs_normalize ? (max_value - min_value + 1e-7F) : 1.0F;
  cv::Mat bgr(crop_height, crop_width, CV_8UC3);
  for (int y = 0; y < crop_height; ++y) {
    auto * row = bgr.ptr<cv::Vec3b>(y);
    for (int x = 0; x < crop_width; ++x) {
      const size_t offset = static_cast<size_t>(y * width + x);
      float r = output_data[offset];
      float g = output_data[plane_size + offset];
      float b = output_data[2 * plane_size + offset];
      if (needs_normalize) {
        r = (r - min_value) / denom;
        g = (g - min_value) / denom;
        b = (b - min_value) / denom;
      }
      row[x] = cv::Vec3b{
        static_cast<unsigned char>(std::lround(std::clamp(b, 0.0F, 1.0F) * 255.0F)),
        static_cast<unsigned char>(std::lround(std::clamp(g, 0.0F, 1.0F) * 255.0F)),
        static_cast<unsigned char>(std::lround(std::clamp(r, 0.0F, 1.0F) * 255.0F))};
    }
  }
  return bgr;
}

void VideoEnhancementMlNode::logStats(
  std::chrono::duration<double, std::milli> callback_ms,
  std::chrono::duration<double, std::milli> inference_ms)
{
  ++frame_count_;
  accumulated_callback_ms_ += callback_ms.count();
  accumulated_inference_ms_ += inference_ms.count();
  if (frame_count_ % 30 != 0) {
    return;
  }

  const double callback_fps = 1000.0 / (accumulated_callback_ms_ / static_cast<double>(frame_count_));
  const double inference_fps = 1000.0 / (accumulated_inference_ms_ / static_cast<double>(frame_count_));
  RCLCPP_INFO(
    get_logger(), "Video enhancement ML processed %zu frames: callback %.2f FPS, inference %.2f FPS",
    frame_count_, callback_fps, inference_fps);
}

}  // namespace asr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<asr::VideoEnhancementMlNode>());
  rclcpp::shutdown();
  return 0;
}

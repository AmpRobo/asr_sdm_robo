#include "asr_sdm_control_manager/front_unit_following_controller_3d.hpp"

#include <matplot/matplot.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

namespace plt = matplot;
using namespace std::chrono_literals;

namespace
{

constexpr size_t kStateSize3D = 43;
constexpr size_t kJoint1BodyPointIndex = 1;

struct ControllerState3D
{
  double t;
  asr::Vec3 head_position;
  asr::Mat3 head_frame;
  asr::HeadCommand3D cmd;
  std::array<double, asr::kNum3dJointDofs> theta;
  std::array<double, asr::kNum3dJointDofs> theta_dot;
  std::array<asr::Vec3, asr::kNum3dPoints> body_points;
};

std::vector<double> values_for_point(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples, size_t point,
  char axis, size_t max_count = 0)
{
  std::vector<double> values;
  const size_t start = max_count > 0 && samples.size() > max_count ? samples.size() - max_count : 0;
  values.reserve(samples.size() - start);
  for (size_t i = start; i < samples.size(); ++i) {
    const auto & sample = samples[i];
    if (axis == 'x') {
      values.push_back(sample[point].x);
    } else if (axis == 'y') {
      values.push_back(sample[point].y);
    } else {
      values.push_back(sample[point].z);
    }
  }
  return values;
}

std::array<std::vector<double>, 3> values_for_body(
  const std::array<asr::Vec3, asr::kNum3dPoints> & body)
{
  std::array<std::vector<double>, 3> values;
  for (auto & axis_values : values) {
    axis_values.reserve(body.size());
  }
  for (const auto & p : body) {
    values[0].push_back(p.x);
    values[1].push_back(p.y);
    values[2].push_back(p.z);
  }
  return values;
}

void update_line3(
  const plt::line_handle & handle, const std::vector<double> & x,
  const std::vector<double> & y, const std::vector<double> & z)
{
  handle->x_data(x);
  handle->y_data(y);
  handle->z_data(z);
}

std::array<std::array<double, 2>, 3> body_history_limits(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  const std::array<asr::Vec3, asr::kNum3dPoints> & initial_points, size_t max_count,
  double margin)
{
  asr::Vec3 min_value = initial_points[0];
  asr::Vec3 max_value = initial_points[0];

  auto expand = [&](const asr::Vec3 & p) {
    min_value.x = std::min(min_value.x, p.x);
    min_value.y = std::min(min_value.y, p.y);
    min_value.z = std::min(min_value.z, p.z);
    max_value.x = std::max(max_value.x, p.x);
    max_value.y = std::max(max_value.y, p.y);
    max_value.z = std::max(max_value.z, p.z);
  };

  for (const auto & p : initial_points) {
    expand(p);
  }

  const size_t start = max_count > 0 && samples.size() > max_count ? samples.size() - max_count : 0;
  for (size_t i = start; i < samples.size(); ++i) {
    for (const auto & p : samples[i]) {
      expand(p);
    }
  }

  auto axis_limits = [margin](double min_axis, double max_axis) {
    const double span = std::max(max_axis - min_axis, margin);
    const double padding = std::max(margin, 0.1 * span);
    return std::array<double, 2>{min_axis - padding, max_axis + padding};
  };

  return {
    axis_limits(min_value.x, max_value.x),
    axis_limits(min_value.y, max_value.y),
    axis_limits(min_value.z, max_value.z)};
}

}  // namespace

class RealtimeControllerVisualizer3DNode : public rclcpp::Node
{
public:
  RealtimeControllerVisualizer3DNode()
  : Node("realtime_controller_visualizer_3d_" + std::to_string(getpid()))
  {
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", "/control/asr_sdm/controller_state_3d");
    draw_period_ms_ = this->declare_parameter<int>("draw_period_ms", 250);
    max_history_size_ = this->declare_parameter<int>("max_history_size", 3000);
    trail_samples_ = this->declare_parameter<int>("trail_samples", 360);
    body_snapshot_stride_ = this->declare_parameter<int>("body_snapshot_stride", 80);
    follow_current_body_ = this->declare_parameter<bool>("follow_current_body", true);
    view_half_range_ = this->declare_parameter<double>("view_half_range", 1.0);

    sub_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      state_topic_, rclcpp::QoS(100),
      std::bind(&RealtimeControllerVisualizer3DNode::onControllerState, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(draw_period_ms_),
      std::bind(&RealtimeControllerVisualizer3DNode::onDrawTimer, this));

    trajectory_figure_ = plt::figure(true);
    trajectory_figure_->width(1400);
    trajectory_figure_->height(900);
    plt::figure(trajectory_figure_);
    plt::tiledlayout(2, 3);
    trajectory_axes_ = plt::nexttile();
    xy_axes_ = plt::nexttile();
    trajectory_legend_axes_ = plt::nexttile();
    xz_axes_ = plt::nexttile();
    yz_axes_ = plt::nexttile();
    plane_legend_axes_ = plt::nexttile();
    configureWindowExitBindings();

    RCLCPP_INFO(this->get_logger(), "Realtime 3D visualizer started: state=%s", state_topic_.c_str());
  }

private:
  void configureWindowExitBindings()
  {
    trajectory_figure_->backend()->run_command("bind Close 'exit gnuplot'");
    trajectory_figure_->backend()->run_command("bind q 'exit gnuplot'");
    trajectory_figure_->backend()->render_data();
  }

  void onControllerState(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < kStateSize3D) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore 3D controller state: expected %zu values, got %zu", kStateSize3D, msg->data.size());
      return;
    }

    ControllerState3D state;
    state.t = msg->data[0];
    state.head_position = {msg->data[1], msg->data[2], msg->data[3]};

    size_t index = 4;
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        state.head_frame.v[row][col] = msg->data[index++];
      }
    }

    state.cmd = {msg->data[13], msg->data[14], msg->data[15]};
    for (size_t i = 0; i < asr::kNum3dJointDofs; ++i) {
      state.theta[i] = msg->data[16 + i];
      state.theta_dot[i] = msg->data[22 + i];
    }
    for (size_t point = 0; point < asr::kNum3dPoints; ++point) {
      state.body_points[point] = {
        msg->data[28 + 3 * point],
        msg->data[28 + 3 * point + 1],
        msg->data[28 + 3 * point + 2]};
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_ = state;
    has_state_ = true;
  }

  void onDrawTimer()
  {
    if (!rclcpp::ok()) {
      return;
    }

    ControllerState3D state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_state_) {
        return;
      }
      state = latest_state_;
    }

    appendSample(state);
    drawTrajectoryFrame(state);
    drawPlanePathFrame(xy_axes_, xy_panel_, state, 'x', 'y', "XY joint1 path", "x [m]", "y [m]");
    drawPlanePathFrame(xz_axes_, xz_panel_, state, 'x', 'z', "XZ joint1 path", "x [m]", "z [m]");
    drawPlanePathFrame(yz_axes_, yz_panel_, state, 'y', 'z', "YZ joint1 path", "y [m]", "z [m]");
    trajectory_figure_->draw();
  }

  void appendSample(const ControllerState3D & state)
  {
    time_history_.push_back(state.t);
    body_history_.push_back(state.body_points);

    if (!has_initial_body_) {
      initial_points_ = state.body_points;
      has_initial_body_ = true;
    }

    trimHistory();
  }

  void trimHistory()
  {
    if (max_history_size_ <= 0 || time_history_.size() <= static_cast<size_t>(max_history_size_)) {
      return;
    }

    eraseFront(time_history_);
    eraseFront(body_history_);
  }

  template<typename T>
  void eraseFront(std::vector<T> & values) const
  {
    const size_t extra = values.size() - static_cast<size_t>(max_history_size_);
    values.erase(values.begin(), values.begin() + extra);
  }

  std::array<std::array<double, 2>, 3> currentBodyViewLimits(const ControllerState3D & state) const
  {
    asr::Vec3 center{0.0, 0.0, 0.0};
    for (const auto & p : state.body_points) {
      center = center + p;
    }
    center = center / static_cast<double>(state.body_points.size());

    const double half_range = std::max(view_half_range_, 0.1);
    return {
      std::array<double, 2>{center.x - half_range, center.x + half_range},
      std::array<double, 2>{center.y - half_range, center.y + half_range},
      std::array<double, 2>{center.z - half_range, center.z + half_range}};
  }

  std::array<std::array<double, 2>, 3> viewLimits(const ControllerState3D & state) const
  {
    if (follow_current_body_) {
      return currentBodyViewLimits(state);
    }
    return body_history_limits(body_history_, initial_points_, trail_samples_, 0.08);
  }

  void drawTrajectoryFrame(const ControllerState3D & state)
  {
    if (!trajectory_initialized_) {
      plt::hold(trajectory_axes_, true);
      joint1_path_ = plt::plot3(
        trajectory_axes_, values_for_point(body_history_, kJoint1BodyPointIndex, 'x', trail_samples_),
        values_for_point(body_history_, kJoint1BodyPointIndex, 'y', trail_samples_),
        values_for_point(body_history_, kJoint1BodyPointIndex, 'z', trail_samples_), "r-");
      joint1_path_->line_width(3.0);

      const size_t max_snapshot_count =
        static_cast<size_t>(std::max(1, trail_samples_) / std::max(1, body_snapshot_stride_)) + 1;
      snapshot_handles_.reserve(max_snapshot_count);
      for (size_t i = 0; i < max_snapshot_count; ++i) {
        auto snapshot = plt::plot3(
          trajectory_axes_, std::vector<double>{}, std::vector<double>{}, std::vector<double>{}, ".-");
        snapshot->color("k");
        snapshot->line_width(0.5);
        snapshot->marker_size(3);
        snapshot_handles_.push_back(snapshot);
      }

      const auto initial_body_values = values_for_body(initial_points_);
      initial_body_ = plt::plot3(
        trajectory_axes_, initial_body_values[0], initial_body_values[1], initial_body_values[2], "--s");
      initial_body_->color("k");
      initial_body_->line_width(1.0);
      initial_body_->marker_size(6);

      const auto body_values = values_for_body(state.body_points);
      current_body_ = plt::plot3(
        trajectory_axes_, body_values[0], body_values[1], body_values[2], "-ok");
      current_body_->line_width(3.0);
      current_body_->marker_size(7);

      plt::title(trajectory_axes_, "Realtime 3D follow-the-leader body simulation");
      plt::xlabel(trajectory_axes_, "x [m]");
      plt::ylabel(trajectory_axes_, "y [m]");
      plt::zlabel(trajectory_axes_, "z [m]");
      plt::view(45, 28);
      plt::grid(trajectory_axes_, true);
      drawTrajectoryLegend();
      trajectory_initialized_ = true;
    }

    update_line3(
      joint1_path_, values_for_point(body_history_, kJoint1BodyPointIndex, 'x', trail_samples_),
      values_for_point(body_history_, kJoint1BodyPointIndex, 'y', trail_samples_),
      values_for_point(body_history_, kJoint1BodyPointIndex, 'z', trail_samples_));

    const size_t snapshot_start = body_history_.size() > static_cast<size_t>(trail_samples_) ?
      body_history_.size() - static_cast<size_t>(trail_samples_) : 0;
    size_t handle_index = 0;
    for (size_t sample = snapshot_start;
      sample < body_history_.size() && handle_index < snapshot_handles_.size();
      sample += static_cast<size_t>(std::max(1, body_snapshot_stride_)), ++handle_index)
    {
      const auto snapshot_values = values_for_body(body_history_[sample]);
      update_line3(
        snapshot_handles_[handle_index], snapshot_values[0], snapshot_values[1], snapshot_values[2]);
      snapshot_handles_[handle_index]->visible(true);
    }
    for (; handle_index < snapshot_handles_.size(); ++handle_index) {
      snapshot_handles_[handle_index]->visible(false);
    }

    const auto body_values = values_for_body(state.body_points);
    update_line3(current_body_, body_values[0], body_values[1], body_values[2]);

    const auto limits = viewLimits(state);
    plt::xrange(trajectory_axes_, limits[0]);
    plt::yrange(trajectory_axes_, limits[1]);
    trajectory_axes_->z_axis().limits(limits[2]);
    trajectory_axes_->z_axis().limits_mode_auto(false);
  }

  void drawTrajectoryLegend()
  {
    if (trajectory_legend_initialized_) {
      return;
    }

    plt::hold(trajectory_legend_axes_, true);
    trajectory_legend_axes_->box(false);
    trajectory_legend_axes_->x_axis().visible(false);
    trajectory_legend_axes_->y_axis().visible(false);
    auto joint1_path_sample = plt::plot(trajectory_legend_axes_, {0.0, 1.0}, {0.0, 0.0}, "r-");
    joint1_path_sample->line_width(3.0);
    auto initial_body_sample = plt::plot(trajectory_legend_axes_, {0.0, 1.0}, {0.0, 0.0}, "--s");
    initial_body_sample->color("k");
    initial_body_sample->line_width(1.0);
    initial_body_sample->marker_size(6);
    auto current_body_sample = plt::plot(trajectory_legend_axes_, {0.0, 1.0}, {0.0, 0.0}, "-ok");
    current_body_sample->line_width(3.0);
    current_body_sample->marker_size(7);
    auto legend = plt::legend(
      trajectory_legend_axes_, {"joint1 path (link1-link2)", "initial body", "current body"});
    legend->location(matplot::legend::general_alignment::center);
    legend->vertical(true);
    plt::xrange(trajectory_legend_axes_, {0.0, 1.0});
    plt::yrange(trajectory_legend_axes_, {-1.0, 1.0});
    trajectory_legend_initialized_ = true;
  }

  void drawPlaneLegend()
  {
    if (plane_legend_initialized_) {
      return;
    }

    plt::hold(plane_legend_axes_, true);
    plane_legend_axes_->box(false);
    plane_legend_axes_->x_axis().visible(false);
    plane_legend_axes_->y_axis().visible(false);
    auto joint1_sample = plt::plot(
      plane_legend_axes_, {0.0, 1.0}, {0.0, 0.0}, pathLineSpec(kJoint1BodyPointIndex));
    joint1_sample->line_width(3.0);
    auto current_body_sample = plt::plot(plane_legend_axes_, {0.0, 1.0}, {0.0, 0.0}, "-ok");
    current_body_sample->line_width(2.0);
    current_body_sample->marker_size(5);
    auto legend = plt::legend(plane_legend_axes_, {"joint1 (link1-link2)", "current body"});
    legend->location(matplot::legend::general_alignment::center);
    legend->vertical(true);
    plt::xrange(plane_legend_axes_, {0.0, 1.0});
    plt::yrange(plane_legend_axes_, {-1.0, 1.0});
    plane_legend_initialized_ = true;
  }

  struct PlanePathPanel
  {
    std::array<matplot::line_handle, asr::kNum3dPoints - 1> path_lines;
    matplot::line_handle current_body;
    bool initialized{false};
  };

  void drawPlanePathFrame(
    const matplot::axes_handle & axes, PlanePathPanel & panel, const ControllerState3D & state,
    char horizontal_axis, char vertical_axis, const std::string & title, const std::string & xlabel,
    const std::string & ylabel)
  {
    if (!panel.initialized) {
      plt::hold(axes, true);
      panel.path_lines[0] = plt::plot(
        axes, values_for_point(body_history_, kJoint1BodyPointIndex, horizontal_axis, trail_samples_),
        values_for_point(body_history_, kJoint1BodyPointIndex, vertical_axis, trail_samples_), pathLineSpec(kJoint1BodyPointIndex)).front();
      panel.path_lines[0]->line_width(3.0);

      const auto current_body_values = values_for_body_plane(state.body_points, horizontal_axis, vertical_axis, 0);
      panel.current_body = plt::plot(axes, current_body_values[0], current_body_values[1], "-ok");
      panel.current_body->line_width(2.0);
      panel.current_body->marker_size(5);

      plt::title(axes, title);
      plt::xlabel(axes, xlabel);
      plt::ylabel(axes, ylabel);
      plt::grid(axes, true);
      drawPlaneLegend();
      panel.initialized = true;
    } else {
      panel.path_lines[0]->x_data(values_for_point(body_history_, kJoint1BodyPointIndex, horizontal_axis, trail_samples_));
      panel.path_lines[0]->y_data(values_for_point(body_history_, kJoint1BodyPointIndex, vertical_axis, trail_samples_));

      const auto current_body_values = values_for_body_plane(state.body_points, horizontal_axis, vertical_axis, 0);
      panel.current_body->x_data(current_body_values[0]);
      panel.current_body->y_data(current_body_values[1]);
    }

    const auto limits = viewLimits(state);
    plt::xrange(axes, limits[axisIndex(horizontal_axis)]);
    plt::yrange(axes, limits[axisIndex(vertical_axis)]);
  }

  std::array<std::vector<double>, 2> values_for_body_plane(
    const std::array<asr::Vec3, asr::kNum3dPoints> & body, char horizontal_axis,
    char vertical_axis, size_t start_point) const
  {
    std::array<std::vector<double>, 2> values;
    for (auto & axis_values : values) {
      axis_values.reserve(body.size() - start_point);
    }
    for (size_t point = start_point; point < body.size(); ++point) {
      values[0].push_back(valueForAxis(body[point], horizontal_axis));
      values[1].push_back(valueForAxis(body[point], vertical_axis));
    }
    return values;
  }

  double valueForAxis(const asr::Vec3 & p, char axis) const
  {
    if (axis == 'x') {
      return p.x;
    }
    if (axis == 'y') {
      return p.y;
    }
    return p.z;
  }

  size_t axisIndex(char axis) const
  {
    if (axis == 'x') {
      return 0;
    }
    if (axis == 'y') {
      return 1;
    }
    return 2;
  }

  std::string pathLineSpec(size_t point) const
  {
    static const std::array<std::string, asr::kNum3dPoints> specs{"k:", "r-", "b-", "m-", "g-"};
    return specs[point];
  }

  std::string state_topic_;
  int draw_period_ms_;
  int max_history_size_;
  int trail_samples_;
  int body_snapshot_stride_;
  bool follow_current_body_;
  double view_half_range_;

  std::mutex mutex_;
  ControllerState3D latest_state_{};
  bool has_state_{false};
  bool has_initial_body_{false};
  std::array<asr::Vec3, asr::kNum3dPoints> initial_points_{};

  std::vector<double> time_history_;
  std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> body_history_;

  matplot::figure_handle trajectory_figure_;
  matplot::axes_handle trajectory_axes_;
  matplot::axes_handle trajectory_legend_axes_;
  matplot::axes_handle plane_legend_axes_;
  bool trajectory_legend_initialized_{false};
  bool plane_legend_initialized_{false};
  matplot::line_handle head_points_;
  matplot::line_handle joint1_path_;
  matplot::line_handle initial_body_;
  matplot::line_handle current_body_;
  std::vector<matplot::line_handle> snapshot_handles_;
  bool trajectory_initialized_{false};

  matplot::axes_handle xy_axes_;
  matplot::axes_handle xz_axes_;
  matplot::axes_handle yz_axes_;
  PlanePathPanel xy_panel_;
  PlanePathPanel xz_panel_;
  PlanePathPanel yz_panel_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeControllerVisualizer3DNode>());
  rclcpp::shutdown();
  return 0;
}

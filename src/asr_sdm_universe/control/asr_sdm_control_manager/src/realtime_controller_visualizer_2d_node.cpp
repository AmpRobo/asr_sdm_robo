#include "third_party/matplotlibcpp.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace plt = matplotlibcpp;
using namespace std::chrono_literals;

namespace
{

constexpr double pi_value = 3.14159265358979323846;

struct Point
{
  double x;
  double y;
};

struct ControllerState
{
  double t;
  double head_x;
  double head_y;
  double head_psi;
  double cmd_v;
  double cmd_omega;
  std::array<double, 3> phi;
  std::array<double, 3> phi_dot;
  std::array<double, 4> theta_dot;
};

std::array<Point, 5> body_points(const ControllerState & state, double link_length)
{
  const std::array<double, 4> theta{
    state.head_psi,
    state.head_psi + state.phi[0],
    state.head_psi + state.phi[0] + state.phi[1],
    state.head_psi + state.phi[0] + state.phi[1] + state.phi[2]};

  std::array<Point, 5> points{};
  points[0] = {state.head_x, state.head_y};

  for (size_t i = 0; i < theta.size(); ++i) {
    points[i + 1] = {
      points[i].x - link_length * std::cos(theta[i]),
      points[i].y - link_length * std::sin(theta[i])};
  }

  return points;
}

}  // namespace

class RealtimeControllerVisualizerNode : public rclcpp::Node
{
public:
  RealtimeControllerVisualizerNode()
  : Node("realtime_controller_visualizer")
  {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/asr_sdm/controller_state");
    link_length_ = this->declare_parameter<double>("link_length", 0.25);
    draw_period_ms_ = this->declare_parameter<int>("draw_period_ms", 100);
    max_history_size_ = this->declare_parameter<int>("max_history_size", 3000);

    sub_state_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      state_topic_, rclcpp::QoS(100),
      std::bind(&RealtimeControllerVisualizerNode::onControllerState, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(draw_period_ms_),
      std::bind(&RealtimeControllerVisualizerNode::onDrawTimer, this));

    plt::figure_size(1000, 760);
    figure_number_ = 1;
    RCLCPP_INFO(this->get_logger(), "Realtime visualizer started: state=%s", state_topic_.c_str());
  }

private:
  void onControllerState(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 16) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore controller state: expected 16 values, got %zu", msg->data.size());
      return;
    }

    ControllerState state;
    state.t = msg->data[0];
    state.head_x = msg->data[1];
    state.head_y = msg->data[2];
    state.head_psi = msg->data[3];
    state.cmd_v = msg->data[4];
    state.cmd_omega = msg->data[5];
    for (size_t i = 0; i < state.phi.size(); ++i) {
      state.phi[i] = msg->data[6 + i];
      state.phi_dot[i] = msg->data[9 + i];
    }
    for (size_t i = 0; i < state.theta_dot.size(); ++i) {
      state.theta_dot[i] = msg->data[12 + i];
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_ = state;
    has_state_ = true;
  }

  void onDrawTimer()
  {
    if (figure_number_ > 0 && !plt::fignum_exists(figure_number_)) {
      RCLCPP_INFO(this->get_logger(), "Visualizer window closed, shutting down node.");
      rclcpp::shutdown();
      return;
    }

    ControllerState state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_state_) {
        return;
      }
      state = latest_state_;
    }

    appendSample(state);
    drawFrame();
  }

  void appendSample(const ControllerState & state)
  {
    const auto points = body_points(state, link_length_);

    time_history_.push_back(state.t);
    x_head_.push_back(points[0].x);
    y_head_.push_back(points[0].y);
    xj1_.push_back(points[1].x);
    yj1_.push_back(points[1].y);
    xj2_.push_back(points[2].x);
    yj2_.push_back(points[2].y);
    xj3_.push_back(points[3].x);
    yj3_.push_back(points[3].y);
    x_tail_.push_back(points[4].x);
    y_tail_.push_back(points[4].y);

    x_body_ = {points[0].x, points[1].x, points[2].x, points[3].x, points[4].x};
    y_body_ = {points[0].y, points[1].y, points[2].y, points[3].y, points[4].y};

    if (x0_.empty()) {
      x0_ = x_body_;
      y0_ = y_body_;
    }

    phi1_history_.push_back(state.phi[0]);
    phi2_history_.push_back(state.phi[1]);
    phi3_history_.push_back(state.phi[2]);
    v_history_.push_back(state.cmd_v);
    omega_history_.push_back(state.cmd_omega);

    trimHistory();
  }

  void trimHistory()
  {
    if (max_history_size_ <= 0 || time_history_.size() <= static_cast<size_t>(max_history_size_)) {
      return;
    }

    eraseFront(time_history_);
    eraseFront(x_head_);
    eraseFront(y_head_);
    eraseFront(xj1_);
    eraseFront(yj1_);
    eraseFront(xj2_);
    eraseFront(yj2_);
    eraseFront(xj3_);
    eraseFront(yj3_);
    eraseFront(x_tail_);
    eraseFront(y_tail_);
    eraseFront(phi1_history_);
    eraseFront(phi2_history_);
    eraseFront(phi3_history_);
    eraseFront(v_history_);
    eraseFront(omega_history_);
  }

  void eraseFront(std::vector<double> & values) const
  {
    const size_t extra = values.size() - static_cast<size_t>(max_history_size_);
    values.erase(values.begin(), values.begin() + extra);
  }

  void drawFrame()
  {
    plt::clf();

    plt::subplot2grid(2, 2, 0, 0);
    plt::named_plot("Head", x_head_, y_head_, "b-");
    plt::named_plot("Joint 1", xj1_, yj1_, "r--");
    plt::named_plot("Joint 2", xj2_, yj2_, "m-.");
    plt::named_plot("Joint 3", xj3_, yj3_, "c:");
    plt::named_plot("Tail", x_tail_, y_tail_, "g-");
    plt::named_plot("current body", x_body_, y_body_, "ko-");
    plt::scatter(x0_, y0_, 18.0, {{"label", "initial body"}});
    plt::title("Realtime body trajectories");
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::xlim(-1., 1.);
    plt::ylim(-1., 1.);
    plt::axis("equal");
    plt::grid(true);
    plt::legend({{"loc", "lower left"}, {"fontsize", "x-small"}});

    plt::subplot2grid(2, 2, 0, 1);
    plt::named_plot("current body", x_body_, y_body_, "ko-");
    plt::title("Current body shape");
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::xlim(-0.8, 0.8);
    plt::ylim(-1.0, 0.4);
    plt::axis("equal");
    plt::grid(true);
    plt::legend();

    plt::subplot2grid(2, 2, 1, 0);
    plt::named_plot("phi1", time_history_, phi1_history_, "b-");
    plt::named_plot("phi2", time_history_, phi2_history_, "r--");
    plt::named_plot("phi3", time_history_, phi3_history_, "m-.");
    plt::title("Joint angles");
    plt::xlabel("time [s]");
    plt::ylabel("phi [rad]");
    if (!time_history_.empty()) {
      const double t_max = time_history_.back();
      plt::xlim(std::max(0.0, t_max - 20.0), std::max(20.0, t_max));
    }
    plt::ylim(-1.5, 1.5);
    plt::grid(true);
    plt::legend();

    plt::subplot2grid(2, 2, 1, 1);
    plt::named_plot("v [m/s]", time_history_, v_history_, "b-");
    plt::named_plot("omega [rad/s]", time_history_, omega_history_, "r--");
    plt::title("Head command profile");
    plt::xlabel("time [s]");
    if (!time_history_.empty()) {
      const double t_max = time_history_.back();
      plt::xlim(std::max(0.0, t_max - 20.0), std::max(20.0, t_max));
    }
    plt::ylim(-1., 1.);
    plt::grid(true);
    plt::legend();

    plt::subplots_adjust({{"hspace", 0.42}, {"wspace", 0.28}});
    plt::pause(0.001);
  }

  std::string state_topic_;
  double link_length_;
  int draw_period_ms_;
  int max_history_size_;
  long figure_number_{-1};
  std::mutex mutex_;
  ControllerState latest_state_{};
  bool has_state_{false};

  std::vector<double> time_history_;
  std::vector<double> x_head_;
  std::vector<double> y_head_;
  std::vector<double> xj1_;
  std::vector<double> yj1_;
  std::vector<double> xj2_;
  std::vector<double> yj2_;
  std::vector<double> xj3_;
  std::vector<double> yj3_;
  std::vector<double> x_tail_;
  std::vector<double> y_tail_;
  std::vector<double> x0_;
  std::vector<double> y0_;
  std::vector<double> x_body_;
  std::vector<double> y_body_;
  std::vector<double> phi1_history_;
  std::vector<double> phi2_history_;
  std::vector<double> phi3_history_;
  std::vector<double> v_history_;
  std::vector<double> omega_history_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealtimeControllerVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}

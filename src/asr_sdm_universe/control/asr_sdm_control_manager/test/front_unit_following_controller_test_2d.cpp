#include "asr_sdm_controller/front_unit_following_controller_2d.hpp"
#include "asr_sdm_controller/robot_model.hpp"
#include "third_party/matplotlibcpp.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace plt = matplotlibcpp;

namespace {

constexpr double pi_value = 3.14159265358979323846;

struct Point {
  double x;
  double y;
};

struct Pose {
  double x;
  double y;
  double psi;
};

struct VelocityCommand {
  double v;
  double omega;
  size_t steps;
};

double saturate(double value, double limit)
{
  return std::max(-limit, std::min(value, limit));
}

std::array<Point, 5> body_points(
  const Pose & head, const std::vector<double> & phi, double link_length)
{
  const std::array<double, 4> theta{
    head.psi,
    head.psi + phi[0],
    head.psi + phi[0] + phi[1],
    head.psi + phi[0] + phi[1] + phi[2]};

  std::array<Point, 5> points{};
  points[0] = {head.x, head.y};

  for (size_t i = 0; i < theta.size(); ++i) {
    points[i + 1] = {
      points[i].x - link_length * std::cos(theta[i]),
      points[i].y - link_length * std::sin(theta[i])};
  }

  return points;
}

}  // namespace

int main()
{
  asr::RobotParameters params;
  params.link_length = 0.25;
  params.screw_radius = 0.075;
  params.alpha = {-M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0};

  constexpr double dt = 0.02;
  constexpr double phi_dot_limit = 2.0;
  constexpr double phi_limit = 0.85 * pi_value;
  constexpr size_t draw_stride = 12;

  asr::FrontUnitFollowingController controller(params);
  asr::RobotModel robot_model(params);

  Pose head{0., 0., pi_value / 2.};
  asr::JointState state;
  state.phi = {0., 0., 0.};
  double t = 0.;

  std::vector<double> x_head;
  std::vector<double> y_head;
  std::vector<double> xj1;
  std::vector<double> yj1;
  std::vector<double> xj2;
  std::vector<double> yj2;
  std::vector<double> xj3;
  std::vector<double> yj3;
  std::vector<double> x_tail;
  std::vector<double> y_tail;
  std::vector<double> x0;
  std::vector<double> y0;
  std::vector<double> x_body;
  std::vector<double> y_body;
  std::vector<double> time_history;
  std::vector<double> phi1_history;
  std::vector<double> phi2_history;
  std::vector<double> phi3_history;
  std::vector<double> v_history;
  std::vector<double> omega_history;

  auto append_sample = [&](double v, double omega) {
    const auto points = body_points(head, state.phi, params.link_length);

    x_head.push_back(points[0].x);
    y_head.push_back(points[0].y);
    xj1.push_back(points[1].x);
    yj1.push_back(points[1].y);
    xj2.push_back(points[2].x);
    yj2.push_back(points[2].y);
    xj3.push_back(points[3].x);
    yj3.push_back(points[3].y);
    x_tail.push_back(points[4].x);
    y_tail.push_back(points[4].y);

    x_body = {points[0].x, points[1].x, points[2].x, points[3].x, points[4].x};
    y_body = {points[0].y, points[1].y, points[2].y, points[3].y, points[4].y};

    if (x0.empty()) {
      x0 = x_body;
      y0 = y_body;
    }

    time_history.push_back(t);
    phi1_history.push_back(state.phi[0]);
    phi2_history.push_back(state.phi[1]);
    phi3_history.push_back(state.phi[2]);
    v_history.push_back(v);
    omega_history.push_back(omega);
  };

  auto step_simulation = [&](const asr::HeadCommand & cmd) {
    const asr::JointVelocity joint_vel = controller.computeJointVelocity(cmd, state);
    robot_model.computeScrewVelocity(cmd, state, joint_vel);

    for (size_t i = 0; i < state.phi.size(); ++i) {
      state.phi[i] = saturate(
        state.phi[i] + saturate(joint_vel.phi_dot[i], phi_dot_limit) * dt, phi_limit);
    }

    head.x += cmd.linear_velocity * std::cos(head.psi) * dt;
    head.y += cmd.linear_velocity * std::sin(head.psi) * dt;
    head.psi += cmd.angular_velocity * dt;
    t += dt;

    append_sample(cmd.linear_velocity, cmd.angular_velocity);
  };

  auto draw_frame = [&]() {
    plt::clf();

    plt::subplot2grid(2, 2, 0, 0);
    plt::named_plot("Head", x_head, y_head, "b-");
    plt::named_plot("Joint 1", xj1, yj1, "r--");
    plt::named_plot("Joint 2", xj2, yj2, "m-.");
    plt::named_plot("Joint 3", xj3, yj3, "c:");
    plt::named_plot("Tail", x_tail, y_tail, "g-");
    plt::named_plot("current body", x_body, y_body, "ko-");
    plt::scatter(x0, y0, 18.0, {{"label", "initial body"}});
    plt::title("Body trajectories - follow-the-leader controller");
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::xlim(-1., 1.);
    plt::ylim(-1., 1.);
    plt::axis("equal");
    plt::grid(true);
    plt::legend({{"loc", "lower left"}, {"fontsize", "x-small"}});

    plt::subplot2grid(2, 2, 0, 1);
    plt::named_plot("current body", x_body, y_body, "ko-");
    plt::title("Current body shape");
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::xlim(-0.6, 0.6);
    plt::ylim(-0.8, 0.2);
    plt::axis("equal");
    plt::grid(true);
    plt::legend();

    plt::subplot2grid(2, 2, 1, 0);
    plt::named_plot("phi1", time_history, phi1_history, "b-");
    plt::named_plot("phi2", time_history, phi2_history, "r--");
    plt::named_plot("phi3", time_history, phi3_history, "m-.");
    plt::title("Joint angles - controller output");
    plt::xlabel("time [s]");
    plt::ylabel("phi [rad]");
    plt::xlim(0., 18.);
    plt::ylim(-1.5, 1.5);
    plt::grid(true);
    plt::legend();

    plt::subplot2grid(2, 2, 1, 1);
    plt::named_plot("v [m/s]", time_history, v_history, "b-");
    plt::named_plot("omega [rad/s]", time_history, omega_history, "r--");
    plt::title("Head command profile");
    plt::xlabel("time [s]");
    plt::xlim(0., 18.);
    plt::ylim(-1., 1.);
    plt::grid(true);
    plt::legend();

    plt::subplots_adjust({{"hspace", 0.42}, {"wspace", 0.28}});
    plt::pause(0.001);
  };

  const std::vector<VelocityCommand> commands{
    {0.12, 0.00, 120}, {0.10, 0.50, 160}, {0.12, 0.00, 100},
    {0.10, -0.50, 160}, {0.10, 0.45, 140}, {0.10, -0.55, 140}};

  append_sample(0., 0.);
  plt::figure_size(1000, 760);
  draw_frame();

  constexpr auto frame_period = std::chrono::milliseconds(200);
  auto next_frame = std::chrono::steady_clock::now();

  for (const auto & command : commands) {
    for (size_t i = 0; i < command.steps; ++i) {
      const asr::HeadCommand head_cmd{command.v, command.omega};
      step_simulation(head_cmd);

      if ((i + 1) % draw_stride == 0 || i + 1 == command.steps) {
        draw_frame();
        next_frame += frame_period;
        const auto now = std::chrono::steady_clock::now();
        if (next_frame > now) {
          std::this_thread::sleep_until(next_frame);
        } else {
          next_frame = now;
        }
      }
    }
  }

  const asr::HeadCommand final_cmd{0., 0.};
  const auto final_jv = controller.computeJointVelocity(final_cmd, state);
  const auto final_sv = robot_model.computeScrewVelocity(final_cmd, state, final_jv);

  std::cout << "Final joint phi: ";
  for (double p : state.phi) {
    std::cout << p << " ";
  }
  std::cout << std::endl;

  std::cout << "Final joint velocities:" << std::endl;
  for (double v : final_jv.phi_dot) {
    std::cout << "  " << v << std::endl;
  }

  std::cout << "Final screw velocities:" << std::endl;
  for (double v : final_sv.theta_dot) {
    std::cout << "  " << v << std::endl;
  }

  plt::show();
  return 0;
}

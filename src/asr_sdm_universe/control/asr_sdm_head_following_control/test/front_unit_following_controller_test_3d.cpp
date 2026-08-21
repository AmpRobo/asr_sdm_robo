#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"

#include <matplot/matplot.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace plt = matplot;

namespace {

constexpr double pi_value = 3.14159265358979323846;
constexpr size_t kJoint1BodyPointIndex = 1;

struct OfflineHeadCommand3D
{
  double linear_velocity;
  double pitch_rate;
  double yaw_rate;
  size_t steps;
};

struct SpatialRms
{
  double xyz;
  double xy;
  double z;
};

struct SpatialPathLagRms
{
  double xyz;
  double xy;
  double z;
  size_t count;
};

std::vector<OfflineHeadCommand3D> make_commands()
{
  return {
    {0.10, 0.18, 0.00, 120}, {0.10, -0.14, 0.00, 120},
    {0.10, 0.00, 0.35, 140}, {0.10, 0.12, 0.30, 160},
    {0.10, -0.12, -0.30, 160}, {0.12, 0.00, 0.00, 100}};
}

asr_sdm_control_msgs::msg::RobotCommand makeRobotCommand(
  double linear_velocity, double pitch_rate, double yaw_rate)
{
  asr_sdm_control_msgs::msg::RobotCommand cmd;
  cmd.vel.linear.x = linear_velocity;
  cmd.vel.angular.y = pitch_rate;
  cmd.vel.angular.z = yaw_rate;
  return cmd;
}

bool has_nonzero_pitch_command(const std::vector<OfflineHeadCommand3D> & commands)
{
  return std::any_of(commands.begin(), commands.end(), [](const auto & cmd) {
    return std::abs(cmd.pitch_rate) > 1.0e-12;
  });
}

bool has_nonzero_yaw_command(const std::vector<OfflineHeadCommand3D> & commands)
{
  return std::any_of(commands.begin(), commands.end(), [](const auto & cmd) {
    return std::abs(cmd.yaw_rate) > 1.0e-12;
  });
}

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
  const plt::line_handle & handle, const std::vector<double> & x, const std::vector<double> & y,
  const std::vector<double> & z)
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

void append_joint_history(
  const std::array<double, asr::kNum3dJointDofs> & theta,
  std::array<std::vector<double>, asr::kNum3dJointDofs> & theta_history)
{
  for (size_t i = 0; i < theta.size(); ++i) {
    theta_history[i].push_back(theta[i]);
  }
}

double rms_lagged_difference(
  const std::vector<double> & reference, const std::vector<double> & follower, size_t lag_steps)
{
  const size_t n = std::min(reference.size(), follower.size());
  if (n <= lag_steps) {
    return 0.0;
  }

  double sum = 0.0;
  size_t count = 0;
  for (size_t i = lag_steps; i < n; ++i) {
    const double error = follower[i] - reference[i - lag_steps];
    sum += error * error;
    ++count;
  }
  return std::sqrt(sum / static_cast<double>(count));
}

SpatialRms lagged_spatial_rms(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  size_t reference_point, size_t follower_point, size_t lag_steps, size_t start_index = 0)
{
  if (samples.size() <= lag_steps) {
    return {0.0, 0.0, 0.0};
  }

  double xyz_sum = 0.0;
  double xy_sum = 0.0;
  double z_sum = 0.0;
  size_t count = 0;
  const size_t first = std::max(lag_steps, start_index);
  for (size_t i = first; i < samples.size(); ++i) {
    const asr::Vec3 error = samples[i][follower_point] - samples[i - lag_steps][reference_point];
    xyz_sum += asr::dot(error, error);
    xy_sum += error.x * error.x + error.y * error.y;
    z_sum += error.z * error.z;
    ++count;
  }
  if (count == 0) {
    return {0.0, 0.0, 0.0};
  }
  return {
    std::sqrt(xyz_sum / static_cast<double>(count)),
    std::sqrt(xy_sum / static_cast<double>(count)),
    std::sqrt(z_sum / static_cast<double>(count))};
}

std::vector<double> lagged_spatial_error_history(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  size_t reference_point, size_t follower_point, size_t lag_steps)
{
  std::vector<double> errors(samples.size(), 0.0);
  for (size_t i = lag_steps; i < samples.size(); ++i) {
    errors[i] = asr::norm(samples[i][follower_point] - samples[i - lag_steps][reference_point]);
  }
  return errors;
}

double max_lagged_spatial_error(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  size_t reference_point, size_t follower_point, size_t lag_steps)
{
  if (samples.size() <= lag_steps) {
    return 0.0;
  }

  double max_error = 0.0;
  for (size_t i = lag_steps; i < samples.size(); ++i) {
    max_error = std::max(
      max_error,
      asr::norm(samples[i][follower_point] - samples[i - lag_steps][reference_point]));
  }
  return max_error;
}

std::vector<double> cumulative_path_distance(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples, size_t point)
{
  std::vector<double> distances(samples.size(), 0.0);
  for (size_t i = 1; i < samples.size(); ++i) {
    distances[i] = distances[i - 1] + asr::norm(samples[i][point] - samples[i - 1][point]);
  }
  return distances;
}

asr::Vec3 interpolate_point_at_distance(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  const std::vector<double> & distances, size_t point, double target_distance, size_t upper_index)
{
  size_t hi = std::min(upper_index, distances.size() - 1);
  while (hi > 0 && distances[hi - 1] >= target_distance) {
    --hi;
  }
  const size_t lo = hi > 0 ? hi - 1 : 0;
  const double span = distances[hi] - distances[lo];
  if (span < 1.0e-12) {
    return samples[hi][point];
  }

  const double alpha = std::clamp((target_distance - distances[lo]) / span, 0.0, 1.0);
  return samples[lo][point] * (1.0 - alpha) + samples[hi][point] * alpha;
}

SpatialPathLagRms path_lagged_spatial_rms(
  const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples,
  size_t reference_point, size_t follower_point, double lag_distance, size_t start_index = 1)
{
  if (samples.size() < 2 || lag_distance <= 0.0) {
    return {0.0, 0.0, 0.0, 0};
  }

  const std::vector<double> distances = cumulative_path_distance(samples, reference_point);
  double xyz_sum = 0.0;
  double xy_sum = 0.0;
  double z_sum = 0.0;
  size_t count = 0;

  for (size_t i = std::max<size_t>(1, start_index); i < samples.size(); ++i) {
    if (distances[i] < lag_distance) {
      continue;
    }

    const asr::Vec3 reference = interpolate_point_at_distance(
      samples, distances, reference_point, distances[i] - lag_distance, i);
    const asr::Vec3 error = samples[i][follower_point] - reference;
    xyz_sum += asr::dot(error, error);
    xy_sum += error.x * error.x + error.y * error.y;
    z_sum += error.z * error.z;
    ++count;
  }

  if (count == 0) {
    return {0.0, 0.0, 0.0, 0};
  }

  return {
    std::sqrt(xyz_sum / static_cast<double>(count)),
    std::sqrt(xy_sum / static_cast<double>(count)),
    std::sqrt(z_sum / static_cast<double>(count)),
    count};
}

double max_abs_z_extent(const std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> & samples)
{
  double max_z = 0.0;
  for (const auto & sample : samples) {
    for (const auto & p : sample) {
      max_z = std::max(max_z, std::abs(p.z));
    }
  }
  return max_z;
}

std::array<double, asr::kNum3dJoints> adjacent_axis_angles(
  const std::array<asr::Mat3, asr::kNum3dLinks> & link_frames)
{
  std::array<double, asr::kNum3dJoints> angles{};
  const auto axes = asr::linkAxes(link_frames);
  for (size_t i = 0; i < angles.size(); ++i) {
    angles[i] = std::acos(std::clamp(asr::dot(axes[i], axes[i + 1]), -1.0, 1.0));
  }
  return angles;
}

}  // namespace

int main()
{
  constexpr double link_length = 0.25;
  constexpr double dt = 0.02;
  constexpr double joint_rate_limit = 2.0;
  constexpr double joint_limit = 0.85 * pi_value;
  constexpr double damping = 0.02;
  constexpr double max_curvature = 1.2;
  constexpr double curvature_velocity_epsilon = 1.0e-3;
  constexpr size_t draw_stride = 12;
  constexpr size_t trail_samples = 360;
  constexpr size_t body_snapshot_stride = 80;
  constexpr double fixed_lag_reference_velocity = 0.10;
  const size_t fixed_lag_steps = static_cast<size_t>(
    std::round(link_length / (fixed_lag_reference_velocity * dt)));

  asr::FrontUnitController3DParameters params;
  params.link_length = link_length;
  params.damping = damping;
  params.joint_rate_limit = joint_rate_limit;
  params.joint_limit = joint_limit;
  params.max_curvature = max_curvature;
  params.curvature_velocity_epsilon = curvature_velocity_epsilon;
  const asr::FrontUnitFollowingController3D controller(params);
  asr::SimulationState3D state = controller.makeInitialState();
  asr::JointVelocity3D joint_velocity{};

  std::array<std::vector<double>, asr::kNum3dJointDofs> theta_history{};
  std::vector<double> time_history;
  std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> body_history;
  std::vector<double> v_history;
  std::vector<double> pitch_rate_history;
  std::vector<double> yaw_rate_history;
  double max_observed_curvature = 0.0;
  std::array<double, asr::kNum3dJoints> max_adjacent_axis_angle{};

  const auto initial_points = state.body_points;

  auto append_sample = [&](double v, double pitch_rate, double yaw_rate) {
    time_history.push_back(state.time);
    body_history.push_back(state.body_points);
    append_joint_history(state.joints.theta, theta_history);
    v_history.push_back(v);
    pitch_rate_history.push_back(pitch_rate);
    yaw_rate_history.push_back(yaw_rate);
    const auto angles = adjacent_axis_angles(state.link_frames);
    for (size_t i = 0; i < angles.size(); ++i) {
      max_adjacent_axis_angle[i] = std::max(max_adjacent_axis_angle[i], angles[i]);
    }
  };

  auto trajectory_figure = plt::figure(true);
  trajectory_figure->width(1000);
  trajectory_figure->height(760);
  auto trajectory_axes = trajectory_figure->current_axes();

  auto initial_body_values = values_for_body(initial_points);
  const size_t max_snapshot_count = trail_samples / body_snapshot_stride + 1;
  std::vector<plt::line_handle> snapshot_handles;
  snapshot_handles.reserve(max_snapshot_count);
  bool trajectory_initialized = false;
  plt::line_handle head_points;
  plt::line_handle joint1_path;
  plt::line_handle initial_body;
  plt::line_handle current_body;

  auto draw_frame = [&]() {
    plt::figure(trajectory_figure);

    if (!trajectory_initialized) {
      plt::hold(trajectory_axes, true);

      head_points = plt::scatter3(
        trajectory_axes, values_for_point(body_history, 0, 'x', trail_samples),
        values_for_point(body_history, 0, 'y', trail_samples),
        values_for_point(body_history, 0, 'z', trail_samples), ".");
      head_points->color("k");
      head_points->marker_size(2);

      joint1_path = plt::plot3(
        trajectory_axes, values_for_point(body_history, 1, 'x', trail_samples),
        values_for_point(body_history, 1, 'y', trail_samples),
        values_for_point(body_history, 1, 'z', trail_samples), "r-");
      joint1_path->line_width(3.0);

      for (size_t i = 0; i < max_snapshot_count; ++i) {
        auto snapshot = plt::plot3(
          trajectory_axes, std::vector<double>{}, std::vector<double>{}, std::vector<double>{}, ".-");
        snapshot->color("k");
        snapshot->line_width(0.5);
        snapshot->marker_size(3);
        snapshot_handles.push_back(snapshot);
      }

      initial_body = plt::plot3(
        trajectory_axes, initial_body_values[0], initial_body_values[1], initial_body_values[2], "--s");
      initial_body->color("k");
      initial_body->line_width(1.0);
      initial_body->marker_size(6);

      const auto body_values = values_for_body(state.body_points);
      current_body = plt::plot3(trajectory_axes, body_values[0], body_values[1], body_values[2], "-ok");
      current_body->line_width(3.0);
      current_body->marker_size(7);

      plt::title(trajectory_axes, "3D follow-the-leader body simulation");
      plt::xlabel(trajectory_axes, "x [m]");
      plt::ylabel(trajectory_axes, "y [m]");
      plt::zlabel(trajectory_axes, "z [m]");
      plt::view(45, 28);
      plt::grid(trajectory_axes, true);
      plt::legend(
        {joint1_path, initial_body, current_body},
        {"joint1 path", "initial body", "current body"});
      trajectory_initialized = true;
    }

    update_line3(
      head_points, values_for_point(body_history, 0, 'x', trail_samples),
      values_for_point(body_history, 0, 'y', trail_samples),
      values_for_point(body_history, 0, 'z', trail_samples));
    update_line3(
      joint1_path, values_for_point(body_history, 1, 'x', trail_samples),
      values_for_point(body_history, 1, 'y', trail_samples),
      values_for_point(body_history, 1, 'z', trail_samples));

    const size_t snapshot_start = body_history.size() > trail_samples ? body_history.size() - trail_samples : 0;
    size_t handle_index = 0;
    for (size_t sample = snapshot_start;
      sample < body_history.size() && handle_index < snapshot_handles.size();
      sample += body_snapshot_stride, ++handle_index)
    {
      const auto snapshot_values = values_for_body(body_history[sample]);
      update_line3(
        snapshot_handles[handle_index], snapshot_values[0], snapshot_values[1], snapshot_values[2]);
      snapshot_handles[handle_index]->visible(true);
    }
    for (; handle_index < snapshot_handles.size(); ++handle_index) {
      snapshot_handles[handle_index]->visible(false);
    }

    const auto body_values = values_for_body(state.body_points);
    update_line3(current_body, body_values[0], body_values[1], body_values[2]);

    const auto limits = body_history_limits(body_history, initial_points, trail_samples, 0.08);
    plt::xrange(trajectory_axes, limits[0]);
    plt::yrange(trajectory_axes, limits[1]);
    trajectory_axes->z_axis().limits(limits[2]);
    trajectory_axes->z_axis().limits_mode_auto(false);

    trajectory_figure->draw();
  };

  auto draw_analysis = [&]() {
    auto analysis_figure = plt::figure(true);
    analysis_figure->width(1000);
    analysis_figure->height(760);
    plt::tiledlayout(2, 2);

    auto pitch_axes = plt::nexttile();
    plt::plot(pitch_axes, time_history, theta_history[asr::pitchIndex(0)], "b-");
    plt::hold(pitch_axes, true);
    plt::plot(pitch_axes, time_history, theta_history[asr::pitchIndex(1)], "r--");
    plt::plot(pitch_axes, time_history, theta_history[asr::pitchIndex(2)], "m-.");
    plt::title(pitch_axes, "Pitch joint angles");
    plt::xlabel(pitch_axes, "time [s]");
    plt::ylabel(pitch_axes, "pitch [rad]");
    plt::grid(pitch_axes, true);
    plt::legend(pitch_axes, {"joint1 pitch", "joint2 pitch", "joint3 pitch"});

    auto yaw_axes = plt::nexttile();
    plt::plot(yaw_axes, time_history, theta_history[asr::yawIndex(0)], "b-");
    plt::hold(yaw_axes, true);
    plt::plot(yaw_axes, time_history, theta_history[asr::yawIndex(1)], "r--");
    plt::plot(yaw_axes, time_history, theta_history[asr::yawIndex(2)], "m-.");
    plt::title(yaw_axes, "Yaw joint angles");
    plt::xlabel(yaw_axes, "time [s]");
    plt::ylabel(yaw_axes, "yaw [rad]");
    plt::grid(yaw_axes, true);
    plt::legend(yaw_axes, {"joint1 yaw", "joint2 yaw", "joint3 yaw"});

    auto command_axes = plt::nexttile();
    plt::plot(command_axes, time_history, v_history, "b-");
    plt::hold(command_axes, true);
    plt::plot(command_axes, time_history, pitch_rate_history, "r--");
    plt::plot(command_axes, time_history, yaw_rate_history, "m-.");
    plt::title(command_axes, "Head command profile");
    plt::xlabel(command_axes, "time [s]");
    plt::grid(command_axes, true);
    plt::legend(command_axes, {"v [m/s]", "pitch rate [rad/s]", "yaw rate [rad/s]"});

    auto tracking_axes = plt::nexttile();
    plt::plot(tracking_axes, time_history, lagged_spatial_error_history(body_history, 1, 2, fixed_lag_steps), "r--");
    plt::hold(tracking_axes, true);
    plt::plot(tracking_axes, time_history, lagged_spatial_error_history(body_history, 1, 3, 2 * fixed_lag_steps), "m-.");
    plt::plot(tracking_axes, time_history, lagged_spatial_error_history(body_history, 1, 4, 3 * fixed_lag_steps), "g-");
    plt::title(tracking_axes, "Spatial lag error relative to joint1");
    plt::xlabel(tracking_axes, "time [s]");
    plt::ylabel(tracking_axes, "distance [m]");
    plt::grid(tracking_axes, true);
    plt::legend(tracking_axes, {"joint2 <- joint1", "joint3 <- joint1", "tail <- joint1"});

    analysis_figure->draw();

    auto path_figure = plt::figure(true);
    path_figure->width(1100);
    path_figure->height(850);
    plt::tiledlayout(2, 2);

    const auto joint1_x = values_for_point(body_history, kJoint1BodyPointIndex, 'x');
    const auto joint1_y = values_for_point(body_history, kJoint1BodyPointIndex, 'y');
    const auto joint1_z = values_for_point(body_history, kJoint1BodyPointIndex, 'z');
    const auto current_body_values = values_for_body(state.body_points);

    auto path3d_axes = plt::nexttile();
    auto joint1_3d = plt::plot3(path3d_axes, joint1_x, joint1_y, joint1_z, "r-");
    joint1_3d->line_width(2.5);
    plt::hold(path3d_axes, true);
    auto current_body_3d = plt::plot3(
      path3d_axes, current_body_values[0], current_body_values[1], current_body_values[2], "-ok");
    current_body_3d->line_width(2.0);
    current_body_3d->marker_size(5);
    plt::title(path3d_axes, "3D joint1 path");
    plt::xlabel(path3d_axes, "x [m]");
    plt::ylabel(path3d_axes, "y [m]");
    plt::zlabel(path3d_axes, "z [m]");
    plt::view(45, 28);
    plt::grid(path3d_axes, true);
    plt::legend(path3d_axes, {"joint1 path (link1-link2)", "current body"});

    auto xy_axes = plt::nexttile();
    auto joint1_xy = plt::plot(xy_axes, joint1_x, joint1_y, "r-");
    joint1_xy->line_width(2.5);
    plt::hold(xy_axes, true);
    auto current_body_xy = plt::plot(xy_axes, current_body_values[0], current_body_values[1], "-ok");
    current_body_xy->line_width(2.0);
    current_body_xy->marker_size(5);
    plt::title(xy_axes, "XY joint1 path");
    plt::xlabel(xy_axes, "x [m]");
    plt::ylabel(xy_axes, "y [m]");
    plt::grid(xy_axes, true);
    plt::legend(xy_axes, {"joint1 path (link1-link2)", "current body"});

    auto xz_axes = plt::nexttile();
    auto joint1_xz = plt::plot(xz_axes, joint1_x, joint1_z, "r-");
    joint1_xz->line_width(2.5);
    plt::hold(xz_axes, true);
    auto current_body_xz = plt::plot(xz_axes, current_body_values[0], current_body_values[2], "-ok");
    current_body_xz->line_width(2.0);
    current_body_xz->marker_size(5);
    plt::title(xz_axes, "XZ joint1 path");
    plt::xlabel(xz_axes, "x [m]");
    plt::ylabel(xz_axes, "z [m]");
    plt::grid(xz_axes, true);
    plt::legend(xz_axes, {"joint1 path (link1-link2)", "current body"});

    auto yz_axes = plt::nexttile();
    auto joint1_yz = plt::plot(yz_axes, joint1_y, joint1_z, "r-");
    joint1_yz->line_width(2.5);
    plt::hold(yz_axes, true);
    auto current_body_yz = plt::plot(yz_axes, current_body_values[1], current_body_values[2], "-ok");
    current_body_yz->line_width(2.0);
    current_body_yz->marker_size(5);
    plt::title(yz_axes, "YZ joint1 path");
    plt::xlabel(yz_axes, "y [m]");
    plt::ylabel(yz_axes, "z [m]");
    plt::grid(yz_axes, true);
    plt::legend(yz_axes, {"joint1 path (link1-link2)", "current body"});

    path_figure->draw();
  };

  auto step_simulation = [&](const OfflineHeadCommand3D & cmd) {
    const auto requested_cmd = makeRobotCommand(cmd.linear_velocity, cmd.pitch_rate, cmd.yaw_rate);
    const auto limited_cmd = controller.limitCommand(requested_cmd);
    joint_velocity = controller.step(limited_cmd, dt, state);
    append_sample(limited_cmd.vel.linear.x, limited_cmd.vel.angular.y, limited_cmd.vel.angular.z);
    const double speed = std::max(std::abs(limited_cmd.vel.linear.x), curvature_velocity_epsilon);
    max_observed_curvature = std::max(
      max_observed_curvature, std::hypot(limited_cmd.vel.angular.y, limited_cmd.vel.angular.z) / speed);
  };

  const std::vector<OfflineHeadCommand3D> commands = make_commands();

  std::cout << "Purpose: 3D front-unit following validation for Chapter 8 kinematic reference generation" << std::endl;
  std::cout << "Contains nonzero pitch command: "
            << (has_nonzero_pitch_command(commands) ? "yes" : "no") << std::endl;
  std::cout << "Contains nonzero yaw command: "
            << (has_nonzero_yaw_command(commands) ? "yes" : "no") << std::endl;
  std::cout << "Fixed-step lag reference velocity: " << fixed_lag_reference_velocity
            << " m/s, steps=" << fixed_lag_steps << ", lag=" << fixed_lag_steps * dt
            << "s" << std::endl;
  std::cout << "Max curvature limit: " << max_curvature << " 1/m" << std::endl;

  // === 回归测试：文档第 8 章局部近似方案（§8） ===

  // 8.1 常曲率圆弧稳态：相对 link1 的路径滞后误差 < 1mm
  {
    asr::FrontUnitController3DParameters arc_params = params;
    arc_params.max_curvature = 2.0;
    asr::FrontUnitFollowingController3D arc_controller(arc_params);
    asr::SimulationState3D arc_state = arc_controller.makeInitialState();
    constexpr double arc_v = 0.5;
    constexpr double arc_yaw_rate = 0.5;
    constexpr double arc_dt = 0.0025;
    constexpr size_t arc_steps = 8000;
    constexpr size_t steady_start = arc_steps / 2;
    const double arc_radius = arc_v / arc_yaw_rate;
    const double arc_lag = 2.0 * arc_radius * std::asin(link_length / (2.0 * arc_radius));
    const auto arc_cmd = makeRobotCommand(arc_v, 0.0, arc_yaw_rate);
    std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> arc_samples;
    arc_samples.reserve(arc_steps);

    for (size_t k = 0; k < arc_steps; ++k) {
      arc_controller.step(arc_cmd, arc_dt, arc_state);
      arc_samples.push_back(arc_state.body_points);
    }

    const auto lag2 = path_lagged_spatial_rms(arc_samples, 1, 2, arc_lag, steady_start);
    const auto lag3 = path_lagged_spatial_rms(arc_samples, 1, 3, 2.0 * arc_lag, steady_start);
    const auto lag4 = path_lagged_spatial_rms(arc_samples, 1, 4, 3.0 * arc_lag, steady_start);
    const double max_lag_rms = std::max({lag2.xyz, lag3.xyz, lag4.xyz});
    const bool arc_ok = max_lag_rms < 0.001;
    std::cout << "  8.1 Constant-curvature arc: lag_rms j2/j3/j4="
              << lag2.xyz << ", " << lag3.xyz << ", " << lag4.xyz
              << " " << (arc_ok ? "PASS" : "FAIL") << " (threshold 1mm)" << std::endl;
    if (!arc_ok) {
      return 1;
    }
  }

  // 8.2 直线归零：v=0.5 直线 2s，所有关节角 ≈ 0
  {
    asr::SimulationState3D straight_state = controller.makeInitialState();
    constexpr double straight_v = 0.5;
    constexpr double straight_dt = 0.02;
    constexpr size_t straight_steps = 100;
    const auto straight_cmd = makeRobotCommand(straight_v, 0.0, 0.0);

    for (size_t k = 0; k < straight_steps; ++k) {
      controller.step(straight_cmd, straight_dt, straight_state);
    }

    double max_theta_abs = 0.0;
    for (const auto & th : straight_state.joints.theta) {
      max_theta_abs = std::max(max_theta_abs, std::abs(th));
    }
    const bool straight_ok = max_theta_abs < 1e-6;
    std::cout << "  8.2 Straight-line zero: max|θ|=" << max_theta_abs
              << " " << (straight_ok ? "PASS" : "FAIL") << " (threshold 1e-6)" << std::endl;
    if (!straight_ok) {
      return 1;
    }
  }

  // 8.3 纯偏航/纯俯仰无串扰
  {
    // 纯偏航：只有 yaw 关节应有非零速率
    asr::SimulationState3D yaw_state = controller.makeInitialState();
    constexpr double yaw_v = 0.3;
    constexpr double yaw_yaw_rate = 0.4;
    constexpr double yaw_dt = 0.02;
    constexpr size_t yaw_steps = 100;
    const auto yaw_cmd = makeRobotCommand(yaw_v, 0.0, yaw_yaw_rate);
    double max_pitch_dot = 0.0;
    for (size_t k = 0; k < yaw_steps; ++k) {
      const auto vel = controller.step(yaw_cmd, yaw_dt, yaw_state);
      for (size_t j = 0; j < asr::kNum3dJoints; ++j) {
        max_pitch_dot = std::max(
          max_pitch_dot, std::abs(vel.theta_dot[asr::pitchIndex(j)]));
      }
    }
    const bool yaw_only_ok = max_pitch_dot < 1e-9;
    std::cout << "  8.3 Pure yaw (pitch_crosstalk): max|pitch_dot|=" << max_pitch_dot
              << " " << (yaw_only_ok ? "PASS" : "FAIL") << std::endl;
    if (!yaw_only_ok) {
      return 1;
    }

    // 纯俯仰：只有 pitch 关节应有非零速率
    asr::SimulationState3D pitch_state = controller.makeInitialState();
    constexpr double pitch_v = 0.3;
    constexpr double pitch_pitch_rate = 0.3;
    const auto pitch_cmd = makeRobotCommand(pitch_v, pitch_pitch_rate, 0.0);
    double max_yaw_dot = 0.0;
    for (size_t k = 0; k < yaw_steps; ++k) {
      const auto vel = controller.step(pitch_cmd, yaw_dt, pitch_state);
      for (size_t j = 0; j < asr::kNum3dJoints; ++j) {
        max_yaw_dot = std::max(
          max_yaw_dot, std::abs(vel.theta_dot[asr::yawIndex(j)]));
      }
    }
    const bool pitch_only_ok = max_yaw_dot < 1e-9;
    std::cout << "  8.3 Pure pitch (yaw_crosstalk): max|yaw_dot|=" << max_yaw_dot
              << " " << (pitch_only_ok ? "PASS" : "FAIL") << std::endl;
    if (!pitch_only_ok) {
      return 1;
    }
  }

  // 8.4 阻尼无关性：λ∈{0.01,0.05,0.2} 常曲率路径滞后误差均 < 1mm
  {
    constexpr double arc_v = 0.5;
    constexpr double arc_yaw_rate = 0.5;
    constexpr double arc_dt = 0.0025;
    constexpr size_t arc_steps = 8000;
    constexpr size_t steady_start = arc_steps / 2;
    const double arc_radius = arc_v / arc_yaw_rate;
    const double arc_lag = 2.0 * arc_radius * std::asin(link_length / (2.0 * arc_radius));
    const auto arc_cmd = makeRobotCommand(arc_v, 0.0, arc_yaw_rate);

    auto run_with_damping = [&](double lam) {
      asr::FrontUnitController3DParameters p = params;
      p.max_curvature = 2.0;
      p.damping = lam;
      asr::FrontUnitFollowingController3D ctrl(p);
      asr::SimulationState3D st = ctrl.makeInitialState();
      std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> samples;
      samples.reserve(arc_steps);
      for (size_t k = 0; k < arc_steps; ++k) {
        ctrl.step(arc_cmd, arc_dt, st);
        samples.push_back(st.body_points);
      }
      return path_lagged_spatial_rms(samples, 1, 4, 3.0 * arc_lag, steady_start).xyz;
    };

    double max_tail_lag = 0.0;
    for (const double lam : {0.01, 0.05, 0.2}) {
      max_tail_lag = std::max(max_tail_lag, run_with_damping(lam));
    }
    const bool damp_ok = max_tail_lag < 0.001;
    std::cout << "  8.4 Damping independence: max tail lag RMS for λ∈{0.01,0.05,0.2}=" << max_tail_lag
              << " " << (damp_ok ? "PASS" : "FAIL") << " (threshold 1mm)" << std::endl;
    if (!damp_ok) {
      return 1;
    }
  }

  // 8.5 符号自洽：头速取反应显著变差（验证 §4 三条约定正确耦合）
  {
    constexpr double arc_v = 0.5;
    constexpr double arc_yaw_rate = 0.5;
    constexpr double arc_dt = 0.0025;
    constexpr size_t arc_steps = 8000;
    constexpr size_t steady_start = arc_steps / 2;

    auto run_arc_tail_lag = [&](double velocity) {
      asr::FrontUnitController3DParameters arc_params = params;
      arc_params.max_curvature = 2.0;
      asr::FrontUnitFollowingController3D ctrl(arc_params);
      asr::SimulationState3D st = ctrl.makeInitialState();
      std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> samples;
      samples.reserve(arc_steps);
      const auto cmd = makeRobotCommand(velocity, 0.0, arc_yaw_rate);
      const double radius = std::abs(velocity / arc_yaw_rate);
      const double lag = 2.0 * radius * std::asin(link_length / (2.0 * radius));
      for (size_t k = 0; k < arc_steps; ++k) {
        ctrl.step(cmd, arc_dt, st);
        samples.push_back(st.body_points);
      }
      return path_lagged_spatial_rms(samples, 1, 4, 3.0 * lag, steady_start).xyz;
    };

    const double pos_tail_err = run_arc_tail_lag(arc_v);
    const double neg_tail_err = run_arc_tail_lag(-arc_v);
    const bool sign_ok = pos_tail_err < 0.001 && neg_tail_err > 0.01;
    std::cout << "  8.5 Sign consistency: pos_v tail_lag_rms=" << pos_tail_err
              << ", neg_v tail_lag_rms=" << neg_tail_err
              << " " << (sign_ok ? "PASS" : "FAIL")
              << " (neg_v should be >> pos_v)" << std::endl;
    if (!sign_ok) {
      return 1;
    }
  }

  // 8.6 链式传播：螺旋指令下三处关节都应明显弯曲
  {
    asr::SimulationState3D chain_state = controller.makeInitialState();
    constexpr double chain_dt = 0.02;
    constexpr size_t chain_steps = 500;
    const auto chain_cmd = makeRobotCommand(0.10, 0.08, 0.10);

    for (size_t k = 0; k < chain_steps; ++k) {
      controller.step(chain_cmd, chain_dt, chain_state);
    }

    std::array<double, asr::kNum3dJoints> bend{};
    for (size_t j = 0; j < asr::kNum3dJoints; ++j) {
      bend[j] = std::hypot(
        chain_state.joints.theta[asr::yawIndex(j)],
        chain_state.joints.theta[asr::pitchIndex(j)]);
    }
    const bool chain_ok = bend[0] > 0.35 && bend[1] > 0.20 && bend[2] > 0.20;
    std::cout << "  8.6 Chain propagation: bend joint0/1/2=" << bend[0]
              << ", " << bend[1] << ", " << bend[2]
              << " " << (chain_ok ? "PASS" : "FAIL") << std::endl;
    if (!chain_ok) {
      return 1;
    }
  }

  // 8.7 关节限幅稳定性：高指令下不抖动、不发散
  {
    asr::SimulationState3D limit_state = controller.makeInitialState();
    constexpr double high_v = 0.12;
    constexpr double high_pitch = 0.35;
    constexpr double high_yaw = 0.35;
    constexpr double limit_dt = 0.02;
    constexpr size_t limit_steps = 500;
    const auto limit_cmd = makeRobotCommand(high_v, high_pitch, high_yaw);

    bool stable = true;
    for (size_t k = 0; k < limit_steps; ++k) {
      controller.step(limit_cmd, limit_dt, limit_state);
      const double tail_norm = asr::norm(limit_state.body_points[4]);

      // 检查所有关节角在限位内
      for (size_t j = 0; j < asr::kNum3dJoints; ++j) {
        const double y = limit_state.joints.theta[asr::yawIndex(j)];
        const double p = limit_state.joints.theta[asr::pitchIndex(j)];
        if (std::abs(p) > params.joint_limit + 1e-9 || std::abs(y) > params.joint_limit + 1e-9) {
          stable = false;
        }
      }
      stable = stable && std::isfinite(tail_norm);
    }
    // 验证所有点在合理范围（不 NaN、不无限）
    bool all_finite = true;
    for (const auto & pt : limit_state.body_points) {
      all_finite = all_finite && std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
    }

    const bool limit_ok = stable && all_finite;
    std::cout << "  8.7 Joint limit stability: " << (limit_ok ? "PASS" : "FAIL") << std::endl;
    if (!limit_ok) {
      return 1;
    }
  }

  // 8.8 Table I 复算（ω₁=−(π/30)cos(λπ/60 t)，λ=1 → j2/j3/j4 MAX ≈ 1.98/3.92/5.80 ×10⁻³ m）
  {
    constexpr double table_dt = 0.01;
    constexpr size_t table_steps = 600;
    constexpr double table_omega_0 = pi_value / 30.0;
    constexpr double table_curvature_lambda = 1.0;
    constexpr double table_v = 0.1;
    constexpr double table_link_length = 0.1;
    const size_t table_lag_steps = static_cast<size_t>(
      std::round(table_link_length / (table_v * table_dt)));

    asr::FrontUnitController3DParameters table_params = params;
    table_params.link_length = table_link_length;
    table_params.max_curvature = 2.0;
    table_params.damping = 0.0;
    asr::FrontUnitFollowingController3D table_ctrl(table_params);
    asr::SimulationState3D table_hist_state = table_ctrl.makeInitialState();
    std::vector<std::array<asr::Vec3, asr::kNum3dPoints>> table_samples;
    for (size_t k = 0; k < table_steps; ++k) {
      const double t = k * table_dt;
      const double yaw_rate = -table_omega_0 * std::cos(table_curvature_lambda * pi_value * t / 60.0);
      const auto table_cmd = makeRobotCommand(table_v, 0.0, yaw_rate);
      table_ctrl.step(table_cmd, table_dt, table_hist_state);
      table_samples.push_back(table_hist_state.body_points);
    }

    const double segment_12 = max_lagged_spatial_error(table_samples, 1, 2, table_lag_steps);
    const double segment_23 = max_lagged_spatial_error(table_samples, 2, 3, table_lag_steps);
    const double segment_34 = max_lagged_spatial_error(table_samples, 3, 4, table_lag_steps);
    const double table_j2_max = segment_12;
    const double table_j3_max = segment_12 + segment_23;
    const double table_j4_max = segment_12 + segment_23 + segment_34;

    constexpr double expected_j2 = 1.98e-3;
    constexpr double expected_j3 = 3.92e-3;
    constexpr double expected_j4 = 5.80e-3;
    constexpr double table_tolerance = 0.45e-3;
    const bool table_ok =
      std::abs(table_j2_max - expected_j2) < table_tolerance &&
      std::abs(table_j3_max - expected_j3) < table_tolerance &&
      std::abs(table_j4_max - expected_j4) < table_tolerance;

    std::cout << "  8.8 Table I (λ=1): j2/j3/j4 MAX=" << table_j2_max
              << ", " << table_j3_max << ", " << table_j4_max << " m"
              << " expected≈" << expected_j2 << ", " << expected_j3 << ", " << expected_j4
              << " " << (table_ok ? "PASS" : "FAIL")
              << " (tolerance " << table_tolerance << " m)" << std::endl;
    if (!table_ok) {
      return 1;
    }
  }

  append_sample(0.0, 0.0, 0.0);
  draw_frame();

  constexpr auto frame_period = std::chrono::milliseconds(200);
  auto next_frame = std::chrono::steady_clock::now();

  for (const auto & command : commands) {
    for (size_t i = 0; i < command.steps; ++i) {
      step_simulation(command);

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

  std::cout << "Final yaw-pitch joint angles:" << std::endl;
  for (size_t joint = 0; joint < asr::kNum3dJoints; ++joint) {
    std::cout << "  joint " << (joint + 1)
              << ": yaw=" << state.joints.theta[asr::yawIndex(joint)]
              << ", pitch=" << state.joints.theta[asr::pitchIndex(joint)] << std::endl;
  }

  std::cout << "Delayed joint angle RMS error relative to joint 1:" << std::endl;
  std::cout << "  joint 2 lag=" << fixed_lag_steps * dt << "s pitch="
            << rms_lagged_difference(
                 theta_history[asr::pitchIndex(0)], theta_history[asr::pitchIndex(1)], fixed_lag_steps)
            << ", yaw="
            << rms_lagged_difference(
                 theta_history[asr::yawIndex(0)], theta_history[asr::yawIndex(1)], fixed_lag_steps)
            << std::endl;
  std::cout << "  joint 3 lag=" << 2 * fixed_lag_steps * dt << "s pitch="
            << rms_lagged_difference(
                 theta_history[asr::pitchIndex(0)], theta_history[asr::pitchIndex(2)], 2 * fixed_lag_steps)
            << ", yaw="
            << rms_lagged_difference(
                 theta_history[asr::yawIndex(0)], theta_history[asr::yawIndex(2)], 2 * fixed_lag_steps)
            << std::endl;

  auto print_path_lag_rms = [&](const std::string & label, size_t follower_point, double lag_distance) {
      const SpatialPathLagRms rms = path_lagged_spatial_rms(body_history, 1, follower_point, lag_distance);
      std::cout << "  " << label << " lag=" << lag_distance << " m: xyz=" << rms.xyz
                << " m, xy=" << rms.xy << " m, z=" << rms.z << " m, samples=" << rms.count
                << std::endl;
    };
  auto print_fixed_step_spatial_rms = [&](const std::string & label, size_t reference_point,
    size_t follower_point, size_t lag_steps) {
      const SpatialRms rms = lagged_spatial_rms(body_history, reference_point, follower_point, lag_steps);
      std::cout << "  " << label << " lag=" << lag_steps * dt << "s: xyz=" << rms.xyz
                << " m, xy=" << rms.xy << " m, z=" << rms.z << " m" << std::endl;
    };

  std::cout << "Path-lag spatial RMS relative to joint1:" << std::endl;
  print_path_lag_rms("joint2 <- joint1", 2, link_length);
  print_path_lag_rms("joint3 <- joint1", 3, 2.0 * link_length);
  print_path_lag_rms("tail <- joint1", 4, 3.0 * link_length);

  std::cout << "Fixed-step spatial lag diagnostic:" << std::endl;
  print_fixed_step_spatial_rms("joint2 <- joint1", 1, 2, fixed_lag_steps);
  print_fixed_step_spatial_rms("joint3 <- joint2", 2, 3, fixed_lag_steps);
  print_fixed_step_spatial_rms("tail <- joint3", 3, 4, fixed_lag_steps);
  print_fixed_step_spatial_rms("joint3 <- joint1", 1, 3, 2 * fixed_lag_steps);
  print_fixed_step_spatial_rms("tail <- joint1", 1, 4, 3 * fixed_lag_steps);

  std::cout << "Max |z| path extent: " << max_abs_z_extent(body_history) << " m" << std::endl;
  std::cout << "Max observed curvature: " << max_observed_curvature << " 1/m" << std::endl;
  std::cout << "Max adjacent-axis angle and local curvature:" << std::endl;
  for (size_t i = 0; i < max_adjacent_axis_angle.size(); ++i) {
    std::cout << "  link" << i << "-link" << (i + 1) << ": angle="
              << max_adjacent_axis_angle[i] << " rad, curvature="
              << max_adjacent_axis_angle[i] / link_length << " 1/m" << std::endl;
  }
  std::cout << "Fixed lag steps: " << fixed_lag_steps << " (" << fixed_lag_steps * dt << "s)" << std::endl;
  std::cout << "Last recorded samples: " << time_history.size() << std::endl;

  draw_analysis();
  plt::show();
  return 0;
}

#include "asr_sdm_head_following_control/head_command_mpc.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "head_command_mpc_test failed: " << message << std::endl;
    std::exit(1);
  }
}

asr_sdm_control_msgs::msg::RobotCommand makeFeedforward(double v, double pitch_rate, double yaw_rate)
{
  asr_sdm_control_msgs::msg::RobotCommand cmd;
  cmd.vel.linear.x = v;
  cmd.vel.angular.y = pitch_rate;
  cmd.vel.angular.z = yaw_rate;
  return cmd;
}

asr_sdm_control_msgs::msg::RobotCommand makePlanningCommand(
  double px, double py, double pz, double vx, double vy, double vz)
{
  asr_sdm_control_msgs::msg::RobotCommand cmd = makeFeedforward(std::hypot(vx, vy, vz), 0.0, 0.0);
  cmd.trajectory_flag = asr_sdm_control_msgs::msg::RobotCommand::TRAJECTORY_STATUS_READY;
  cmd.position.x = px;
  cmd.position.y = py;
  cmd.position.z = pz;
  cmd.velocity.x = vx;
  cmd.velocity.y = vy;
  cmd.velocity.z = vz;
  cmd.pos.position = cmd.position;
  cmd.pos.orientation.w = 1.0;
  return cmd;
}
}  // namespace

int main()
{
  asr::HeadCommandMPCParameters params;
  params.Thor = 0.4;
  params.dt = 0.02;
  params.Nhor = 16;
  params.max_linear_velocity = 1.0;
  asr::HeadCommandMPC mpc(params);
  asr::FrontUnitFollowingController3D controller({0.25, 2.0, 1.57, params.max_curvature, 1.0e-6, 0.02});
  asr::SimulationState3D state = controller.makeInitialState();

  const auto teleop = makeFeedforward(0.12, 0.0, 0.2);
  const auto teleop_out = mpc.compensate(teleop, teleop, state);
  require(!teleop_out.applied, "teleop command should skip GRAMPC");
  require(std::abs(teleop_out.command.vel.linear.x - 0.12) < 1.0e-9, "teleop v unchanged");
  require(std::abs(teleop_out.command.vel.angular.z - 0.2) < 1.0e-9, "teleop yaw rate unchanged");

  const auto planning = makePlanningCommand(0.4, 0.0, 0.0, 0.1, 0.0, 0.0);
  const auto ff = makeFeedforward(0.1, 0.0, 0.0);
  const auto catch_up = mpc.compensate(planning, ff, state);
  require(catch_up.applied, "planning command should run GRAMPC");
  require(std::isfinite(catch_up.command.vel.linear.x), "compensated v finite");
  require(catch_up.command.vel.linear.x > ff.vel.linear.x, "positive along-track error should raise v");
  require(catch_up.command.vel.linear.x <= params.max_linear_velocity + 1.0e-9, "v within limits");
  require(
    catch_up.position_error.x > 0.3 && std::abs(catch_up.position_error.y) < 1.0e-9,
    "position error should match desired minus head");

  const auto small = makePlanningCommand(0.05, 0.0, 0.0, 0.1, 0.0, 0.0);
  const auto small_out = mpc.compensate(small, ff, state);
  require(small_out.applied, "small error should still run GRAMPC");
  require(small_out.command.vel.linear.x > ff.vel.linear.x, "small along-track error should raise v");
  require(
    small_out.command.vel.linear.x < catch_up.command.vel.linear.x,
    "smaller error should request less extra speed");

  // Target behind and to the side of the head: the unconstrained optimum turns at the rate
  // limit while v stays near zero, which limitCommand would then flatten to zero.
  const auto behind = makePlanningCommand(-0.5, 0.3, 0.0, 0.0, 0.0, 0.0);
  const auto behind_ff = makeFeedforward(0.1, 0.0, 0.0);
  const auto behind_out = mpc.compensate(behind, behind_ff, state);
  require(behind_out.applied, "behind target should run GRAMPC");
  const double behind_v = behind_out.command.vel.linear.x;
  const double behind_omega = std::hypot(
    behind_out.command.vel.angular.y, behind_out.command.vel.angular.z);
  require(
    behind_v >= behind_ff.vel.linear.x - 1.0e-9, "MPC must not stall below the planned speed");
  // Penalty method only enforces the constraint approximately, so allow a small margin.
  require(
    behind_omega <= params.max_curvature * behind_v + 0.05,
    "MPC output should respect the curvature limit");

  // The planner and the MPC model treat angular.y / angular.z as pitch / yaw Euler rates
  // (R = Rz(yaw) * Ry(pitch)). While pitched, the plant must still follow them without
  // accumulating roll, otherwise the MPC prediction and the real head pose diverge.
  {
    asr::SimulationState3D euler_state = controller.makeInitialState();
    constexpr double euler_pitch_rate = -0.2;
    constexpr double euler_yaw_rate = -0.4;
    constexpr double euler_dt = 0.001;
    constexpr int euler_steps = 4000;
    const auto euler_cmd = makeFeedforward(0.5, euler_pitch_rate, euler_yaw_rate);
    for (int k = 0; k < euler_steps; ++k) {
      controller.step(euler_cmd, euler_dt, euler_state);
    }
    const auto & r = euler_state.head_frame.v;
    const double roll = std::atan2(r[2][1], r[2][2]);
    const double pitch = std::atan2(-r[2][0], std::hypot(r[0][0], r[1][0]));
    const double yaw = std::atan2(r[1][0], r[0][0]);
    const double elapsed = euler_dt * euler_steps;
    std::cout << "euler rates: roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << std::endl;
    require(std::abs(roll) < 5.0e-3, "pitch/yaw Euler rates must not build up roll");
    require(std::abs(pitch - euler_pitch_rate * elapsed) < 5.0e-3, "pitch should integrate pitch rate");
    require(std::abs(yaw - euler_yaw_rate * elapsed) < 5.0e-3, "yaw should integrate yaw rate");
  }

  std::cout << "behind target: v=" << behind_v << " omega=" << behind_omega
            << " limit=" << params.max_curvature * behind_v << std::endl;
  std::cout << "head_command_mpc_test passed: v_ff=" << ff.vel.linear.x
            << " v_mpc_large=" << catch_up.command.vel.linear.x
            << " v_mpc_small=" << small_out.command.vel.linear.x
            << " e_x=" << catch_up.position_error.x << std::endl;
  return 0;
}

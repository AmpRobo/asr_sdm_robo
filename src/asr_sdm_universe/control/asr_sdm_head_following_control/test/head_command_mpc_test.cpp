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
  asr::FrontUnitFollowingController3D controller({0.25, 2.0, 1.57, 1.2, 1.0e-6, 0.02});
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

  std::cout << "head_command_mpc_test passed: v_ff=" << ff.vel.linear.x
            << " v_mpc_large=" << catch_up.command.vel.linear.x
            << " v_mpc_small=" << small_out.command.vel.linear.x
            << " e_x=" << catch_up.position_error.x << std::endl;
  return 0;
}

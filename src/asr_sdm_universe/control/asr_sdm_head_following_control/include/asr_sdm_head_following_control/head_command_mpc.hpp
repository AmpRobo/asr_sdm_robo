#ifndef ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_MPC_HPP_
#define ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_MPC_HPP_

#include "asr_sdm_head_following_control/front_unit_following_controller_3d.hpp"

#include "asr_sdm_control_msgs/msg/robot_command.hpp"

#include <memory>

namespace asr
{

struct HeadCommandMPCParameters
{
  double Thor = 0.4;
  double dt = 0.02;
  int Nhor = 20;
  int max_grad_iter = 4;
  double q_position = 8.0;
  double q_heading = 1.5;
  double r_linear = 0.2;
  double r_angular = 0.05;
  double terminal_position_scale = 4.0;
  double min_linear_velocity = 0.0;
  double max_linear_velocity = 1.0;
  double min_pitch_rate = -0.5;
  double max_pitch_rate = 0.5;
  double min_yaw_rate = -0.5;
  double max_yaw_rate = 0.5;
};

struct HeadCommandMPCResult
{
  asr_sdm_control_msgs::msg::RobotCommand command{};
  Vec3 position_error{};
  bool applied{false};
};

/// GRAMPC that corrects the head-following velocity command so the
/// head tracks `robot_cmd` position / velocity / acceleration.
class HeadCommandMPC
{
public:
  explicit HeadCommandMPC(const HeadCommandMPCParameters & params);
  ~HeadCommandMPC();

  HeadCommandMPC(const HeadCommandMPC &) = delete;
  HeadCommandMPC & operator=(const HeadCommandMPC &) = delete;

  static bool hasPositionReference(const asr_sdm_control_msgs::msg::RobotCommand & cmd);

  HeadCommandMPCResult compensate(
    const asr_sdm_control_msgs::msg::RobotCommand & raw_cmd,
    const asr_sdm_control_msgs::msg::RobotCommand & feedforward_cmd,
    const SimulationState3D & state);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace asr

#endif  // ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_MPC_HPP_

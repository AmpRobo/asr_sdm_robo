#include "asr_sdm_head_following_control/head_command_mpc.hpp"

#include "asr_sdm_head_following_control/control_utils.hpp"
#include "head_command_problem_description.hpp"

#include <grampc_s/grampc_s.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace asr
{
namespace
{
void yawPitchFromFrame(const Mat3 & frame, double & yaw, double & pitch)
{
  const Vec3 forward = column(frame, 0);
  const double horiz = std::hypot(forward.x, forward.y);
  pitch = std::atan2(-forward.z, horiz);
  yaw = std::atan2(forward.y, forward.x);
}

void yawPitchFromCommand(const asr_sdm_control_msgs::msg::RobotCommand & cmd, double & yaw, double & pitch)
{
  const auto & q = cmd.pos.orientation;
  const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (qn > 1.0e-6) {
    const double x = q.x / qn;
    const double y = q.y / qn;
    const double z = q.z / qn;
    const double w = q.w / qn;
    const double fx = 1.0 - 2.0 * (y * y + z * z);
    const double fy = 2.0 * (x * y + z * w);
    const double fz = 2.0 * (x * z - y * w);
    const double horiz = std::hypot(fx, fy);
    pitch = std::atan2(-fz, horiz);
    yaw = std::atan2(fy, fx);
    return;
  }
  yaw = cmd.yaw;
  pitch = 0.0;
}

std::array<typeRNum, 13> makeCostWeights(const HeadCommandMPCParameters & params)
{
  const typeRNum qp = static_cast<typeRNum>(params.q_position);
  const typeRNum qh = static_cast<typeRNum>(params.q_heading);
  const typeRNum scale = static_cast<typeRNum>(params.terminal_position_scale);
  return {
    qp, qp, qp, qh, qh,
    static_cast<typeRNum>(params.r_linear),
    static_cast<typeRNum>(params.r_angular),
    static_cast<typeRNum>(params.r_angular),
    scale * qp, scale * qp, scale * qp, scale * qh, scale * qh};
}

}  // namespace

class HeadCommandMPC::Impl
{
public:
  explicit Impl(const HeadCommandMPCParameters & params)
  : params_(params),
    problem_(std::make_shared<HeadCommandProblemDescription>(makeCostWeights(params))),
    solver_(grampc::Solver(problem_))
  {
    const std::vector<typeRNum> xdes(5, 0.0);
    std::vector<typeRNum> u0 = {0.0, 0.0, 0.0};
    std::vector<typeRNum> udes = {0.0, 0.0, 0.0};
    std::vector<typeRNum> umax = {
      static_cast<typeRNum>(params_.max_linear_velocity),
      static_cast<typeRNum>(params_.max_pitch_rate),
      static_cast<typeRNum>(params_.max_yaw_rate)};
    std::vector<typeRNum> umin = {
      static_cast<typeRNum>(params_.min_linear_velocity),
      static_cast<typeRNum>(params_.min_pitch_rate),
      static_cast<typeRNum>(params_.min_yaw_rate)};
    std::vector<typeRNum> x0 = {0.0, 0.0, 0.0, 0.0, 0.0};

    solver_->setparam_real("Thor", static_cast<typeRNum>(params_.Thor));
    solver_->setparam_real("dt", static_cast<typeRNum>(params_.dt));
    solver_->setparam_real("t0", 0.0);
    solver_->setparam_real_vector("x0", x0.data());
    solver_->setparam_real_vector("xdes", xdes.data());
    solver_->setparam_real_vector("u0", u0.data());
    solver_->setparam_real_vector("udes", udes.data());
    solver_->setparam_real_vector("umax", umax.data());
    solver_->setparam_real_vector("umin", umin.data());

    solver_->setopt_int("Nhor", params_.Nhor);
    solver_->setopt_int("MaxGradIter", params_.max_grad_iter);
    solver_->setopt_int("MaxMultIter", 1);
    solver_->setopt_string("Integrator", "erk2");
    solver_->setopt_string("TerminalCost", "on");
  }

  HeadCommandMPCResult compensate(
    const asr_sdm_control_msgs::msg::RobotCommand & raw_cmd,
    const asr_sdm_control_msgs::msg::RobotCommand & feedforward_cmd,
    const SimulationState3D & state)
  {
    HeadCommandMPCResult result;
    result.command = feedforward_cmd;
    result.position_error = {
      raw_cmd.position.x - state.head_position.x,
      raw_cmd.position.y - state.head_position.y,
      raw_cmd.position.z - state.head_position.z};

    if (!HeadCommandMPC::hasPositionReference(raw_cmd)) {
      return result;
    }

    double yaw_des = 0.0;
    double pitch_des = 0.0;
    yawPitchFromCommand(raw_cmd, yaw_des, pitch_des);

    double yaw = 0.0;
    double pitch = 0.0;
    yawPitchFromFrame(state.head_frame, yaw, pitch);

    problem_->setReference(
      {static_cast<typeRNum>(raw_cmd.position.x), static_cast<typeRNum>(raw_cmd.position.y),
        static_cast<typeRNum>(raw_cmd.position.z)},
      {static_cast<typeRNum>(raw_cmd.velocity.x), static_cast<typeRNum>(raw_cmd.velocity.y),
        static_cast<typeRNum>(raw_cmd.velocity.z)},
      {static_cast<typeRNum>(raw_cmd.acceleration.x), static_cast<typeRNum>(raw_cmd.acceleration.y),
        static_cast<typeRNum>(raw_cmd.acceleration.z)},
      static_cast<typeRNum>(yaw_des), static_cast<typeRNum>(pitch_des),
      static_cast<typeRNum>(raw_cmd.yaw_dot), static_cast<typeRNum>(raw_cmd.vel.angular.y),
      0.0);

    std::vector<typeRNum> x0 = {
      static_cast<typeRNum>(state.head_position.x),
      static_cast<typeRNum>(state.head_position.y),
      static_cast<typeRNum>(state.head_position.z),
      static_cast<typeRNum>(yaw),
      static_cast<typeRNum>(pitch)};
    std::vector<typeRNum> udes = {
      static_cast<typeRNum>(feedforward_cmd.vel.linear.x),
      static_cast<typeRNum>(feedforward_cmd.vel.angular.y),
      static_cast<typeRNum>(feedforward_cmd.vel.angular.z)};

    solver_->setparam_real("t0", 0.0);
    solver_->setparam_real_vector("x0", x0.data());
    solver_->setparam_real_vector("udes", udes.data());
    solver_->run();

    const typeGRAMPCsol * sol = solver_->getSolution();
    if (sol == nullptr || sol->unext == nullptr) {
      return result;
    }
    if (
      !std::isfinite(sol->unext[0]) || !std::isfinite(sol->unext[1]) ||
      !std::isfinite(sol->unext[2]))
    {
      return result;
    }

    result.command.vel.linear.x = std::clamp(
      static_cast<double>(sol->unext[0]), params_.min_linear_velocity,
      params_.max_linear_velocity);
    result.command.vel.angular.y = std::clamp(
      static_cast<double>(sol->unext[1]), params_.min_pitch_rate, params_.max_pitch_rate);
    result.command.vel.angular.z = std::clamp(
      static_cast<double>(sol->unext[2]), params_.min_yaw_rate, params_.max_yaw_rate);
    result.applied = true;
    return result;
  }

private:
  HeadCommandMPCParameters params_;
  std::shared_ptr<HeadCommandProblemDescription> problem_;
  grampc::GrampcPtr solver_;
};

HeadCommandMPC::HeadCommandMPC(const HeadCommandMPCParameters & params)
: impl_(std::make_unique<Impl>(params))
{
}

HeadCommandMPC::~HeadCommandMPC() = default;

bool HeadCommandMPC::hasPositionReference(const asr_sdm_control_msgs::msg::RobotCommand & cmd)
{
  if (cmd.trajectory_flag != asr_sdm_control_msgs::msg::RobotCommand::TRAJECTORY_STATUS_READY) {
    return false;
  }
  return std::isfinite(cmd.position.x) && std::isfinite(cmd.position.y) &&
         std::isfinite(cmd.position.z) && std::isfinite(cmd.velocity.x) &&
         std::isfinite(cmd.velocity.y) && std::isfinite(cmd.velocity.z);
}

HeadCommandMPCResult HeadCommandMPC::compensate(
  const asr_sdm_control_msgs::msg::RobotCommand & raw_cmd,
  const asr_sdm_control_msgs::msg::RobotCommand & feedforward_cmd,
  const SimulationState3D & state)
{
  return impl_->compensate(raw_cmd, feedforward_cmd, state);
}

}  // namespace asr

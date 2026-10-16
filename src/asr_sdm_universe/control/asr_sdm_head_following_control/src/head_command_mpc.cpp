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
    problem_(std::make_shared<HeadCommandProblemDescription>(
        makeCostWeights(params), static_cast<typeRNum>(params.max_curvature),
        static_cast<typeRNum>(params.curvature_safety_margin))),
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
    // The curvature inequality is handled by penalty/multiplier updates. With the default
    // PenaltyMin (1) it is negligible next to the tracking cost, so start from a stiffer penalty.
    solver_->setopt_real("PenaltyMin", static_cast<typeRNum>(1.0e2));
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

    const double e_norm = std::sqrt(
      result.position_error.x * result.position_error.x +
      result.position_error.y * result.position_error.y +
      result.position_error.z * result.position_error.z);
    constexpr double kErrorRef = 0.20;
    constexpr double kMaxBoost = 3.0;
    constexpr double kCloseError = 0.06;
    constexpr double kLagError = 0.08;
    const bool lagged = e_norm > kLagError;
    const double boost = 1.0 + std::min(kMaxBoost, e_norm / kErrorRef);
    // Close: prefer feedforward. Lagged: cheap angular so the solver can turn hard
    // toward the frozen pose; stiffen position / terminal.
    const double r_ang_scale = lagged
                                 ? 0.25
                                 : ((e_norm < kCloseError)
                                      ? (kCloseError / std::max(e_norm, 0.02))
                                      : 1.0);
    auto weights = makeCostWeights(params_);
    weights[0] *= static_cast<typeRNum>(boost);
    weights[1] *= static_cast<typeRNum>(boost);
    weights[2] *= static_cast<typeRNum>(boost);
    weights[6] *= static_cast<typeRNum>(r_ang_scale);
    weights[7] *= static_cast<typeRNum>(r_ang_scale);
    weights[8] *= static_cast<typeRNum>(boost);
    weights[9] *= static_cast<typeRNum>(boost);
    weights[10] *= static_cast<typeRNum>(boost);
    if (lagged) {
      // Heading should follow the catch-up aim, not the planner's sharp tangent.
      weights[3] *= static_cast<typeRNum>(0.5);
      weights[4] *= static_cast<typeRNum>(0.5);
      weights[11] *= static_cast<typeRNum>(0.5);
      weights[12] *= static_cast<typeRNum>(0.5);
    }
    problem_->setCostWeights(weights);

    double yaw_des = 0.0;
    double pitch_des = 0.0;
    yawPitchFromCommand(raw_cmd, yaw_des, pitch_des);

    double yaw = 0.0;
    double pitch = 0.0;
    yawPitchFromFrame(state.head_frame, yaw, pitch);

    const double ex = result.position_error.x;
    const double ey = result.position_error.y;
    const double ez = result.position_error.z;

    // Lagged on a sharp timed turn: freeze the reference at the current commanded pose
    // and aim body-x at the error. Chasing p+vτ+½aτ² while already behind only pulls
    // the head farther off (logs: |e|→0.3 m with ω saturated).
    if (lagged && e_norm > 1.0e-6) {
      const double horiz = std::hypot(ex, ey);
      // The bearing atan2(ey, ex) is only meaningful when the error has a horizontal part. For a
      // mostly vertical error it is arbitrary, and chasing it saturates yaw rate, which eats the
      // shared curvature budget (ωp² + cos²p·ωy² ≤ (κv)²) and starves the pitch needed to climb.
      // Weight the yaw correction by the horizontal fraction of the error (cos of aim pitch).
      const double yaw_aim = std::atan2(ey, ex);
      yaw_des = yaw + (horiz / e_norm) * wrapAngle(yaw_aim - yaw);
      pitch_des = std::atan2(-ez, std::max(horiz, 1.0e-6));
      problem_->setReference(
        {static_cast<typeRNum>(raw_cmd.position.x), static_cast<typeRNum>(raw_cmd.position.y),
          static_cast<typeRNum>(raw_cmd.position.z)},
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, static_cast<typeRNum>(yaw_des),
        static_cast<typeRNum>(pitch_des), 0.0, 0.0, 0.0);
    } else {
      problem_->setReference(
        {static_cast<typeRNum>(raw_cmd.position.x), static_cast<typeRNum>(raw_cmd.position.y),
          static_cast<typeRNum>(raw_cmd.position.z)},
        {static_cast<typeRNum>(raw_cmd.velocity.x), static_cast<typeRNum>(raw_cmd.velocity.y),
          static_cast<typeRNum>(raw_cmd.velocity.z)},
        {static_cast<typeRNum>(raw_cmd.acceleration.x),
          static_cast<typeRNum>(raw_cmd.acceleration.y),
          static_cast<typeRNum>(raw_cmd.acceleration.z)},
        static_cast<typeRNum>(yaw_des), static_cast<typeRNum>(pitch_des),
        static_cast<typeRNum>(raw_cmd.yaw_dot), static_cast<typeRNum>(raw_cmd.vel.angular.y),
        0.0);
    }

    std::vector<typeRNum> x0 = {
      static_cast<typeRNum>(state.head_position.x),
      static_cast<typeRNum>(state.head_position.y),
      static_cast<typeRNum>(state.head_position.z),
      static_cast<typeRNum>(yaw),
      static_cast<typeRNum>(pitch)};

    const double ff_v = std::clamp(
      static_cast<double>(feedforward_cmd.vel.linear.x), params_.min_linear_velocity,
      params_.max_linear_velocity);
    std::vector<typeRNum> udes;
    double v_floor = params_.min_linear_velocity;
    if (lagged) {
      // Modest catch-up speed: enough for κ·v to allow turning, not a race with the plan.
      const double catch_v = std::clamp(0.12 + 0.6 * e_norm, 0.12, 0.35);
      udes = {static_cast<typeRNum>(catch_v), 0.0, 0.0};
      // The curvature limit forces ω = 0 at v = 0, so with a floor of 0 the solver can park at
      // v ≈ 0 (e.g. planner finished, ff_v = 0, target behind/beside the head) and never turn
      // back. Keep enough speed to turn at kMinTurnRate under the same limit the MPC enforces.
      constexpr double kMinTurnRate = 0.3;  // rad/s
      const double kappa = params_.max_curvature * params_.curvature_safety_margin;
      v_floor = (kappa > 1.0e-6) ? std::min(catch_v, kMinTurnRate / kappa)
                                 : params_.min_linear_velocity;
      v_floor = std::max(v_floor, params_.min_linear_velocity);
    } else {
      udes = {
        static_cast<typeRNum>(feedforward_cmd.vel.linear.x),
        static_cast<typeRNum>(feedforward_cmd.vel.angular.y),
        static_cast<typeRNum>(feedforward_cmd.vel.angular.z)};
      // Keep planned speed as floor only while the gap is still mostly ahead.
      double along = 1.0;
      if (e_norm > 1.0e-6) {
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const double cpit = std::cos(pitch);
        const double spit = std::sin(pitch);
        along = (cy * cpit * ex + sy * cpit * ey + (-spit) * ez) / e_norm;
      }
      v_floor = (along > 0.3) ? ff_v : params_.min_linear_velocity;
    }
    std::vector<typeRNum> umin = {
      static_cast<typeRNum>(v_floor), static_cast<typeRNum>(params_.min_pitch_rate),
      static_cast<typeRNum>(params_.min_yaw_rate)};

    solver_->setparam_real("t0", 0.0);
    solver_->setparam_real_vector("x0", x0.data());
    solver_->setparam_real_vector("udes", udes.data());
    solver_->setparam_real_vector("umin", umin.data());
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

    double v = std::clamp(
      static_cast<double>(sol->unext[0]), params_.min_linear_velocity,
      params_.max_linear_velocity);
    double wy = std::clamp(
      static_cast<double>(sol->unext[1]), params_.min_pitch_rate, params_.max_pitch_rate);
    double wz = std::clamp(
      static_cast<double>(sol->unext[2]), params_.min_yaw_rate, params_.max_yaw_rate);

    // Soft authority band around the (already curvature-limited) feedforward when the
    // head is close to the reference. Prevents heading-cost bang-bang from yanking
    // ±ωmax while position error is already centimeters.
    if (e_norm < kCloseError) {
      const double band = 0.12 + 3.0 * e_norm;
      const double ff_v = static_cast<double>(feedforward_cmd.vel.linear.x);
      const double ff_wy = static_cast<double>(feedforward_cmd.vel.angular.y);
      const double ff_wz = static_cast<double>(feedforward_cmd.vel.angular.z);
      v = std::clamp(v, ff_v - band, ff_v + band);
      wy = std::clamp(wy, ff_wy - band, ff_wy + band);
      wz = std::clamp(wz, ff_wz - band, ff_wz + band);
      v = std::clamp(v, params_.min_linear_velocity, params_.max_linear_velocity);
      wy = std::clamp(wy, params_.min_pitch_rate, params_.max_pitch_rate);
      wz = std::clamp(wz, params_.min_yaw_rate, params_.max_yaw_rate);
    }

    result.command.vel.linear.x = v;
    result.command.vel.angular.y = wy;
    result.command.vel.angular.z = wz;
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

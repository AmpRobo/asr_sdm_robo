#include "asr_sdm_control_manager/front_unit_following_controller_2d.hpp"

#include <cmath>

namespace asr
{

FrontUnitFollowingController::FrontUnitFollowingController(const RobotParameters & params)
: params_(params)
{
}

JointVelocity FrontUnitFollowingController::computeJointVelocity(
  const HeadCommand & head_cmd, const JointState & state) const
{
  const double L = params_.link_length;

  const double v1 = head_cmd.linear_velocity;
  const double w1 = head_cmd.angular_velocity;

  JointVelocity output;

  const size_t num_joints = state.phi.size();

  output.phi_dot.resize(num_joints, 0.0);

  if (num_joints == 0) {
    return output;
  }

  std::vector<double> v_bar(num_joints + 1, 0.0);
  std::vector<double> psi_dot(num_joints + 1, 0.0);

  v_bar[0] = v1;
  psi_dot[0] = w1;

  for (size_t i = 0; i < num_joints; ++i) {
    const double phi = state.phi[i];
    const double omega_gain = (i == 0) ? (2.0 * std::cos(phi) + 1.0) : (std::cos(phi) + 1.0);

    output.phi_dot[i] = -(2.0 / L) * v_bar[i] * std::sin(phi) - psi_dot[i] * omega_gain;

    psi_dot[i + 1] = psi_dot[i] + output.phi_dot[i];

    const double link_scale = (i == 0) ? L : 0.5 * L;
    v_bar[i + 1] = v_bar[i] * std::cos(phi) - link_scale * psi_dot[i] * std::sin(phi);
  }

  return output;
}

}  // namespace asr
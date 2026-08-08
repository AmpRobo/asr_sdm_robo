#ifndef ROBOT_MODEL_HPP_
#define ROBOT_MODEL_HPP_

#include "asr_sdm_controller/types.hpp"

namespace asr
{

class RobotModel
{
public:
  explicit RobotModel(const RobotParameters & params)
  : params_(params)
  {
  }

  ScrewVelocity computeScrewVelocity(
    const HeadCommand & /* cmd */, const JointState & /* state */,
    const JointVelocity & joint_vel) const
  {
    ScrewVelocity output;
    size_t n = params_.alpha.size();
    output.theta_dot.resize(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
      output.theta_dot[i] = (i < joint_vel.phi_dot.size())
        ? joint_vel.phi_dot[i] * std::sin(params_.alpha[i])
        : 0.0;
    }
    return output;
  }

private:
  RobotParameters params_;
};

}  // namespace asr

#endif  // ROBOT_MODEL_HPP_

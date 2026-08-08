#ifndef FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_
#define FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_

#include "asr_sdm_control_manager/types.hpp"

namespace asr
{

class FrontUnitFollowingController
{
public:
  explicit FrontUnitFollowingController(const RobotParameters & params);

  JointVelocity computeJointVelocity(const HeadCommand & head_cmd, const JointState & state) const;

private:
  RobotParameters params_;
};

}  // namespace asr

#endif  // FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_

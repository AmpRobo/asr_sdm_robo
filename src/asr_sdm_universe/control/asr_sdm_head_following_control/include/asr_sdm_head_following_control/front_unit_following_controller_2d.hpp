#ifndef ASR_SDM_HEAD_FOLLOWING_CONTROL_FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_
#define ASR_SDM_HEAD_FOLLOWING_CONTROL_FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_

#include "asr_sdm_head_following_control/types.hpp"

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

#endif  // ASR_SDM_HEAD_FOLLOWING_CONTROL_FRONT_UNIT_FOLLOWING_CONTROLLER_2D_HPP_

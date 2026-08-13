#ifndef ASR_SDM_HEAD_FOLLOWING_CONTROL_TYPES_HPP_
#define ASR_SDM_HEAD_FOLLOWING_CONTROL_TYPES_HPP_

#include <cmath>
#include <vector>

namespace asr
{

struct HeadCommand
{
  double linear_velocity;
  double angular_velocity;
};

struct JointState
{
  std::vector<double> phi;
};

struct JointVelocity
{
  std::vector<double> phi_dot;
};

struct ScrewVelocity
{
  std::vector<double> theta_dot;
};

struct RobotParameters
{
  double link_length;
  double screw_radius;
  std::vector<double> alpha;
};

}  // namespace asr

#endif  // ASR_SDM_HEAD_FOLLOWING_CONTROL_TYPES_HPP_

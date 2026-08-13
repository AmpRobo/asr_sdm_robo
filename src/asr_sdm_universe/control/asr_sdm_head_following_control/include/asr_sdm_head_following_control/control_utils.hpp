#ifndef ASR_SDM_HEAD_FOLLOWING_CONTROL_CONTROL_UTILS_HPP_
#define ASR_SDM_HEAD_FOLLOWING_CONTROL_CONTROL_UTILS_HPP_

#include <cmath>

namespace asr
{

inline double wrapAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

}  // namespace asr

#endif  // ASR_SDM_HEAD_FOLLOWING_CONTROL_CONTROL_UTILS_HPP_

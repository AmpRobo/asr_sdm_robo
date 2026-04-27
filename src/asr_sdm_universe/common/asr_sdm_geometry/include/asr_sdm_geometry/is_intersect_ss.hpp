#ifndef IS_INTERSECT_SS_HPP_
#define IS_INTERSECT_SS_HPP_


#include "ccw.hpp"
#include "segment.hpp"

namespace geometry
{

inline bool is_intersect_ss(const Segment & s, const Segment & t)
{
  return ccw(s.a, s.b, t.a) * ccw(s.a, s.b, t.b) <= 0 &&
         ccw(t.a, t.b, s.a) * ccw(t.a, t.b, s.b) <= 0;
}

}  // namespace geometry

#endif  // IS_INTERSECT_SS_HPP_

#ifndef IS_INTERSECT_LS_HPP_
#define IS_INTERSECT_LS_HPP_


#include "line.hpp"
#include "segment.hpp"

namespace geometry
{

inline bool is_intersect_ls(const Line & l, const Segment & s)
{
  return sign(cross_2d(l.b - l.a, s.a - l.a)) *
           sign(cross_2d(l.b - l.a, s.b - l.a)) <= 0;
}

}  // namespace geometry

#endif  // IS_INTERSECT_LS_HPP_

#ifndef IS_INTERSECT_CP_HPP_
#define IS_INTERSECT_CP_HPP_


#include "circle.hpp"

namespace geometry
{

inline bool is_intersect_cp(const Circle & c, const Point & p)
{
  return equals(abs(p - c.p), c.r);
}

}  // namespace geometry

#endif  // IS_INTERSECT_CP_HPP_

#ifndef IS_INTERSECT_CL_HPP_
#define IS_INTERSECT_CL_HPP_


#include "circle.hpp"
#include "distance_lp.hpp"
#include "line.hpp"

namespace geometry
{

inline bool is_intersect_cl(const Circle & c, const Line & l)
{
  return sign(c.r - distance_lp(l, c.p)) >= 0;
}

}  // namespace geometry

#endif  // IS_INTERSECT_CL_HPP_

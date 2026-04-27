#ifndef DISTANCE_LP_HPP_
#define DISTANCE_LP_HPP_


#include "projection.hpp"

namespace geometry
{

inline Real distance_lp(const Line & l, const Point & p)
{
  return abs(p - projection(l, p));
}

}  // namespace geometry

#endif  // DISTANCE_LP_HPP_

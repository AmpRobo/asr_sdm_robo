#ifndef DISTANCE_PP_HPP_
#define DISTANCE_PP_HPP_


#include "point.hpp"

namespace geometry
{

inline Real distance_pp(const Point & a, const Point & b) { return abs(a - b); }

}  // namespace geometry

#endif  // DISTANCE_PP_HPP_

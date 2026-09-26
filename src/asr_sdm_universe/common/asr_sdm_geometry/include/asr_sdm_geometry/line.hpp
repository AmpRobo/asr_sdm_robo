#ifndef LINE_HPP_
#define LINE_HPP_


#include "point.hpp"

namespace asr
{
namespace geometry
{

struct Line
{
  Point a;
  Point b;

  Line() = default;

  // Line through two distinct 3D points; 2D points are those with z = 0.
  Line(const Point & a_, const Point & b_) : a(a_), b(b_) {}

  // Construct the line Ax + By = C in the xy-plane (z = 0).
  Line(Real A, Real B, Real C)
  {
    if (equals(A, 0)) {
      assert(!equals(B, 0));
      a = Point(0, C / B);
      b = Point(1, C / B);
    } else if (equals(B, 0)) {
      a = Point(C / A, 0);
      b = Point(C / A, 1);
    } else {
      a = Point(0, C / B);
      b = Point(C / A, 0);
    }
  }

  // Line through p with direction d (d must be non-zero).
  static Line from_direction(const Point & p, const Point & d)
  {
    assert(!equals(abs(d), 0));
    return Line(p, p + d);
  }

  // Intersection of planes A1x + B1y + C1z = D1 and A2x + B2y + C2z = D2.
  // `a` is the point on the line closest to the origin; the planes must not be parallel.
  static Line from_planes(
    Real A1, Real B1, Real C1, Real D1, Real A2, Real B2, Real C2, Real D2)
  {
    const Point n1(A1, B1, C1);
    const Point n2(A2, B2, C2);
    const Point d = cross(n1, n2);
    const Real d2 = norm(d);
    assert(!equals(abs(d), 0));
    const Point p = (cross(n2, d) * D1 + cross(d, n1) * D2) / d2;
    return Line(p, p + d);
  }

  Point direction() const { return b - a; }

  // a + t (b - a): t = 0 gives a, t = 1 gives b, t = 0.5 gives the midpoint.
  Point point_at(Real t) const { return a + (b - a) * t; }

  bool is_planar_xy() const { return equals(a.z, 0) && equals(b.z, 0); }

  friend ostream & operator<<(ostream & os, const Line & l)
  {
    return os << l.a << " to " << l.b;
  }
  friend istream & operator>>(istream & is, Line & l) { return is >> l.a >> l.b; }
};

using Lines = vector<Line>;

}  // namespace geometry
}  // namespace asr

#endif  // LINE_HPP_

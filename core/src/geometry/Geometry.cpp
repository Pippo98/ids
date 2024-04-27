#include "Geometry.hpp"

#include <vector>

#include "raymath.h"

bool SegmentsIntersects(const segment_t &s1, const segment_t &s2) {
  const auto &ccw = [](const Vector2 &p1, const Vector2 &p2,
                       const Vector2 &p3) -> bool {
    return (p3.y - p1.y) * (p2.x - p1.x) > (p2.y - p1.y) * (p3.x - p1.x);
  };

  if (ccw(s1.p1, s2.p1, s2.p2) == ccw(s1.p2, s2.p1, s2.p2) ||
      ccw(s1.p1, s1.p2, s2.p1) == ccw(s1.p1, s1.p2, s2.p2)) {
    return false;
  }
  return true;
}
Vector2 SegmentIntersection(const segment_t &s1, const segment_t &s2) {
  const auto &ABCD = [](double xy1, double xy2, double xy3, double xy4,
                        double &ac, double &bd) {
    ac = (xy3 - xy1) / (xy2 - xy1);
    bd = (xy4 - xy3) / (xy2 - xy1);
  };

  Vector2 point;
  if (FloatEquals(s1.p1.y, s1.p2.y)) {  // segment 1 horizontal
    double t = (s1.p1.y - s2.p1.y) / (s2.p2.y - s2.p1.y);
    point = Vector2Lerp(s2.p1, s2.p2, t);
  } else if (FloatEquals(s2.p1.y, s2.p2.y)) {  // segment 2 horizontal
    double t = (s2.p1.y - s1.p1.y) / (s1.p2.y - s1.p1.y);
    point = Vector2Lerp(s1.p1, s1.p2, t);
  } else if (FloatEquals(s1.p1.x, s1.p2.x)) {  // segment 1 vertical
    double t = (s1.p1.x - s2.p1.x) / (s2.p2.x - s2.p1.x);
    point = Vector2Lerp(s2.p2, s2.p2, t);
  } else if (FloatEquals(s2.p1.x, s2.p2.x)) {  // segment 2 vertical
    double t = (s2.p1.x - s1.p1.x) / (s1.p2.x - s1.p1.x);
    point = Vector2Lerp(s1.p1, s1.p2, t);
  } else {
    double a, b, c, d;
    ABCD(s1.p1.x, s1.p2.x, s2.p1.x, s2.p2.x, a, b);
    ABCD(s1.p1.y, s1.p2.y, s2.p1.y, s2.p2.y, c, d);
    double u = (c - a) / (b - d);
    double t = a + u * b;
    point = Vector2Lerp(s1.p1, s1.p2, t);
  }
  return point;
}

int CirclesIntersects(const Vector2 &c1, float r1, const Vector2 &c2,
                      float r2) {
  float dist = Vector2Distance(c1, c2);
  if (FloatEquals(dist, 0.0f)) {
    return 0;
  } else if (dist >= r1 + r2) {  // Too distant
    return 0;
  } else if (dist <= std::abs(r1 - r2)) {  // One circle inside another and
                                           // no possible intersection
    return 1;
  }
  return 2;
}
std::vector<Vector2> CirclesIntersections(const Vector2 &c1, float r1,
                                          const Vector2 &c2, float r2) {
  int intersectionNumber = CirclesIntersects(c1, r1, c2, r2);
  std::vector<Vector2> intersections(intersectionNumber);
  if (intersectionNumber == 1) {
    intersections[0] = Vector2Lerp(c1, c2, 0.5);
  } else if (intersectionNumber == 2) {
    float dist = Vector2Distance(c1, c2);
    float a =
        (std::pow(r1, 2) - std::pow(r2, 2) + std::pow(dist, 2)) / (2 * dist);
    Vector2 median = Vector2MoveTowards(c1, c2, a);
    float h = std::sqrt(std::pow(r1, 2) - std::pow(a, 2));
    intersections[0] = (Vector2){median.x + h * (c2.y - c1.y) / dist,
                                 median.y - h * (c2.x - c1.x) / dist};
    intersections[1] = (Vector2){median.x - h * (c2.y - c1.y) / dist,
                                 median.y + h * (c2.x - c1.x) / dist};
  }
  return intersections;
}

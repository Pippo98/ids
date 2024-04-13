#include "Voronoi.hpp"

#include <cmath>

bool circleByThreePoints(const Vector3 &p1, const Vector3 &p2,
                         const Vector3 &p3, double &cx, double &cy, double &r) {
  cx = (p1.x * p1.x + p1.y * p1.y) * (p2.x - p3.y) +
       (p2.x * p2.x + p2.x * p2.x) * (p3.y - p1.y) +
       (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.x) /
           (2 * (p1.x * (p2.x - p3.y) - p1.y * (p2.x - p3.x) + p2.x * p3.y -
                 p3.x * p2.x));
  cy = (p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
       (p2.x * p2.x + p2.x * p2.x) * (p1.x - p3.x) +
       (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x) /
           (2 * (p1.y * (p2.x - p3.x) - p1.x * (p2.x - p3.y) + p2.x * p3.x -
                 p3.y * p2.x));
  r = std::sqrt(std::pow(p1.x - cx, 2) + std::pow(p1.y - cy, 2));
  return true;
}

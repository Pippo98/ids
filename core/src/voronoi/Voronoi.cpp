#include "Voronoi.hpp"

#include <raylib.h>
#include <stdio.h>

#include <cmath>
#include <cstring>
#include <span>
#include <utility>

#include "raymath.h"
#include "src/raylib.h"

struct cache_t {
  unsigned int visisted : 1;
};

Voronoi::Voronoi(Vector2 _pos, double _radius)
    : pos(_pos), maxRadius(_radius) {}

bool Voronoi::operator==(const Voronoi &other) {
  return Vector2Equals(pos, other.pos) &&
         FloatEquals(maxRadius, other.maxRadius);
}
void Voronoi::setPosition(Vector2 _pos) { pos = _pos; }
void Voronoi::setMaxRadius(double _radius) { maxRadius = _radius; }

Voronoi &VoronoiSolver::addVoronoi(const Voronoi &voronoi) {
  cells.push_back(voronoi);
  return cells.back();
}
Voronoi &VoronoiSolver::addVoronoi(Vector2 position, double maxRadius) {
  return addVoronoi(Voronoi(position, maxRadius));
}

const auto &cacheAt = [](cache_t *arr, const size_t &cols, const size_t &row,
                         const size_t &col) -> cache_t & {
  return arr[row * cols + col];
};
const auto &cacheClear = [](cache_t *arr, const size_t &cols) -> void {
  memset(arr, 0, cols * cols * sizeof(cache_t));
};

void VoronoiSolver::findIntersections() {
  cache_t *cache = new cache_t[cells.size() * cells.size()];
  cacheClear(cache, cells.size());

  for (auto &cell : cells) {
    cell.bounds.clear();
  }

  for (size_t i = 0; i < cells.size(); ++i) {
    for (size_t j = 0; j < cells.size(); ++j) {
      if (i == j) {
        continue;
      }
      cache_t &cacheEl = cacheAt(cache, cells.size(), i, j);
      if (cacheEl.visisted == 1) {
        continue;
      } else {
        cacheEl.visisted = 1;
        cacheAt(cache, cells.size(), j, i).visisted = 1;
      }
      auto &v1 = cells[i];
      auto &v2 = cells[j];
      float dist = Vector2Distance(v1.pos, v2.pos);
      // Coincident
      if (dist == 0) {
        continue;
      } else if (dist >= v1.maxRadius + v2.maxRadius) {  // Too distant
        continue;
      } else if (dist <=
                 std::abs(v1.maxRadius -
                          v2.maxRadius)) {  // One circle inside another and no
                                            // possible intersection
        continue;
      }
      double a = (std::pow(v1.maxRadius, 2) - std::pow(v2.maxRadius, 2) +
                  std::pow(dist, 2)) /
                 (2 * dist);
      Vector2 median = Vector2MoveTowards(v1.pos, v2.pos, a);
      double h = std::sqrt(std::pow(v1.maxRadius, 2) - std::pow(a, 2));
      Vector2 intersection1(median.x + h * (v2.pos.y - v1.pos.y) / dist,
                            median.y - h * (v2.pos.x - v1.pos.x) / dist);
      Vector2 intersection2(median.x - h * (v2.pos.y - v1.pos.y) / dist,
                            median.y + h * (v2.pos.x - v1.pos.x) / dist);
      auto newBound1 =
          Voronoi::intersection_t{{intersection1, intersection2}, v2};
      v1.bounds.push_back(newBound1);
      std::swap(intersection1, intersection2);
      auto newBound2 =
          Voronoi::intersection_t{{intersection1, intersection2}, v1};
      v2.bounds.push_back(newBound2);
    }
  }
  delete[] cache;
}

void VoronoiSolver::removeBoundsIntersections() {
  for (size_t i = 0; i < cells.size(); ++i) {
    auto &bounds = cells[i].bounds;
    for (size_t j = 0; j < bounds.size(); ++j) {
      for (size_t k = 0; k < bounds.size(); ++k) {
        if (j == k) {
          continue;
        }

        auto &b1 = bounds[j];
        auto &b2 = bounds[k];

        const auto &ccw = [](const Vector2 &p1, const Vector2 &p2,
                             const Vector2 &p3) -> bool {
          return (p3.y - p1.y) * (p2.x - p1.x) > (p2.y - p1.y) * (p3.x - p1.x);
        };

        if (ccw(b1.segment.p1, b2.segment.p1, b2.segment.p2) ==
                ccw(b1.segment.p2, b2.segment.p1, b2.segment.p2) ||
            ccw(b1.segment.p1, b1.segment.p2, b2.segment.p1) ==
                ccw(b1.segment.p1, b1.segment.p2, b2.segment.p2)) {
          continue;
        }

        const auto &ABCD = [](double xy1, double xy2, double xy3, double xy4,
                              double &ac, double &bd) {
          ac = (xy3 - xy1) / (xy2 - xy1);
          bd = (xy4 - xy3) / (xy2 - xy1);
        };

        Vector2 point;
        // b1 horizontal
        if (FloatEquals(b1.segment.p1.y, b1.segment.p2.y)) {
          double t = (b1.segment.p1.y - b2.segment.p1.y) /
                     (b2.segment.p2.y - b2.segment.p1.y);
          point = Vector2Lerp(b2.segment.p1, b2.segment.p2, t);
        } else if (FloatEquals(b1.segment.p1.y, b1.segment.p2.y)) {
          double t = (b1.segment.p1.x - b2.segment.p1.x) /
                     (b2.segment.p2.x - b2.segment.p1.x);
          point = Vector2Lerp(b2.segment.p1, b2.segment.p2, t);
        } else {
          double a, b, c, d;
          ABCD(b1.segment.p1.x, b1.segment.p2.x, b2.segment.p1.x,
               b2.segment.p2.x, a, b);
          ABCD(b1.segment.p1.y, b1.segment.p2.y, b2.segment.p1.y,
               b2.segment.p2.y, c, d);
          double u = (c - a) / (b - d);
          double t = a + u * b;
          point = Vector2Lerp(b1.segment.p1, b1.segment.p2, t);
        }

        auto &v1 = b1.with;
        auto &v2 = b2.with;
        if (Vector2Distance(b1.segment.p1, v2.pos) <
            Vector2Distance(b1.segment.p2, v2.pos)) {
          b1.segment.p1 = point;
        } else {
          b1.segment.p2 = point;
        }
        if (Vector2Distance(b2.segment.p1, v1.pos) <
            Vector2Distance(b2.segment.p2, v1.pos)) {
          b2.segment.p1 = point;
        } else {
          b2.segment.p2 = point;
        }
      }
    }
  }
}

bool VoronoiSolver::solve() {
  findIntersections();
  removeBoundsIntersections();
  return true;
}

void VoronoiSolver::draw() const {
  for (size_t i = 0; i < cells.size(); ++i) {
    DrawCircleLines(cells[i].pos.x, cells[i].pos.y, cells[i].maxRadius, RED);
    DrawCircle(cells[i].pos.x, cells[i].pos.y, 2.0, RED);

    for (const auto &bound : cells[i].bounds) {
      DrawLine(bound.segment.p1.x, bound.segment.p1.y, bound.segment.p2.x,
               bound.segment.p2.y, BLUE);
    }
  }
}

VoronoiSolver VoronoiTest() {
  VoronoiSolver solver;

  solver.addVoronoi(Vector2{00, 00}, 100);
  solver.addVoronoi(Vector2{100, 50}, 100);
  solver.addVoronoi(Vector2{100, 170}, 100);
  solver.addVoronoi(Vector2{0, -300}, 100);
  solver.addVoronoi(Vector2{-80, 150}, 50);

  solver.solve();

  return solver;
}

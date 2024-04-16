#include "Voronoi.hpp"

#include <cmath>
#include <span>

#include "raymath.h"

Voronoi::Voronoi(Vector2 _pos, double _radius)
    : pos(_pos), maxRadius(_radius) {}

bool Voronoi::operator==(const Voronoi &other) {
  return Vector2Equals(pos, other.pos) &&
         FloatEquals(maxRadius, other.maxRadius);
}

Voronoi &VoronoiSolver::addVoronoi(const Voronoi &voronoi) {
  cells.push_back(voronoi);
  return cells.back();
}
Voronoi &VoronoiSolver::addVoronoi(Vector2 position, double maxRadius) {
  return addVoronoi(Voronoi(position, maxRadius));
}

void VoronoiSolver::findIntersections() {
  cache_t *cache = new cache_t[cells.size() * cells.size()];

  const auto &cacheAt = [](cache_t *arr, const size_t &cols, const size_t &row,
                           const size_t &col) -> cache_t & {
    return arr[row + cols * col];
  };
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
      Vector2 median = Vector2MoveTowards(v1.pos, v1.pos, a);
      double h = std::sqrt(std::pow(v1.maxRadius, 2) - std::pow(a, 2));
      Vector2 intersection1 =
          Vector2(median.x + h * (v2.pos.y - v1.pos.y) / dist,
                  median.y - h * (v2.pos.x - v1.pos.x) / dist);
      Vector2 intersection2 =
          Vector2(median.x - h * (v2.pos.y - v1.pos.y) / dist,
                  median.y + h * (v2.pos.x - v1.pos.x) / dist);
      v1.bounds.push_back(Voronoi::segment_t{intersection1, intersection2});
    }
  }
  delete[] cache;
}

bool VoronoiSolver::solve() {
  findIntersections();
  return true;
}

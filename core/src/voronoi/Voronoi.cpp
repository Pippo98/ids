#include "Voronoi.hpp"

#include <raylib.h>
#include <stdio.h>

#include <cmath>
#include <cstring>
#include <ratio>
#include <span>
#include <utility>

#include "geometry/Geometry.hpp"
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
void Voronoi::setMaxRadius(float _radius) { maxRadius = _radius; }

bool Voronoi::pointInVoronoi(const Vector2 &position) const {
  segment_t outOfVoronoi{pos, position};
  for (const auto &bound : bounds) {
    if (SegmentsIntersects(bound.segment, outOfVoronoi)) {
      return false;
    }
  }
  return true;
}
float Voronoi::getDensity(const Map &map, const Vector2 &position) const {
  return 1.0 + map.getConfidence(position);
}
void Voronoi::calculateCenterOfMass() const {
  double mass = 0.0;
  centerOfMass = (Vector2){0.0f, 0.0f};
  const size_t numberOfPoints = PI * maxRadius * maxRadius / 100.0;
  const float turnFactor = 0.61;
  for (size_t i = 0; i < numberOfPoints; ++i) {
    float radius = maxRadius * std::sqrt(i / (numberOfPoints - 1.0));
    float angle = 2 * PI * turnFactor * i;

    Vector2 sample{pos.x + radius * std::cos(angle),
                   pos.y + radius * std::sin(angle)};
    if (!pointInVoronoi(sample)) {
      continue;
    }
    DrawCircle(sample.x, sample.y, 1.0, BLACK);
    mass += 1.0;
    centerOfMass.x += sample.x;
    centerOfMass.y += sample.y;
  }
  if (!FloatEquals(mass, 0.0f)) {
    centerOfMass.x /= mass;
    centerOfMass.y /= mass;
  }
}
void Voronoi::calculateCenterOfMass(const Map &map) const {
  double mass = 0.0;
  centerOfMass = (Vector2){0.0f, 0.0f};
  const size_t numberOfPoints = PI * maxRadius * maxRadius / 100.0;
  const float turnFactor = 0.61;
  for (size_t i = 0; i < numberOfPoints; ++i) {
    float radius = maxRadius * std::sqrt(i / (numberOfPoints - 1.0));
    float angle = 2 * PI * turnFactor * i;

    Vector2 sample{pos.x + radius * std::cos(angle),
                   pos.y + radius * std::sin(angle)};
    if (map.getTileType(sample) == TileType::OBSTACLE) {
      continue;
    }
    if (!pointInVoronoi(sample)) {
      continue;
    }
    float density = getDensity(map, sample);
    mass += density;
    centerOfMass.x += density * sample.x;
    centerOfMass.y += density * sample.y;
  }

  if (!FloatEquals(mass, 0.0f)) {
    centerOfMass.x /= mass;
    centerOfMass.y /= mass;
  }
}

size_t VoronoiSolver::addVoronoi(const Voronoi &voronoi) {
  idCounter++;
  cells[idCounter] = voronoi;
  return idCounter;
}
size_t VoronoiSolver::addVoronoi(Vector2 position, double maxRadius) {
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

  for (auto &[_, cell] : cells) {
    cell.bounds.clear();
  }

  size_t i = -1, j;
  for (auto &[_, cellI] : cells) {
    i++;
    j = -1;
    for (auto &[_, cellJ] : cells) {
      j++;
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
      auto &v1 = cellI;
      auto &v2 = cellJ;

      int intersectionNumber =
          CirclesIntersects(v1.pos, v1.maxRadius, v2.pos, v2.maxRadius);
      if (intersectionNumber != 2) {
        continue;
      }

      const auto &intersections =
          CirclesIntersections(v1.pos, v1.maxRadius, v2.pos, v2.maxRadius);
      auto newBound1 =
          Voronoi::intersection_t{{intersections[0], intersections[1]}, &v2};
      v1.bounds.push_back(newBound1);
      auto newBound2 =
          Voronoi::intersection_t{{intersections[1], intersections[0]}, &v1};
      v2.bounds.push_back(newBound2);
    }
  }
  delete[] cache;
}

void VoronoiSolver::removeBoundsIntersections() {
  for (auto &[id, cell] : cells) {
    auto &bounds = cell.bounds;
    for (size_t j = 0; j < bounds.size(); ++j) {
      for (size_t k = 0; k < bounds.size(); ++k) {
        if (j == k) {
          continue;
        }

        auto &b1 = bounds[j];
        auto &b2 = bounds[k];

        if (!SegmentsIntersects(b1.segment, b2.segment)) {
          continue;
        }
        Vector2 point = SegmentIntersection(b1.segment, b2.segment);

        auto &v1 = b1.with;
        auto &v2 = b2.with;
        if (Vector2Distance(b1.segment.p1, v2->pos) <
            Vector2Distance(b1.segment.p2, v2->pos)) {
          b1.segment.p1 = point;
        } else {
          b1.segment.p2 = point;
        }
        if (Vector2Distance(b2.segment.p1, v1->pos) <
            Vector2Distance(b2.segment.p2, v1->pos)) {
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
  for (const auto &[id, cell] : cells) {
    DrawCircleLines(cell.pos.x, cell.pos.y, cell.maxRadius, RED);
    DrawCircle(cell.pos.x, cell.pos.y, 2.0, RED);

    for (const auto &bound : cell.bounds) {
      DrawLine(bound.segment.p1.x, bound.segment.p1.y, bound.segment.p2.x,
               bound.segment.p2.y, BLUE);
    }

    Vector2 CenterOfMass = cell.getLastCenterOfMass();
    DrawCircle(CenterOfMass.x, CenterOfMass.y, 3.0, DARKPURPLE);
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

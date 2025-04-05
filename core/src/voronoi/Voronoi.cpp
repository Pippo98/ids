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
  cells.push_back(voronoi);
  return cells.size() - 1;
}
size_t VoronoiSolver::addVoronoi(Vector2 position, double maxRadius) {
  cells.emplace_back(position, maxRadius);
  return cells.size() - 1;
}

void VoronoiSolver::findIntersections() {
  for (size_t i = 0; i < cells.size(); i++) {
    cells[i].bounds.clear();

    for (size_t j = i + 1; j < cells.size(); j++) {
      auto &v1 = cells[i];
      auto &v2 = cells[j];

      // first check how many intersections we have between two voronoi
      int intersectionNumber =
          CirclesIntersects(v1.pos, v1.maxRadius, v2.pos, v2.maxRadius);
      // if intersections == 0 or 1 we are not interested in computing the bound
      if (intersectionNumber != 2) {
        continue;
      }

      const auto &intersections =
          CirclesIntersections(v1.pos, v1.maxRadius, v2.pos, v2.maxRadius);
      v1.bounds.emplace_back(intersections[0], intersections[1], &v2);
      v2.bounds.emplace_back(intersections[1], intersections[0], &v1);
    }
  }
}

void VoronoiSolver::removeBoundsIntersections() {
  // We want to solve bound-bound intersections.
  // If necessary there is an optimization to do:
  // cell_i has a bound shared with cell_j, so when correcting
  // cell_i intersection, we could also update cell_j bounds
  for (size_t i = 0; i < cells.size(); i++) {
    auto &bounds = cells[i].bounds;
    for (size_t j = 0; j < bounds.size(); ++j) {
      for (size_t k = j + 1; k < bounds.size(); ++k) {
        auto &b1 = bounds[j];
        auto &b2 = bounds[k];

        // Check for b1-b2 intersection
        if (!SegmentsIntersects(b1.segment, b2.segment)) {
          continue;
        }

        // Get intersection point
        const Vector2 &point = SegmentIntersection(b1.segment, b2.segment);

        // check on both the bounds if we need to move p1 or p2
        if (Vector2Distance(b1.segment.p1, b2.with->pos) <
            Vector2Distance(b1.segment.p2, b2.with->pos)) {
          b1.segment.p1 = point;
        } else {
          b1.segment.p2 = point;
        }
        if (Vector2Distance(b2.segment.p1, b1.with->pos) <
            Vector2Distance(b2.segment.p2, b1.with->pos)) {
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
  for (size_t i = 0; i < cells.size(); i++) {
    const auto &cell = cells.at(i);

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

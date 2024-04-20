#pragma once

#include <vector>

#include "geometry/Geometry.hpp"
#include "raylib.h"

class Voronoi {
 public:
  Voronoi() = default;
  Voronoi(Vector2 pos, double radius);
  void setPosition(Vector2 pos);
  void setMaxRadius(double radius);

  Vector2 getPosition() const { return pos; };
  double getMaxRadius() const { return maxRadius; };

  bool operator==(const Voronoi &other);

  bool hasIntersection() const { return !bounds.empty(); }
  Vector2 getCenterOfMass() const { return Vector2(); }

  friend class VoronoiSolver;
  struct intersection_t {
    segment_t segment;
    Voronoi &with;
  };

 private:
  Vector2 pos;
  double maxRadius;
  std::vector<intersection_t> bounds;
};

class VoronoiSolver {
 public:
  Voronoi &addVoronoi(const Voronoi &voronoi);
  Voronoi &addVoronoi(Vector2 position, double maxRadius);

  bool solve();

  void draw() const;

 private:
  void findIntersections();
  void removeBoundsIntersections();

 private:
  std::vector<Voronoi> cells;
};

VoronoiSolver VoronoiTest();

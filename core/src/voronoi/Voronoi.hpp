#pragma once

#include <map>

#include "geometry/Geometry.hpp"
#include "map/Map.hpp"
#include "raylib.h"

class Voronoi {
 public:
  Voronoi() = default;
  Voronoi(Vector2 pos, double radius);
  void setPosition(Vector2 pos);
  void setMaxRadius(float radius);

  Vector2 getPosition() const { return pos; };
  float getMaxRadius() const { return maxRadius; };

  bool operator==(const Voronoi &other);

  bool pointInVoronoi(const Vector2 &position) const;
  bool hasIntersection() const { return !bounds.empty(); }
  void calculateCenterOfMass() const;
  void calculateCenterOfMass(const Map &map) const;
  Vector2 getLastCenterOfMass() const { return centerOfMass; }

  friend class VoronoiSolver;
  struct intersection_t {
    segment_t segment;
    Voronoi *with;
    intersection_t(const Vector2 &p1, const Vector2 &p2, Voronoi *_with) {
      segment.p1 = p1;
      segment.p2 = p2;
      with = _with;
    }
  };

 private:
  mutable Vector2 centerOfMass;
  Vector2 pos;
  float maxRadius;
  std::vector<intersection_t> bounds;

 private:
  float getDensity(const Map &map, const Vector2 &position) const;
};

class VoronoiSolver {
 public:
  size_t addVoronoi(const Voronoi &voronoi);
  size_t addVoronoi(Vector2 position, double maxRadius);
  const std::vector<Voronoi> &getCells() const { return cells; };

  bool solve();
  void draw() const;

  Voronoi &getVoronoi(size_t id) { return cells[id]; }

 private:
  void findIntersections();
  void removeBoundsIntersections();

 private:
  std::vector<Voronoi> cells;
};

VoronoiSolver VoronoiTest();

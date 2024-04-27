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
  const std::map<size_t, Voronoi> &getCells() const { return cells; };

  bool solve();
  void draw() const;

  Voronoi &getVoronoi(size_t id) { return cells[id]; }

 private:
  void findIntersections();
  void removeBoundsIntersections();

 private:
  size_t idCounter;
  std::map<size_t, Voronoi> cells;
};

VoronoiSolver VoronoiTest();

#pragma once

#include <vector>

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

  friend class VoronoiSolver;

  struct segment_t {
    Vector2 p1;
    Vector2 p2;
  };

 private:
  Vector2 pos;
  double maxRadius;
  std::vector<segment_t> bounds;
};

class VoronoiSolver {
 public:
  Voronoi &addVoronoi(const Voronoi &voronoi);
  Voronoi &addVoronoi(Vector2 position, double maxRadius);

  bool solve();

 private:
  void findIntersections();

 private:
  struct cache_t {
    unsigned int visisted : 1;
  };
  std::vector<Voronoi> cells;
  std::vector<std::vector<cache_t>> cache;
};

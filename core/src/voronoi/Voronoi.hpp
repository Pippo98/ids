#pragma once

#include <vector>

#include "raylib.h"

class Voronoi {
 public:
  void setPosition(Vector2 pos);
  void setMaxRadius(double radius);

  Vector2 getPosition() const { return pos; };

 private:
  Vector2 pos;
  double maxRadius;
  std::vector<Vector2> bounds;
};

class VoronoiSolver {
 public:
  Voronoi &addVoronoi(const Voronoi &voronoi);
  Voronoi &addVoronoi(Vector2 position, double maxRadius);

  bool solve();

 private:
  std::vector<Voronoi> cells;
};

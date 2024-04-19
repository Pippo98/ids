#pragma once

#include <cstddef>

#include "raylib.h"

enum class TileType { EMPTY, VISITED, OBSTACLE };

class Map {
 public:
  ~Map();
  Map(Vector2 tl, Vector2 br, float resolution = 0.5);
  Map(Vector2 tl, Vector2 br, size_t cols, size_t rows);

  TileType getTileType(Vector2 position) const;
  float getConfidence(Vector2 position) const;

  float getResolution() const { return resolution; };

 private:
  Vector2 tl;
  Vector2 br;
  float resolution;

  size_t sizeX;
  size_t sizeY;

  float *confidenceMap;
  TileType *tileTypeMap;

 private:
  size_t mapX(float x) const;
  size_t mapY(float y) const;
};

#pragma once

#include <cstddef>

#include "raylib.h"

enum class TileType { EMPTY, VISITED, OBSTACLE };

class Map {
 public:
  ~Map();
  Map(Vector2 tl, Vector2 br, float resolution = 0.5);
  Map(Vector2 tl, Vector2 br, size_t cols, size_t rows);

  float getConfidence(Vector2 position) const;
  TileType getTileType(Vector2 position) const;

  void setTileType(Vector2 position, TileType type) const;
  void setConfidence(Vector2 position, float confidence) const;

  float getResolution() const { return resolution; };
  Vector2 getTopLeftCorner() const { return tl; };
  Vector2 getBottomRightCorner() const { return br; };

  void visitLocation(class Agent &agent) const;

  // Expose map limits
  Vector2 tl;
  Vector2 br;

 private:
  float resolution;

  size_t sizeX;
  size_t sizeY;

  float *confidenceMap;
  TileType *tileTypeMap;

 private:
  size_t mapX(float x) const;
  size_t mapY(float y) const;

  template <typename ArrType>
  ArrType &mapAtPosition(ArrType *arr, Vector2 position) const;
  template <typename ArrType>
  ArrType &mapAtIndex(ArrType *arr, size_t column, size_t row) const;
};

#pragma once

#include <cstddef>
#include <vector>

#include "raylib.h"

enum class TileType { EMPTY, VISITED, OBSTACLE };

class Agent;

class Map {
 public:
  ~Map();
  Map(Vector2 tl, Vector2 br, float resolution = 0.5);
  Map(Vector2 tl, Vector2 br, size_t cols, size_t rows);

  float getConfidence(Vector2 position) const;
  TileType getTileType(Vector2 position) const;

  void setTileType(Vector2 position, TileType type);
  void setConfidence(Vector2 position, float confidence);

  float getResolution() const { return resolution; };
  Vector2 getTopLeftCorner() const { return tl; };
  Vector2 getBottomRightCorner() const { return br; };
  Vector2 getSize() const { return Vector2{br.x - tl.x, tl.y - br.y}; }

  void visitLocation(const Agent &agent);

  // Expose map limits
  Vector2 tl;
  Vector2 br;

 private:
  float resolution;

  size_t sizeX;
  size_t sizeY;

  std::vector<float> confidenceMap;
  std::vector<TileType> tileTypeMap;

 private:
  size_t mapX(float x) const;
  size_t mapY(float y) const;

  template <typename ArrType>
  const ArrType &mapAtIndex(const std::vector<ArrType> &arr, size_t column,
                            size_t row) const;
  template <typename ArrType>
  ArrType &mapAtIndex(std::vector<ArrType> &arr, size_t column,
                      size_t row) const;
  template <typename ArrType>
  const ArrType &mapAtPosition(const std::vector<ArrType> &arr,
                               Vector2 position) const;
  template <typename ArrType>
  ArrType &mapAtPosition(std::vector<ArrType> &arr, Vector2 position) const;
};

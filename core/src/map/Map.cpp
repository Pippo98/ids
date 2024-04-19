#include "Map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

template <typename ArrType>
ArrType &Map::mapAtIndex(ArrType *arr, size_t x, size_t y) const {
  return arr[y * sizeX + x];
}
template <typename ArrType>
ArrType &Map::mapAtPosition(ArrType *arr, Vector2 position) const {
  size_t x = mapX(position.x);
  size_t y = mapY(position.y);
  return mapAtIndex(arr, x, y);
}

Map::Map(Vector2 tl, Vector2 br, float resolution)
    : tl(tl), br(br), resolution(resolution) {
  sizeX = (br.x - tl.x) / resolution;
  sizeY = (tl.y - br.y) / resolution;
  confidenceMap = new float[sizeX * sizeY];
  tileTypeMap = new TileType[sizeX * sizeY];
}
Map::Map(Vector2 tl, Vector2 br, size_t cols, size_t rows) {
  float horizontalResolution = (br.x - tl.x) / cols;
  float verticalResolution = (tl.y - br.y) / rows;

  float minResolution = std::min(horizontalResolution, verticalResolution);
  *this = Map(tl, br, minResolution);
}
Map::~Map() {
  delete[] confidenceMap;
  delete[] tileTypeMap;
}

size_t Map::mapX(float x) const {
  x = std::clamp(x, tl.x, br.x);
  x -= tl.x;
  size_t idx = std::floor(x / resolution);
  idx = std::clamp(idx, (size_t)0, sizeX - 1);
  return idx;
}
size_t Map::mapY(float y) const {
  y = std::clamp(y, br.y, tl.y);
  y -= br.y;
  size_t idx = std::floor(y / resolution);
  idx = std::clamp(idx, (size_t)0, sizeY - 1);
  return idx;
}

float Map::getConfidence(Vector2 position) const {
  return mapAtPosition(confidenceMap, position);
}
TileType Map::getTileType(Vector2 position) const {
  return mapAtPosition(tileTypeMap, position);
}

void Map::setTileType(Vector2 position, TileType type) {
  mapAtPosition(tileTypeMap, position) = type;
}
void Map::setConfidence(Vector2 position, float confidence) {
  mapAtPosition(confidenceMap, position) = confidence;
}

#include "Map.hpp"

#include <algorithm>

template <typename ArrType>
ArrType &mapAtIndex(ArrType *arr, size_t width, size_t column, size_t row) {
  return arr[row * width + column];
}

Map::Map(Vector2 tl, Vector2 br, float resolution) {
  sizeX = (br.x - tl.y) / resolution;
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
  x = std::max(x, tl.x);
  x = std::min(x, br.x);
  return x / resolution;
}
size_t Map::mapY(float y) const {
  y = std::max(y, br.x);
  y = std::max(y, tl.x);
  return y / resolution;
}

TileType Map::getTileType(Vector2 position) const {
  size_t x = mapX(position.x);
  size_t y = mapY(position.y);
  return mapAtIndex(tileTypeMap, sizeX, x, y);
}
float Map::getConfidence(Vector2 position) const {
  size_t x = mapX(position.x);
  size_t y = mapY(position.y);
  return mapAtIndex(confidenceMap, sizeX, x, y);
}

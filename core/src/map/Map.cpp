#include "Map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "agent/Agent.hpp"
#include "raymath.h"

template <typename ArrType>
const ArrType &Map::mapAtIndex(const std::vector<ArrType> &arr, size_t x,
                               size_t y) const {
  return arr[y * sizeX + x];
}
template <typename ArrType>
ArrType &Map::mapAtIndex(std::vector<ArrType> &arr, size_t x, size_t y) const {
  return arr[y * sizeX + x];
}
template <typename ArrType>
const ArrType &Map::mapAtPosition(const std::vector<ArrType> &arr,
                                  Vector2 position) const {
  size_t x = mapX(position.x);
  size_t y = mapY(position.y);
  return mapAtIndex(arr, x, y);
}
template <typename ArrType>
ArrType &Map::mapAtPosition(std::vector<ArrType> &arr, Vector2 position) const {
  size_t x = mapX(position.x);
  size_t y = mapY(position.y);
  return mapAtIndex(arr, x, y);
}

Map::Map(Vector2 _tl, Vector2 _br, float resolution)
    : tl(_tl), br(_br), resolution(resolution) {
  if (tl.x > br.x || tl.y < br.y) {
    throw std::logic_error("Invalid top left or/and bottom right corners");
  }
  if (resolution < 0.0f) {
    throw std::logic_error("Invalid resolution");
  }

  sizeX = (br.x - tl.x) / resolution;
  sizeY = (tl.y - br.y) / resolution;

  confidenceMap = std::vector<float>(sizeX * sizeY, float());
  tileTypeMap = std::vector<TileType>(sizeX * sizeY, TileType::EMPTY);
}
Map::Map(Vector2 tl, Vector2 br, size_t cols, size_t rows) {
  const float horizontalResolution = (br.x - tl.x) / cols;
  const float verticalResolution = (tl.y - br.y) / rows;

  float minResolution = std::min(horizontalResolution, verticalResolution);
  *this = Map(tl, br, minResolution);
}
Map::~Map() {}

size_t Map::mapX(float x) const {
  x = std::clamp(x, tl.x, br.x);
  x -= tl.x;
  size_t idx = std::round(x / resolution);
  idx = std::clamp(idx, (size_t)0, sizeX - 1);
  return idx;
}
size_t Map::mapY(float y) const {
  y = std::clamp(y, br.y, tl.y);
  y -= br.y;
  size_t idx = std::round(y / resolution);
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

void Map::visitLocation(const Agent &agent) {
  const Vector2 agentPosXY{static_cast<float>(agent.GetPosition().x),
                           static_cast<float>(agent.GetPosition().y)};
  if (getTileType(agentPosXY) == TileType::EMPTY) {
    setTileType(agentPosXY, TileType::VISITED);
  }
  // Update confidence of nearby tiles with a gaussian distribution around the
  // agent with radius watchRadius
  for (float x = agentPosXY.x - agent.GetWatchRadius();
       x < agentPosXY.x + agent.GetWatchRadius(); x += resolution) {
    for (float y = agentPosXY.y - agent.GetWatchRadius();
         y < agentPosXY.y + agent.GetWatchRadius(); y += resolution) {
      const Vector2 tile = {x, y};
      const float distance = Vector2Distance(tile, agentPosXY);

      if (distance < agent.GetWatchRadius()) {
        const float confidence = -(1 - distance / agent.GetWatchRadius());

        float currentConfidence = getConfidence(tile);
        float newConfidence = currentConfidence + confidence;
        if (newConfidence < -100) {
          setConfidence(tile, -100);
        } else {
          setConfidence(tile, newConfidence);
        }
      }
    }
  }
}

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
  if (getTileType({agent.GetPosition().x, agent.GetPosition().y}) ==
      TileType::EMPTY) {
    setTileType({agent.GetPosition().x, agent.GetPosition().y},
                TileType::VISITED);
  }
  std::vector<float *> visited;
  // Update confidence of nearby tiles with a gaussian distribution around the
  // agent with radius watchRadius
  for (float x = agent.GetPosition().x - agent.GetWatchRadius();
       x < agent.GetPosition().x + agent.GetWatchRadius(); x += 0.5) {
    for (float y = agent.GetPosition().y - agent.GetWatchRadius();
         y < agent.GetPosition().y + agent.GetWatchRadius(); y += 0.5) {
      float distance = Vector2Distance(
          {x, y}, {agent.GetPosition().x, agent.GetPosition().y});
      float confidence = -(1 - distance / agent.GetWatchRadius());

      if (distance < agent.GetWatchRadius()) {
        // TODO: Fix performance issue
        if (std::find(visited.begin(), visited.end(),
                      &mapAtPosition(confidenceMap, {x, y})) != visited.end()) {
          continue;
        }

        visited.push_back(&mapAtPosition(confidenceMap, {x, y}));

        float currentConfidence = getConfidence({x, y});
        float newConfidence = currentConfidence + confidence;
        if (newConfidence < -100) {
          setConfidence({x, y}, -100);
        } else {
          setConfidence({x, y}, newConfidence);
        }
      }
    }
  }
}

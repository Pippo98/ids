#include "Map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "agent/Agent.hpp"
#include "raymath.h"

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

Map::Map(Vector2 _tl, Vector2 _br, float resolution)
    : tl(_tl), br(_br), resolution(resolution) {
  sizeX = (br.x - tl.x) / resolution;
  sizeY = (tl.y - br.y) / resolution;
  confidenceMap = new float[sizeX * sizeY];
  tileTypeMap = new TileType[sizeX * sizeY];
  memset(confidenceMap, 0, sizeX * sizeY * sizeof(float));
  memset(tileTypeMap, (int)TileType::EMPTY, sizeX * sizeY * sizeof(TileType));
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

void Map::setTileType(Vector2 position, TileType type) const {
  mapAtPosition(tileTypeMap, position) = type;
}
void Map::setConfidence(Vector2 position, float confidence) const {
  mapAtPosition(confidenceMap, position) = confidence;
}

void Map::visitLocation(Agent &agent) const {
  if (getTileType({agent.GetPosition().x, agent.GetPosition().y}) ==
      TileType::EMPTY) {
    setTileType({agent.GetPosition().x, agent.GetPosition().y},
                TileType::VISITED);
  }
  std::vector<float *> visited;
  // Update confidence of nearby tiles with a gaussian distribution around the
  // agent with radiun watchRadius
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
                      &mapAtPosition(confidenceMap, {x, y})) != visited.end())
          continue;
        visited.push_back(&mapAtPosition(confidenceMap, {x, y}));
        float currentConfidence = getConfidence({x, y});
        float newConfidence = currentConfidence + confidence;
        if (newConfidence < -100) {
          setConfidence({x, y}, -100);
          continue;
        }
        setConfidence({x, y}, newConfidence);
      }
    }
  }
}

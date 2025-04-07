#include "ControlModel.hpp"

#include "position/PositionModel.hpp"
#include "raylib.h"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

template <>
std::pair<bool, PositionModelXY>
ControlModelVoronoi<PositionModelXY>::computeTargetPosition(
    const Map &map, const Voronoi &cell) {
  cell.calculateCenterOfMass(map);
  constexpr Vector2 origin = {0.0f, 0.0f};

  const auto &isValidTarget = [=](const Vector2 &target,
                                  const Voronoi &cell) -> bool {
    return !Vector2Equals(target, origin) &&
           Vector2Distance(target, cell.getPosition()) > 10.0;
  };
  const auto &projectToBoundary = [](const Vector2 &target,
                                     const Voronoi &cell) -> Vector2 {
    auto norm = Vector2Normalize(Vector2Subtract(target, cell.getPosition()));
    return Vector2Add(Vector2Scale(norm, cell.getMaxRadius()),
                      cell.getPosition());
  };

  Vector2 com;
  com = cell.getPositiveCenterOfMass();
  if (isValidTarget(com, cell)) {
    com = projectToBoundary(com, cell);
    return {true, com};
  }
  com = cell.getCenterOfMass();
  if (isValidTarget(com, cell)) {
    com = projectToBoundary(com, cell);
    return {true, com};
  }
  return {true, origin};
}

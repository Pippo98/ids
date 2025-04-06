#include "ControlModel.hpp"

#include "position/PositionModel.hpp"

template <>
std::pair<bool, PositionModelXY>
ControlModelVoronoi<PositionModelXY>::computeTargetPosition(
    const Map &map, const Voronoi &cell) {
  cell.calculateCenterOfMass(map);

  const Vector2 &com = cell.getLastCenterOfMass();
  return {true, com};
}

#pragma once

#include "voronoi/Voronoi.hpp"

template <class PositionModel>
class ControlModelVoronoi {
 public:
  std::pair<bool, PositionModel> computeTargetPosition(const Map &map,
                                                       const Voronoi &cell);
};

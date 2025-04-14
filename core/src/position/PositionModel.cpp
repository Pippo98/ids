#include "PositionModel.hpp"

#include "raymath.h"

void PositionModelXY::Move(PositionModelXY &target, double maxSpeed,
                           double dt) {
  double dist = Vector2Distance(target, *this);
  *this = Vector2MoveTowards(*this, target, std::min(dist, maxSpeed * dt));
}

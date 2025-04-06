#include "PositionModel.hpp"

void PositionModelXY::Move(PositionModelXY &target, double maxSpeed,
                           double dt) {
  this->x += (target.x - x) * maxSpeed * dt;
  this->y += (target.y - y) * maxSpeed * dt;
}

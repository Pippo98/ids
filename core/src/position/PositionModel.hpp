#pragma once

#include "raylib.h"
class PositionModelXY {
 public:
  double x;
  double y;

  PositionModelXY() = default;
  PositionModelXY(double _x, double _y) : x(_x), y(_y) {}
  PositionModelXY(Vector2 vec) : x(vec.x), y(vec.y) {}

  operator Vector2() const {
    return Vector2{static_cast<float>(x), static_cast<float>(y)};
  }

  void Move(PositionModelXY &target, double maxSpeed, double dt);
};

class PositionModelXYPsi {
 public:
  double m_x;
  double m_y;
  double m_psi;
};

/*class PositionModelXYZ {*/
/* public:*/
/*  double m_x;*/
/*  double m_y;*/
/*  double m_z;*/
/*};*/
/**/
/*class PositionModelXYZPsiTheta {*/
/* public:*/
/*  double m_x;*/
/*  double m_y;*/
/*  double m_z;*/
/*  double m_psi;*/
/*  double m_theta;*/
/*};*/

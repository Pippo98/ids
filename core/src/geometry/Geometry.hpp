#pragma once

#include <vector>

#include "raylib.h"

struct segment_t {
  Vector2 p1;
  Vector2 p2;
};

bool SegmentsIntersects(const segment_t &s1, const segment_t &s2);
Vector2 SegmentIntersection(const segment_t &s1, const segment_t &s2);

int CirclesIntersects(const Vector2 &c1, float r1, const Vector2 &c2, float r2);
std::vector<Vector2> CirclesIntersections(const Vector2 &c1, float r1,
                                          const Vector2 &c2, float r2);

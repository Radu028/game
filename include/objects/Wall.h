#pragma once

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Wall : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

 public:
  Wall(Vector3 position, Vector3 dimensions, Color color,
       bool hasCollisions = true);

  void draw() const { DrawCube(position, 20.0f, 20.0f, 20.0f, RED); }
  BoundingBox getBoundingBox() const { return {0.0f, 0.0f}; }
};
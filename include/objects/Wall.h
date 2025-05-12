#pragma once

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Wall : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

 public:
  Wall(Vector3 position, Vector3 dimensions, Color color,
       bool hasCollisions = true);
};
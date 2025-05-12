#pragma once

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Floor : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

 public:
  Floor(Vector3 position, Vector3 dimensions, Color color,
        bool hasCollision = true);
};
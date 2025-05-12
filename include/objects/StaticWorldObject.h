#pragma once

#include "objects/GameObject.h"

class GameObject;

class StaticWorldObject : public GameObject {
 public:
  StaticWorldObject(Vector3 position, bool hasCollision = true);

  void draw() const { DrawCube(position, 20.0f, 20.0f, 20.0f, RED); }
  BoundingBox getBoundingBox() const { return {0.0f, 0.0f}; }
};
#pragma once

#include "objects/GameObject.h"

class GameObject;

class StaticWorldObject : public GameObject {
 public:
  StaticWorldObject(Vector3 position, bool hasCollision = true);
};
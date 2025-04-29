#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H

#include "GameObject.h"
#include "Physics.h"
#include "raylib.h"

class PhysicsSystem {
 public:
  static void applyGravity(GameObject& obj, float deltaTime);
  static void moveWithSliding(GameObject& obj, const Vector3& desiredMove);
};

#endif

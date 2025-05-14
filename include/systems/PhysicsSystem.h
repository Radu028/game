#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H

// #include <memory>
#include <vector>

#include "objects/GameObject.h"
#include "raylib.h"
#include "settings/Physics.h"

class GameWorld;

class PhysicsSystem {
 private:
  std::vector<GameObject*> physicsObjects;
  GameWorld* world;

  // Checks if obj would collide with any other relevant object in the world if
  // it were at futurePosition. This is crucial for predictive collision
  // checking.
  // bool checkCollisionWithWorld(GameObject& obj,
  //                              const Vector3& futurePosition) const;

  // A helper method to perform a continuous collision detection sweep (like a
  // binary search on the movement path) to find the exact point of contact.
  // float getContactTime(GameObject& obj, const Vector3& movementVector) const;

 public:
  PhysicsSystem(GameWorld* gameWorld);

  void addObject(GameObject* obj);
  void removeObject(GameObject* obj);

  void update(float deltaTime);

 private:
  void applyGravityToObject(GameObject& obj, float deltaTime);
};

#endif

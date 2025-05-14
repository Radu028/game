#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include <memory>  // For std::shared_ptr

#include "GameWorld.h"
#include "raylib.h"
#include "settings/Physics.h"

class GameWorld;

class GameObject {
 protected:
  Vector3 position;
  Vector3 velocity;
  bool isOnGround;
  bool affectedByGravity;
  bool isStatic;

  bool hasCollision;

 public:
  GameObject(Vector3 position, bool hasCollision = true,
             bool affectedByGravity = true, bool isStatic = false);
  virtual ~GameObject() = default;

  Vector3 getPosition() const { return position; }
  Vector3 getVelocity() const { return velocity; }
  bool getIsOnGround() const { return isOnGround; }
  bool isAffectedByGravity() const { return affectedByGravity; }
  bool getIsStatic() const { return isStatic; }
  bool getHasCollision() const { return hasCollision; }

  void setPosition(Vector3 newPosition) { position = newPosition; }
  void setVelocity(Vector3 newVelocity) { velocity = newVelocity; }
  void setIsOnGround(bool onGround) { isOnGround = onGround; }

  virtual void draw() const = 0;
  virtual BoundingBox getBoundingBox() const = 0;

  virtual void update(float deltaTime) {};
  virtual void interact() {};
  virtual bool checkCollisionWith(const BoundingBox& otherBox) const;
  virtual float getVerticalCollisionContactTime(
      const Vector3& verticalMovementVector, const GameWorld* world,
      int maxIterations) const;

  bool checkCollision(const GameObject& other) const;
  float getDistance(const GameObject& other) const;
};

#endif
#include "systems/PhysicsSystem.h"

#include <algorithm>

#include "GameWorld.h"
#include "entities/Player.h"
#include "objects/GameObject.h"
#include "raymath.h"
#include "settings/Physics.h"

PhysicsSystem::PhysicsSystem(GameWorld* gameWorld) : world(gameWorld) {}

void PhysicsSystem::addObject(GameObject* obj) {
  if (obj && !obj->getIsStatic() && obj->isAffectedByGravity()) {
    physicsObjects.push_back(obj);
  }
}

void PhysicsSystem::removeObject(GameObject* obj) {
  physicsObjects.erase(
      std::remove(physicsObjects.begin(), physicsObjects.end(), obj),
      physicsObjects.end());
}

void PhysicsSystem::update(float deltaTime) {
  for (GameObject* obj : physicsObjects) {
    if (obj) {
      applyGravityToObject(*obj, deltaTime);
    }
  }
}

void PhysicsSystem::applyGravityToObject(GameObject& obj, float deltaTime) {
  if (obj.getIsStatic() || !obj.isAffectedByGravity()) {
    return;
  }

  // Apply gravitational acceleration
  Vector3 currentVelocity = obj.getVelocity();
  bool justJumped = false;
  Player* playerObj = dynamic_cast<Player*>(&obj);
  if (playerObj && !playerObj->getIsOnGround() && currentVelocity.y > 0.0f) {
    if (std::abs(currentVelocity.y - PhysicsSettings::JUMP_VELOCITY) <
        EPSILON) {
    }
  }

  currentVelocity.y += PhysicsSettings::GRAVITY_ACCELERATION * deltaTime;

  // Clamp fall speed (terminal velocity)
  if (currentVelocity.y < PhysicsSettings::MAX_FALL_SPEED) {
    currentVelocity.y = PhysicsSettings::MAX_FALL_SPEED;
  }

  Vector3 currentPosition = obj.getPosition();
  float verticalDeltaThisFrame = currentVelocity.y * deltaTime;
  Vector3 verticalMovementVector = {0.0f, verticalDeltaThisFrame, 0.0f};

  // TODO: Try implement swept collision detection.

  float contactTimeFactor = 1.0f;
  if (std::abs(verticalDeltaThisFrame) > EPSILON) {
    contactTimeFactor = obj.getVerticalCollisionContactTime(
        verticalMovementVector, world, PhysicsSettings::MAX_PHYSICS_ITERATIONS);
  }

  Vector3 actualDisplacement =
      Vector3Scale(verticalMovementVector, contactTimeFactor);
  Vector3 finalPosition = Vector3Add(currentPosition, actualDisplacement);
  obj.setPosition(finalPosition);

  bool prelimOnGround = false;
  Vector3 resolvedVelocity = currentVelocity;

  if (contactTimeFactor < 1.0f - EPSILON) {
    if (currentVelocity.y <= 0) {
      resolvedVelocity.y = 0;
      prelimOnGround = true;
    } else {
      resolvedVelocity.y = 0;
      prelimOnGround = false;
    }
  } else {
    if (obj.getIsOnGround() && resolvedVelocity.y <= 0.0f &&
        std::abs(verticalDeltaThisFrame) <
            PhysicsSettings::GROUND_STICK_DETACH_THRESHOLD) {
      prelimOnGround = true;
      resolvedVelocity.y = 0;
    } else {
      prelimOnGround = false;
    }
  }

  if (!playerObj) {
    obj.setIsOnGround(prelimOnGround);
  }
  obj.setVelocity(resolvedVelocity);
}
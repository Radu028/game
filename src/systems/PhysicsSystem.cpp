#include "systems/PhysicsSystem.h"

#include <algorithm>

#include "GameWorld.h"
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

bool PhysicsSystem::checkCollisionWithWorld(
    GameObject& obj, const Vector3& futurePosition) const {
  if (!world) return false;

  BoundingBox objOriginalBox = obj.getBoundingBox();
  Vector3 currentPosition = obj.getPosition();

  Vector3 displacement = Vector3Subtract(futurePosition, currentPosition);

  BoundingBox objTestBox = {Vector3Add(objOriginalBox.min, displacement),
                            Vector3Add(objOriginalBox.max, displacement)};

  for (const auto& otherSharedPtr : world->getObjects()) {
    GameObject* other = otherSharedPtr.get();
    if (!other || other == &obj || !other->getHasCollision()) {
      continue;
    }

    if (CheckCollisionBoxes(objTestBox, other->getBoundingBox())) {
      return true;  // Collision detected
    }
  }
  return false;
}

float PhysicsSystem::getContactTime(GameObject& obj,
                                    const Vector3& movementVector) const {
  if (!world) return 1.0f;

  Vector3 originalPosition = obj.getPosition();
  float t0 = 0.0f;
  float t1 = 1.0f;
  float tMid;

  const int MAX_ITERATIONS = 10;

  Vector3 fullTargetPosition = Vector3Add(originalPosition, movementVector);

  if (!checkCollisionWithWorld(obj, fullTargetPosition)) {
    return 1.0f;
  }

  for (int i = 0; i < MAX_ITERATIONS && (t1 - t0) > EPSILON; i++) {
    tMid = (t0 + t1) / 2.0f;

    Vector3 testMovementAtTMid = Vector3Scale(movementVector, tMid);
    Vector3 hypotheticalPositionAtTMid =
        Vector3Add(originalPosition, testMovementAtTMid);

    if (checkCollisionWithWorld(obj, hypotheticalPositionAtTMid)) {
      t1 = tMid;
    } else {
      t0 = tMid;
    }
  }
  return t0;  // Return the latest time t0 where no collision was found
}

void PhysicsSystem::applyGravityToObject(GameObject& obj, float deltaTime) {
  if (obj.getIsStatic() || !obj.isAffectedByGravity()) {
    return;
  }

  // Apply gravitational acceleration
  Vector3 currentVelocity = obj.getVelocity();
  currentVelocity.y += PhysicsSettings::GRAVITY_ACCELERATION * deltaTime;

  // Clamp fall speed (terminal velocity)
  if (currentVelocity.y < PhysicsSettings::MAX_FALL_SPEED) {
    currentVelocity.y = PhysicsSettings::MAX_FALL_SPEED;
  }

  Vector3 currentPosition = obj.getPosition();
  float verticalDeltaThisFrame = currentVelocity.y * deltaTime;
  Vector3 verticalMovementVector = {0.0f, verticalDeltaThisFrame, 0.0f};

  // TODO: Try implement swept collision detection.

  float contactT = getContactTime(obj, verticalMovementVector);

  Vector3 actualDisplacement = Vector3Scale(verticalMovementVector, contactT);
  Vector3 finalPosition = Vector3Add(currentPosition, actualDisplacement);
  obj.setPosition(finalPosition);

  if (contactT < 1.0f && currentVelocity.y <= 0) {
    obj.setIsOnGround(true);
    currentVelocity.y = 0;
  } else {
    obj.setIsOnGround(false);
    if (contactT < 1.0f && currentVelocity.y > 0) {
      currentVelocity.y = 0;
    }
  }
  obj.setVelocity(currentVelocity);
}
#include "GameObject.h"

#include <cmath>

GameObject::GameObject(Vector3 position, bool hasCollision,
                       bool affectedByGravity, bool isStatic)
    : position(position),
      velocity({0.0f, 0.0f, 0.0f}),
      isOnGround(false),
      affectedByGravity(affectedByGravity),
      isStatic(isStatic),
      hasCollision(hasCollision) {}

bool GameObject::checkCollision(const GameObject& other) const {
  if (!this->hasCollision || !other.getHasCollision()) {
    return false;
  }
  return CheckCollisionBoxes(this->getBoundingBox(), other.getBoundingBox());
}

bool GameObject::checkCollisionWith(const BoundingBox& otherBox) const {
  if (!hasCollision) return false;
  return CheckCollisionBoxes(getBoundingBox(), otherBox);
}

float GameObject::getDistance(const GameObject& other) const {
  return sqrtf((other.position.x - this->position.x) *
                   (other.position.x - this->position.x) +
               (other.position.y - this->position.y) *
                   (other.position.y - this->position.y) +
               (other.position.z - this->position.z) *
                   (other.position.z - this->position.z));
}

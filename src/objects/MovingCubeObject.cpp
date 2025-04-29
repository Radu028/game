#include "objects/MovingCubeObject.h"

#include <cmath>

MovingCubeObject::MovingCubeObject(Vector3 position, float width, float height,
                                   float length, Color color, bool hasCollision,
                                   Vector3 velocity, Vector3 minBounds,
                                   Vector3 maxBounds, const char* texturePath)
    : CubeObject(position, width, height, length, color, hasCollision,
                 texturePath),
      velocity(velocity),
      minBounds(minBounds),
      maxBounds(maxBounds) {}

MovingCubeObject::MovingCubeObject(const MovingCubeObject& other)
    : CubeObject(other),
      velocity(other.velocity),
      minBounds(other.minBounds),
      maxBounds(other.maxBounds) {}

std::shared_ptr<GameObject> MovingCubeObject::clone() const {
  return std::make_shared<MovingCubeObject>(*this);
}

void MovingCubeObject::update(float deltaTime) {
  // Update position based on velocity
  position.x += velocity.x * deltaTime;
  position.y += velocity.y * deltaTime;
  position.z += velocity.z * deltaTime;

  // Bounce against boundaries
  if (position.x < minBounds.x || position.x > maxBounds.x) {
    velocity.x = -velocity.x;
    position.x = (position.x < minBounds.x) ? minBounds.x : maxBounds.x;
  }

  if (position.y < minBounds.y || position.y > maxBounds.y) {
    velocity.y = -velocity.y;
    position.y = (position.y < minBounds.y) ? minBounds.y : maxBounds.y;
  }

  if (position.z < minBounds.z || position.z > maxBounds.z) {
    velocity.z = -velocity.z;
    position.z = (position.z < minBounds.z) ? minBounds.z : maxBounds.z;
  }
}

void MovingCubeObject::interact() {
  // Change velocity direction on interaction
  velocity.x = -velocity.x;
  velocity.z = -velocity.z;

  // Also change color like the parent class
  CubeObject::interact();
}
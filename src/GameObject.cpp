#include "GameObject.h"

#include <cmath>

// GameObject::GameObject(Vector3 position, Vector3 size, Color color, bool
// hasCollision)
//     : position(position), size(size), color(color),
//     hasCollision(hasCollision) {}

GameObject::GameObject(Vector3 position, bool hasCollision)
    : position(position), hasCollision(hasCollision) {}

// BoundingBox GameObject::getBoundingBox() const {
//     return (BoundingBox){
//         (Vector3){position.x - size.x / 2, position.y - size.y / 2,
//         position.z - size.z / 2}, (Vector3){position.x + size.x / 2,
//         position.y + size.y / 2, position.z + size.z / 2},
//     };
// }

// void GameObject::draw() const {
//     DrawCube(this->position, this->size.x, this->size.y, this->size.z,
//     this->color);

//     Color wireColor = RED;
//     if (this->color.r == 255) wireColor = GREEN;

//     BoundingBox box = this->getBoundingBox();
//     DrawBoundingBox(box, wireColor);
// }

bool GameObject::checkCollision(const GameObject& other) const {
  if (!hasCollision || !other.hasCollision) {
    return false;
  }

  BoundingBox boundingBox = getBoundingBox();
  BoundingBox otherBoundingBox = other.getBoundingBox();

  return CheckCollisionBoxes(boundingBox, otherBoundingBox);
}

float GameObject::getDistance(const GameObject& other) const {
  Vector3 otherPos = other.getPosition();

  float dx = position.x - otherPos.x;
  float dy = position.y - otherPos.y;
  float dz = position.z - otherPos.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// void GameObject::handleCollision(GameObject& other) {
//     // For derivated classes to overwrite this.
// }
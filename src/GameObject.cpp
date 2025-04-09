#include "GameObject.h"

GameObject::GameObject(Vector3 position, Vector3 size, Color color, bool hasCollison = true)
    : position(position), size(size), color(color), hasCollision(hasCollision) {}

BoundingBox GameObject::getBoundingBox() const {
    return BoundingBox{
        (Vector3){position.x - size.x / 2, position.y - size.y / 2, position.z - size.z / 2},
        (Vector3){position.x + size.x / 2, position.y + size.y / 2, position.z + size.z / 2},
    };
}

void GameObject::draw() const {
    DrawCube(this->position, this->size.x, this->size.y, this->size.y, this->color);
}

bool GameObject::checkCollision(const GameObject& other) const {
    if (!this->hasCollision || !other.getHasCollision()) return false;
    return CheckCollisionBoxes(this->getBoundingBox(), other.getBoundingBox());
}

// void GameObject::handleCollision(GameObject& other) {
//     // For derivated classes to overwrite this.
// }
#include "GameObject.h"

#include <iostream>

GameObject::GameObject(Vector3 position, Vector3 size, Color color, bool hasCollision)
    : position(position), size(size), color(color), hasCollision(hasCollision) {}

BoundingBox GameObject::getBoundingBox() const {
    float expansionFactor = 0.0f;

    return (BoundingBox){
        (Vector3){position.x - size.x / 2 - expansionFactor,
                  position.y - size.y - expansionFactor,
                  position.z - size.z / 2 - expansionFactor},
        (Vector3){position.x + size.x / 2 + expansionFactor,
                  position.y + size.y + expansionFactor,
                  position.z + size.z / 2 + expansionFactor},
    };
}

void GameObject::draw() const {
    DrawCube(this->position, this->size.x, this->size.y, this->size.z, this->color);

    Color wireColor = RED;
    if (this->color.r == 255) wireColor = GREEN;

    BoundingBox box = this->getBoundingBox();
    DrawBoundingBox(box, wireColor);
}

bool GameObject::checkCollision(const GameObject& other) const {
    if (!this->hasCollision || !other.getHasCollision()) {
        return false;
    }

    BoundingBox box1 = this->getBoundingBox();
    BoundingBox box2 = other.getBoundingBox();

    bool result = CheckCollisionBoxes(box1, box2);

    // if (result) {
    //     std::cout << "COLIZIUNE DETECTATĂ între obiectele la pozițiile:" << std::endl;
    //     std::cout << "  (" << position.x << "," << position.y << "," << position.z << ") și"
    //               << std::endl;
    //     std::cout << "  (" << other.position.x << "," << other.position.y << "," <<
    //     other.position.z
    //               << ")" << std::endl;
    // }

    return result;
}

// void GameObject::handleCollision(GameObject& other) {
//     // For derivated classes to overwrite this.
// }
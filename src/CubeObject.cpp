#include "CubeObject.h"

CubeObject::CubeObject(Vector3 position, float width, float height, float length, Color color,
                       bool hasCollision)
    : GameObject(position, hasCollision),
      width(width),
      height(height),
      length(length),
      color(color) {}

BoundingBox CubeObject::getBoundingBox() const {
    return (BoundingBox){
        (Vector3){position.x - this->width / 2, position.y - height / 2, position.z - length / 2},
        (Vector3){position.x + this->width / 2, position.y + height / 2, position.z + length / 2},
    };
}

void CubeObject::draw() const {
    DrawCube(this->position, this->width, this->height, this->length, this->color);

    Color wireColor = RED;
    if (this->color.r == 255) wireColor = GREEN;

    BoundingBox box = this->getBoundingBox();
    DrawBoundingBox(box, wireColor);
}
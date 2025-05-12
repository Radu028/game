#include "BodyPart.h"

#include "raylib.h"

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision)
    : GameObject(position, hasCollision, false, false),
      size(size),
      color(color),
      rotationAxis({0.0f, 1.0f, 0.0f}),
      rotationAngle(0.0f) {}

void BodyPart::setRotation(const Vector3 axis, float angle) {
  rotationAxis = axis;
  rotationAngle = angle;
}

void BodyPart::setPosition(Vector3 newPos) { GameObject::setPosition(newPos); }

Vector3 BodyPart::getSize() const { return size; }
Color BodyPart::getColor() const { return color; }

void BodyPart::draw() const {
  // TODO: Reconsider this.
  DrawModelEx(LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z)), position,
              rotationAxis, rotationAngle, {1.0f, 1.0f, 1.0f}, color);
}

BoundingBox BodyPart::getBoundingBox() const {
  Vector3 currentPosition = getPosition();
  return (BoundingBox){
      (Vector3){currentPosition.x - size.x / 2, currentPosition.y - size.y / 2,
                currentPosition.z - size.z / 2},
      (Vector3){currentPosition.x + size.x / 2, currentPosition.y + size.y / 2,
                currentPosition.z + size.z / 2},
  };
}
#include "BodyPart.h"

#include "raylib.h"

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision)
    : GameObject(position, hasCollision, false, false),
      size(size),
      color(color),
      rotationAxis({0.0f, 1.0f, 0.0f}),
      rotationAngle(0.0f) {
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));
}

void BodyPart::setRotation(const Vector3 axis, float angle) {
  rotationAxis = axis;
  rotationAngle = angle;
}

void BodyPart::setPosition(Vector3 newPos) { GameObject::setPosition(newPos); }

Vector3 BodyPart::getSize() const { return size; }
Color BodyPart::getColor() const { return color; }

float BodyPart::getRotationAngle() const { return rotationAngle; }
Vector3 BodyPart::getRotationAxis() const { return rotationAxis; }

void BodyPart::draw() const {
  DrawModelEx(model, position, rotationAxis, rotationAngle, {1.0f, 1.0f, 1.0f},
              color);
}

BoundingBox BodyPart::getBoundingBox() const {
  Vector3 currentPosition = getPosition();
  // PERFORMANCE OPTIMIZATION: Cache half-size calculations to avoid repeated division
  float halfX = size.x * 0.5f;
  float halfY = size.y * 0.5f;
  float halfZ = size.z * 0.5f;
  
  return (BoundingBox){
      (Vector3){currentPosition.x - halfX, currentPosition.y - halfY, currentPosition.z - halfZ},
      (Vector3){currentPosition.x + halfX, currentPosition.y + halfY, currentPosition.z + halfZ},
  };
}
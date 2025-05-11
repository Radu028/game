#include "BodyPart.h"

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision)
    : CubeObject(position, size, color, hasCollision),
      rotationAxis({0.0f, 1.0f, 0.0f}),
      rotationAngle(0.0f) {}

void BodyPart::setRotation(const Vector3& axis, float angle) {
  rotationAxis = axis;
  rotationAngle = angle;
}

void BodyPart::setPosition(Vector3 newPos) const {
  const_cast<BodyPart*>(this)->position = newPos;
}

void BodyPart::draw() const {
  DrawModelEx(model, position, rotationAxis, rotationAngle, {1.0f, 1.0f, 1.0f},
              color);
}
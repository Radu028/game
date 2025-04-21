#ifndef BODYPART_H
#define BODYPART_H

#include "CubeObject.h"

class BodyPart : public CubeObject {
 private:
  Vector3 rotationAxis;
  float rotationAngle;

 public:
  BodyPart(Vector3 position, Vector3 size, Color color,
           bool hasCollision = false);

  void setRotation(const Vector3& axis, float angle);
  void setPosition(Vector3 newPos) const;
  void draw() const override;
};

#endif
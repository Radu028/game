#ifndef ROTATING_CUBE_OBJECT_H
#define ROTATING_CUBE_OBJECT_H

#include "CubeObject.h"

class RotatingCubeObject : public CubeObject {
 private:
  float rotationX;
  float rotationY;
  float rotationZ;
  float rotationSpeedX;
  float rotationSpeedY;
  float rotationSpeedZ;

 public:
  RotatingCubeObject(Vector3 position, float width, float height, float length,
                     Color color, bool hasCollision, float rotationSpeedX,
                     float rotationSpeedY, float rotationSpeedZ,
                     const char* texturePath = nullptr);

  // Copy constructor
  RotatingCubeObject(const RotatingCubeObject& other);

  // Clone method
  std::shared_ptr<GameObject> clone() const override;

  // Update method to handle rotation
  void update(float deltaTime) override;

  // Override draw to apply rotation
  void draw() const override;

  // Override interact to change rotation speed
  void interact() override;
};

#endif  // ROTATING_CUBE_OBJECT_H
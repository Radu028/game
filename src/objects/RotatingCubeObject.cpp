#include "objects/RotatingCubeObject.h"

#include <memory>  // For std::shared_ptr

#include "raylib.h"
#include "rlgl.h"

RotatingCubeObject::RotatingCubeObject(Vector3 position, float width,
                                       float height, float length, Color color,
                                       bool hasCollision, float rotationSpeedX,
                                       float rotationSpeedY,
                                       float rotationSpeedZ,
                                       const char* texturePath)
    : CubeObject(position, width, height, length, color, hasCollision,
                 texturePath),
      rotationX(0.0f),
      rotationY(0.0f),
      rotationZ(0.0f),
      rotationSpeedX(rotationSpeedX),
      rotationSpeedY(rotationSpeedY),
      rotationSpeedZ(rotationSpeedZ) {}

RotatingCubeObject::RotatingCubeObject(const RotatingCubeObject& other)
    : CubeObject(other),
      rotationX(other.rotationX),
      rotationY(other.rotationY),
      rotationZ(other.rotationZ),
      rotationSpeedX(other.rotationSpeedX),
      rotationSpeedY(other.rotationSpeedY),
      rotationSpeedZ(other.rotationSpeedZ) {}

std::shared_ptr<GameObject> RotatingCubeObject::clone() const {
  return std::make_shared<RotatingCubeObject>(*this);
}

void RotatingCubeObject::update(float deltaTime) {
  // Update rotation angles
  rotationX += rotationSpeedX * deltaTime;
  rotationY += rotationSpeedY * deltaTime;
  rotationZ += rotationSpeedZ * deltaTime;

  // Keep angles in the range [0, 360)
  if (rotationX >= 360.0f) rotationX -= 360.0f;
  if (rotationY >= 360.0f) rotationY -= 360.0f;
  if (rotationZ >= 360.0f) rotationZ -= 360.0f;
}

void RotatingCubeObject::draw() const {
  // Save current transform
  rlPushMatrix();

  // Translate to position
  rlTranslatef(position.x, position.y, position.z);

  // Apply rotations
  rlRotatef(rotationX, 1.0f, 0.0f, 0.0f);
  rlRotatef(rotationY, 0.0f, 1.0f, 0.0f);
  rlRotatef(rotationZ, 0.0f, 0.0f, 1.0f);

  // Draw the cube at the origin (since we've already translated)
  if (hasTexture) {
    // For textured cubes, draw with the model
    DrawModel(model, (Vector3){0, 0, 0}, 1.0f, color);
  } else {
    DrawCube((Vector3){0, 0, 0}, width, height, length, color);
  }

  // Restore transform
  rlPopMatrix();
}

void RotatingCubeObject::interact() {
  // Reverse rotation directions
  rotationSpeedX = -rotationSpeedX;
  rotationSpeedY = -rotationSpeedY;
  rotationSpeedZ = -rotationSpeedZ;

  // Change color on interaction
  color = BLUE;  // Change color instead of calling non-existent parent method
}
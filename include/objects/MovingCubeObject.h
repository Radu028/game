#ifndef MOVING_CUBE_OBJECT_H
#define MOVING_CUBE_OBJECT_H

#include <memory>  // For std::shared_ptr

#include "CubeObject.h"

class MovingCubeObject : public CubeObject {
 private:
  Vector3 velocity;
  Vector3 minBounds;
  Vector3 maxBounds;

 public:
  MovingCubeObject(Vector3 position, float width, float height, float length,
                   Color color, bool hasCollision, Vector3 velocity,
                   Vector3 minBounds, Vector3 maxBounds,
                   const char* texturePath = nullptr);

  // Copy constructor
  MovingCubeObject(const MovingCubeObject& other);

  // Clone method
  std::shared_ptr<GameObject> clone() const override;

  // Update method to handle movement
  void update(float deltaTime) override;

  // Interact method definition
  void interact() override;
};

#endif  // MOVING_CUBE_OBJECT_H
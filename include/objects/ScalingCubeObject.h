#ifndef SCALING_CUBE_OBJECT_H
#define SCALING_CUBE_OBJECT_H

#include <memory>  // For std::shared_ptr

#include "CubeObject.h"

class ScalingCubeObject : public CubeObject {
 private:
  float minScale;
  float maxScale;
  float scaleSpeed;
  float currentScale;
  bool growingPhase;

 public:
  ScalingCubeObject(Vector3 position, float width, float height, float length,
                    Color color, bool hasCollision, float minScale,
                    float maxScale, float scaleSpeed,
                    const char* texturePath = nullptr);

  // Copy constructor
  ScalingCubeObject(const ScalingCubeObject& other);

  // Clone method
  std::shared_ptr<GameObject> clone() const override;

  // Update method to handle scaling
  void update(float deltaTime) override;

  // Override draw to apply scale
  void draw() const override;

  // Override interact to change scaling direction
  void interact() override;

  // Override getBoundingBox to account for scaling
  BoundingBox getBoundingBox() const override;
};

#endif  // SCALING_CUBE_OBJECT_H
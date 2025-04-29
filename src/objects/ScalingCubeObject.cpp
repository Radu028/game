#include "objects/ScalingCubeObject.h"

ScalingCubeObject::ScalingCubeObject(Vector3 position, float width,
                                     float height, float length, Color color,
                                     bool hasCollision, float minScale,
                                     float maxScale, float scaleSpeed,
                                     const char* texturePath)
    : CubeObject(position, width, height, length, color, hasCollision,
                 texturePath),
      minScale(minScale),
      maxScale(maxScale),
      scaleSpeed(scaleSpeed),
      currentScale(1.0f),
      growingPhase(true) {}

ScalingCubeObject::ScalingCubeObject(const ScalingCubeObject& other)
    : CubeObject(other),
      minScale(other.minScale),
      maxScale(other.maxScale),
      scaleSpeed(other.scaleSpeed),
      currentScale(other.currentScale),
      growingPhase(other.growingPhase) {}

std::shared_ptr<GameObject> ScalingCubeObject::clone() const {
  return std::make_shared<ScalingCubeObject>(*this);
}

void ScalingCubeObject::update(float deltaTime) {
  // Update scale based on current phase
  if (growingPhase) {
    currentScale += scaleSpeed * deltaTime;
    if (currentScale >= maxScale) {
      currentScale = maxScale;
      growingPhase = false;
    }
  } else {
    currentScale -= scaleSpeed * deltaTime;
    if (currentScale <= minScale) {
      currentScale = minScale;
      growingPhase = true;
    }
  }
}

void ScalingCubeObject::draw() const {
  if (hasTexture) {
    // Draw with texture and scale
    DrawCubeTexture(texture, position, width * currentScale,
                    height * currentScale, length * currentScale, color);
  } else {
    // Draw with scale
    DrawCube(position, width * currentScale, height * currentScale,
             length * currentScale, color);
  }
}

void ScalingCubeObject::interact() {
  // Reverse scaling direction
  growingPhase = !growingPhase;

  // Also change color like parent
  CubeObject::interact();
}

BoundingBox ScalingCubeObject::getBoundingBox() const {
  float scaledWidth = width * currentScale;
  float scaledHeight = height * currentScale;
  float scaledLength = length * currentScale;

  return (BoundingBox){
      (Vector3){position.x - scaledWidth / 2, position.y - scaledHeight / 2,
                position.z - scaledLength / 2},
      (Vector3){position.x + scaledWidth / 2, position.y + scaledHeight / 2,
                position.z + scaledLength / 2}};
}
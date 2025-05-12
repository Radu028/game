#include "objects/Floor.h"

Floor::Floor(Vector3 position, Vector3 dimensions, Color color,
             bool hasCollisions)
    : StaticWorldObject(position, hasCollision),
      dimensions(dimensions),
      color(color) {}

void Floor::draw() const {
  DrawCube(position, dimensions.x, dimensions.y, dimensions.z, color);
}

BoundingBox Floor::getBoundingBox() const {
  return (BoundingBox){
      (Vector3){position.x - dimensions.x / 2, position.y - dimensions.y / 2,
                position.z - dimensions.z / 2},
      (Vector3){position.x + dimensions.x / 2, position.y + dimensions.y / 2,
                position.z + dimensions.z / 2},
  };
}
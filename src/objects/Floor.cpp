#include "objects/Floor.h"

Floor::Floor(Vector3 position, Vector3 dimensions, Color color,
             bool hasCollisions)
    : StaticWorldObject(position, hasCollision),
      dimensions(dimensions),
      color(color) {}
#include "objects/Wall.h"

Wall::Wall(Vector3 position, Vector3 dimensions, Color color,
           bool hasCollisions)
    : StaticWorldObject(position, hasCollisions),
      dimensions(dimensions),
      color(color) {}
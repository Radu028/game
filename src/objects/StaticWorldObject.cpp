#include "objects/StaticWorldObject.h"

StaticWorldObject::StaticWorldObject(Vector3 position, bool hasCollision)
    : GameObject(position, hasCollision, false, true) {}
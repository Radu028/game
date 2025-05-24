#include "systems/InputSystem.h"

#include <cmath>

Vector2 InputSystem::getMovementAxis() {
  float x = 0.0f;
  float y = 0.0f;

  if (IsKeyDown(KEY_A)) x -= 1.0f;
  if (IsKeyDown(KEY_D)) x += 1.0f;
  if (IsKeyDown(KEY_W)) y += 1.0f;
  if (IsKeyDown(KEY_S)) y -= 1.0f;

  Vector2 dir = {x, y};

  float lengthSquared = x * x + y * y;
  if (lengthSquared > 1.0f) {
    float invLength = 1.0f / std::sqrt(lengthSquared);
    dir.x *= invLength;
    dir.y *= invLength;
  }

  return dir;
}

bool InputSystem::isJumpPressed() { 
  return IsKeyPressed(KEY_SPACE);
}
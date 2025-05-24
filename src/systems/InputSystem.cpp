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

  float length = std::sqrt(x * x + y * y);
  if (length > 1.0f) {
    dir.x /= length;
    dir.y /= length;
  }

  return dir;
}

bool InputSystem::isJumpPressed() { 
  return IsKeyPressed(KEY_SPACE);
}
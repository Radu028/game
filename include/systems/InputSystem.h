#ifndef INPUT_SYSTEM_H
#define INPUT_SYSTEM_H

#include "raylib.h"

class InputSystem {
 public:
  static Vector2 getMovementAxis() {
    Vector2 axis = {0.0f, 0.0f};

    if (IsKeyDown(KEY_W)) axis.y += 1.0f;
    if (IsKeyDown(KEY_S)) axis.y -= 1.0f;
    if (IsKeyDown(KEY_A)) axis.x -= 1.0f;
    if (IsKeyDown(KEY_D)) axis.x += 1.0f;

    return axis;
  }

  static bool isJumpPressed() { return IsKeyPressed(KEY_SPACE); }
};

#endif  // INPUT_SYSTEM_H
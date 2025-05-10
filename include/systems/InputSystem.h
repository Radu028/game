#ifndef INPUTSYSTEM_H
#define INPUTSYSTEM_H

#include "raylib.h"

class InputSystem {
 public:
  static Vector2 getMovementAxis();

  static bool isJumpPressed();
};

#endif
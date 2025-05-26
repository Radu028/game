#include "systems/InputSystem.h"

#include <cmath>

// Static member definitions
bool InputSystem::mouseCameraEnabled = false;
Vector2 InputSystem::lastMousePosition = {0.0f, 0.0f};

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

bool InputSystem::isJumpPressed() { return IsKeyPressed(KEY_SPACE); }

// Mouse camera controls (Roblox-style)
bool InputSystem::isRightMousePressed() {
  return IsMouseButtonPressed(MOUSE_BUTTON_RIGHT);
}

bool InputSystem::isRightMouseDown() {
  return IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
}

bool InputSystem::isRightMouseReleased() {
  return IsMouseButtonReleased(MOUSE_BUTTON_RIGHT);
}

Vector2 InputSystem::getMouseDelta() {
  Vector2 currentMousePosition = GetMousePosition();
  Vector2 delta = {0.0f, 0.0f};

  if (mouseCameraEnabled) {
    delta.x = currentMousePosition.x - lastMousePosition.x;
    delta.y = currentMousePosition.y - lastMousePosition.y;
    lastMousePosition = currentMousePosition;
  }

  return delta;
}

void InputSystem::updateMouseCamera() {
  if (isRightMousePressed()) {
    enableMouseCamera();
  }

  if (isRightMouseReleased()) {
    disableMouseCamera();
  }

  // Always update last mouse position to prevent jumps when enabling camera
  if (!mouseCameraEnabled) {
    lastMousePosition = GetMousePosition();
  }
}

void InputSystem::enableMouseCamera() {
  mouseCameraEnabled = true;
  // Don't disable cursor - keep it visible like in Roblox
  lastMousePosition = GetMousePosition();
}

void InputSystem::disableMouseCamera() {
  mouseCameraEnabled = false;
  // Cursor stays enabled
}
#pragma once
#include <btBulletDynamicsCommon.h>

struct PhysicsSettings {
  static constexpr float GRAVITY_ACCELERATION = -78.4f;
  static constexpr float JUMP_VELOCITY = 18.0f;
  static constexpr float TIME_SCALE = 1.0;
  static constexpr float MAX_FALL_SPEED = -70.0f;

  // Prevents extreme time steps
  static constexpr float MIN_DELTA_TIME = 0.001f;
  static constexpr float MAX_DELTA_TIME = 0.033f;  // Caps at ~30fps minimum

  static constexpr int MAX_PHYSICS_ITERATIONS = 10;

  static constexpr float GROUND_ADJUST_EPSILON = 0.1f;
  static constexpr float COLLISION_SWEEP_EPSILON = 0.001f;

  static constexpr float MIN_Y_OVERLAP_FOR_HORIZONTAL_COLLISION = 0.2f;

  static constexpr float GROUND_CHECK_Y_ALIGN_MAX_PENETRATION = 0.05f;
  static constexpr float GROUND_CHECK_Y_ALIGN_MAX_SEPARATION = 0.2f;

  static constexpr float GROUND_STICK_DETACH_THRESHOLD = 0.01f;
};
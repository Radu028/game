#pragma once

struct PhysicsSettings {
  static constexpr float GRAVITY_ACCELERATION = -78.4f;
  static constexpr float JUMP_VELOCITY = 18.0f;
  static constexpr float TIME_SCALE = 1.0;
  static constexpr float MAX_FALL_SPEED = -70.0f;

  // Prevents extreme time steps
  static constexpr float MIN_DELTA_TIME = 0.001f;
  static constexpr float MAX_DELTA_TIME = 0.033f;  // Caps at ~30fps minimum

  static constexpr int MAX_PHYSICS_ITERATIONS = 10;
};
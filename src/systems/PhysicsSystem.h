#include "systems/PhysicsSystem.h"

void PhysicsSystem::applyGravity(GameObject& obj, float deltaTime) {
  float dt = deltaTime;
  dt = std::clamp(dt, PhysicsSettings::MIN_DELTA_TIME,
                  PhysicsSettings::MAX_DELTA_TIME);
  dt *= PhysicsSettings::TIME_SCALE;

  // obj.velocity
}
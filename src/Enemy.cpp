#include "Enemy.h"

#include <cmath>

#include "raylib.h"
#include "raymath.h"

Enemy::Enemy(Vector3 position, float size, Color color, int health, int damage, float speed)
    : CubeObject(position, size, size, size, color, true),
      health(health),
      maxHealth(health),
      damage(damage),
      speed(speed),
      attackCooldown(1.0f),
      timeSinceLastAttack(0.0f),
      velocity((Vector3){0.0f, 0.0f, 0.0f}),
      isGrounded(true),
      detectionRadius(10.0f),
      attackRadius(1.5f),
      currentState(IDLE),
      target(nullptr) {}

void Enemy::update(float deltaTime) {
    if (isDead() || target == nullptr) return;

    // Update attack cooldown
    timeSinceLastAttack += deltaTime;

    // Simple AI state machine
    if (isTargetInRange(attackRadius)) {
        currentState = ATTACK;
    } else if (isTargetInRange(detectionRadius)) {
        currentState = CHASE;
    } else {
        currentState = IDLE;
    }

    // Handle behavior based on state
    switch (currentState) {
        case IDLE:
            // Just stand still in idle
            break;

        case CHASE:
            chaseTarget(deltaTime);
            break;

        case ATTACK:
            attackTarget();
            chaseTarget(deltaTime);  // Still move towards player while attacking
            break;
    }

    // Apply simple gravity
    if (!isGrounded) {
        position.y -= 0.1f;  // Simple gravity effect
    }

    // Check if on ground (for a simple implementation)
    if (position.y < 0.5f) {
        position.y = 0.5f;
        isGrounded = true;
    }
}

bool Enemy::isTargetInRange(float radius) const {
    if (!target) return false;

    Vector3 targetPos = target->getPosition();
    Vector3 myPos = position;

    // Calculate distance in 2D (ignore Y axis for simpler gameplay)
    float distance = sqrtf(powf(targetPos.x - myPos.x, 2) + powf(targetPos.z - myPos.z, 2));

    return distance <= radius;
}

void Enemy::chaseTarget(float deltaTime) {
    if (!target) return;

    Vector3 targetPos = target->getPosition();
    Vector3 myPos = position;

    // Calculate direction vector to target (in 2D - ignore Y axis)
    Vector3 direction = {targetPos.x - myPos.x,
                         0,  // Ignore Y axis for now
                         targetPos.z - myPos.z};

    // Normalize the direction vector
    float length = sqrtf(direction.x * direction.x + direction.z * direction.z);
    if (length > 0) {
        direction.x /= length;
        direction.z /= length;
    }

    // Move towards the player
    position.x += direction.x * speed * deltaTime * 60.0f;  // Adjust for frame rate
    position.z += direction.z * speed * deltaTime * 60.0f;
}

void Enemy::attackTarget() {
    if (!target || timeSinceLastAttack < attackCooldown) return;

    // If we can attack, deal damage to the player
    if (isTargetInRange(attackRadius)) {
        target->takeDamage(damage);
        timeSinceLastAttack = 0.0f;  // Reset attack timer
    }
}

void Enemy::takeDamage(int amount) {
    health -= amount;
    if (health < 0) health = 0;
}

void Enemy::draw() const {
    // Draw the enemy cube
    CubeObject::draw();

    // Draw health bar above enemy if not dead
    if (!isDead()) {
        // Calculate health bar position above the enemy
        Vector3 barPos = {position.x, position.y + height + 0.2f, position.z};

        // Draw health bar background
        DrawCube(barPos, 1.0f, 0.1f, 0.1f, BLACK);

        // Draw health bar fill based on health percentage
        float healthPercent = (float)health / maxHealth;
        Vector3 fillPos = {barPos.x - (1.0f - healthPercent) * 0.5f, barPos.y, barPos.z};
        DrawCube(fillPos, healthPercent, 0.1f, 0.1f, RED);
    }
}
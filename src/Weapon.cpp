#include "Weapon.h"

#include <cmath>

#include "Enemy.h"
#include "raylib.h"
#include "raymath.h"

Weapon::Weapon(const std::string& name, int damage, float attackSpeed, float range, Vector3 size,
               Color color)
    : name(name),
      damage(damage),
      attackSpeed(attackSpeed),
      range(range),
      cooldown(1.0f / attackSpeed),
      timeSinceLastAttack(cooldown),  // Start ready to attack
      isEquipped(false),
      size(size),
      color(color) {}

void Weapon::update(float deltaTime) {
    // Update attack cooldown
    if (timeSinceLastAttack < cooldown) {
        timeSinceLastAttack += deltaTime;
    }
}

bool Weapon::attack(std::vector<Enemy*>& enemies, const Vector3& playerPosition,
                    const Vector3& playerDirection) {
    if (!canAttack() || !isEquipped) return false;

    // Reset attack timer
    timeSinceLastAttack = 0.0f;

    // Check for enemies in range and in the direction of attack
    bool hitEnemy = false;

    for (Enemy* enemy : enemies) {
        if (enemy->isDead()) continue;

        Vector3 enemyPos = enemy->getPosition();

        // Calculate vector from player to enemy
        Vector3 toEnemy = {enemyPos.x - playerPosition.x,
                           0,  // Ignore Y for simpler gameplay
                           enemyPos.z - playerPosition.z};

        // Calculate distance to enemy
        float distance = sqrtf(toEnemy.x * toEnemy.x + toEnemy.z * toEnemy.z);

        // Check if enemy is in range
        if (distance <= range) {
            // Normalize the vector to enemy
            if (distance > 0) {
                toEnemy.x /= distance;
                toEnemy.z /= distance;
            }

            // Calculate dot product to check if enemy is in front of player
            float dotProduct = toEnemy.x * playerDirection.x + toEnemy.z * playerDirection.z;

            // If dot product is positive, enemy is in front of player (within a 180° arc)
            if (dotProduct > 0.5f) {  // 0.5 = 60° cone (adjust as needed)
                // Hit the enemy
                enemy->takeDamage(damage);
                hitEnemy = true;
            }
        }
    }

    return hitEnemy;
}

void Weapon::draw(const Vector3& playerPosition, const Vector3& playerDirection) const {
    if (!isEquipped) return;

    // Calculate weapon position offset from player
    Vector3 offset = {playerDirection.x * 0.5f,  // Place in front of player
                      -0.2f,                     // Slightly below center
                      playerDirection.z * 0.5f};

    // Calculate rotation angle based on player direction
    float rotationAngle = atan2f(playerDirection.z, playerDirection.x) * RAD2DEG - 90.0f;

    // Calculate weapon position
    Vector3 weaponPos = {
        playerPosition.x + offset.x, playerPosition.y + offset.y, playerPosition.z + offset.z};

    // Draw weapon
    DrawCube(weaponPos, size.x, size.y, size.z, color);

    // Draw weapon outline
    DrawCubeWires(weaponPos, size.x, size.y, size.z, BLACK);
}
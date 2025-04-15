#ifndef ENEMY_H
#define ENEMY_H

#include <string>

#include "Character.h"
#include "CubeObject.h"

class Enemy : public CubeObject {
   private:
    // Enemy stats
    int health;
    int maxHealth;
    int damage;
    float speed;

    // Attack timing
    float attackCooldown;
    float timeSinceLastAttack;

    // Movement
    Vector3 velocity;
    bool isGrounded;
    float detectionRadius;
    float attackRadius;

    // AI state
    enum State { IDLE, CHASE, ATTACK };
    State currentState;

    // Target (player)
    Character* target;

   public:
    Enemy(Vector3 position, float size = 1.0f, Color color = RED, int health = 100, int damage = 10,
          float speed = 0.05f);

    // Set the target to chase
    void setTarget(Character* character) { target = character; }

    // Stats getters
    int getHealth() const { return health; }
    int getMaxHealth() const { return maxHealth; }
    int getDamage() const { return damage; }

    // Update enemy behavior
    void update(float deltaTime) override;

    // Check if target is in detection range
    bool isTargetInRange(float radius) const;

    // Chase target method
    void chaseTarget(float deltaTime);

    // Attack target method
    void attackTarget();

    // Take damage
    void takeDamage(int amount);

    // Check if enemy is dead
    bool isDead() const { return health <= 0; }

    // Override draw to show health bar
    void draw() const override;
};

#endif  // ENEMY_H
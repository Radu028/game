#ifndef WEAPON_H
#define WEAPON_H

#include <string>
#include <vector>

#include "raylib.h"

class Enemy;

// Weapon class that can be used by the player
class Weapon {
   private:
    std::string name;
    int damage;
    float attackSpeed;
    float range;
    float cooldown;
    float timeSinceLastAttack;
    bool isEquipped;

    // Weapon model properties
    Vector3 size;
    Color color;

   public:
    Weapon(const std::string& name, int damage, float attackSpeed, float range,
           Vector3 size = (Vector3){0.1f, 0.5f, 0.1f}, Color color = GRAY);

    // Getters
    std::string getName() const { return name; }
    int getDamage() const { return damage; }
    float getAttackSpeed() const { return attackSpeed; }
    float getRange() const { return range; }
    bool getIsEquipped() const { return isEquipped; }

    // Setters
    void setEquipped(bool equipped) { isEquipped = equipped; }

    // Update weapon state
    void update(float deltaTime);

    // Attack method
    bool attack(std::vector<Enemy*>& enemies, const Vector3& playerPosition,
                const Vector3& playerDirection);

    // Draw weapon (when equipped)
    void draw(const Vector3& playerPosition, const Vector3& playerDirection) const;

    // Check if weapon can attack (cooldown)
    bool canAttack() const { return timeSinceLastAttack >= cooldown; }
};

#endif  // WEAPON_H
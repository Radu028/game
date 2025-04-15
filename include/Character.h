#ifndef CHARACTER_H
#define CHARACTER_H

#include <string>
#include <unordered_map>
#include <vector>

#include "Player.h"
#include "Weapon.h"

// Forward declarations
class Enemy;

// Item structure for inventory
struct Item {
    std::string name;
    std::string description;
    int value;
    bool isEquippable;
    bool isConsumable;
};

class Character : public Player {
   private:
    // Character stats
    int health;
    int maxHealth;
    int stamina;
    int maxStamina;
    float staminaRegenRate;
    int level;
    int experience;
    int experienceToNextLevel;

    // Inventory
    std::vector<Item> inventory;
    std::unordered_map<std::string, int> equippedItems;

    // Weapon system
    Weapon* equippedWeapon;
    Vector3 lookDirection;

    // Combat system
    float invulnerabilityTime;
    float invulnerabilityTimer;
    bool isInvulnerable;

    // Stamina management
    float staminaTimer;
    bool isRunning;

    // Animation state
    int animationState;
    float animationTimer;

   public:
    Character(Vector3 position = (Vector3){0.0f, 0.0f, 0.0f},
              Vector3 size = (Vector3){1.0f, 1.8f, 1.0f}, Color color = BLUE);
    ~Character();

    // Stat methods
    int getHealth() const { return health; }
    int getMaxHealth() const { return maxHealth; }
    int getStamina() const { return stamina; }
    int getMaxStamina() const { return maxStamina; }
    int getLevel() const { return level; }
    int getExperience() const { return experience; }
    int getExperienceToNextLevel() const { return experienceToNextLevel; }

    void setHealth(int newHealth);
    void adjustHealth(int amount);
    void setStamina(int newStamina);
    void adjustStamina(int amount);
    void addExperience(int amount);
    void levelUp();

    // Inventory methods
    void addItem(const Item& item);
    void removeItem(int index);
    bool useItem(int index);
    bool equipItem(int index);
    bool unequipItem(const std::string& slot);
    const std::vector<Item>& getInventory() const { return inventory; }

    // Weapon methods
    void setWeapon(Weapon* weapon);
    Weapon* getEquippedWeapon() const { return equippedWeapon; }
    void attackWithWeapon(std::vector<Enemy*>& enemies);
    Vector3 getLookDirection() const { return lookDirection; }

    // Override movement for stamina usage
    void handleInput(float movementSpeed, float jumpForce);

    // Combat methods
    void takeDamage(int amount);
    bool isDead() const { return health <= 0; }
    bool isInvulnerabilityActive() const { return isInvulnerable; }

    // Override update to handle regeneration
    void update(float deltaTime) override;

    // Override draw to show weapon
    void draw() const override;

    // Animation methods
    int getAnimationState() const { return animationState; }
    void setAnimationState(int state);

    // Make Player protected members accessible
    bool getIsOnGround() const { return isOnGround; }
    Vector3 getVelocity() const { return velocity; }
};

#endif  // CHARACTER_H
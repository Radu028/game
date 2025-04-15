#include "Character.h"

#include "Enemy.h"
#include "raylib.h"
#include "raymath.h"

// Animation states
enum AnimationState { IDLE = 0, WALKING, RUNNING, JUMPING, FALLING, ATTACKING };

Character::Character(Vector3 position, Vector3 size, Color color)
    : Player(position, size, color),
      health(100),
      maxHealth(100),
      stamina(100),
      maxStamina(100),
      staminaRegenRate(10.0f),
      level(1),
      experience(0),
      experienceToNextLevel(100),
      equippedWeapon(nullptr),
      lookDirection({0.0f, 0.0f, -1.0f}),  // Default looking forward (negative Z)
      invulnerabilityTime(0.5f),
      invulnerabilityTimer(0.0f),
      isInvulnerable(false),
      staminaTimer(0.0f),
      isRunning(false),
      animationState(IDLE),
      animationTimer(0.0f) {}

Character::~Character() {
    // Weapon is managed externally, don't delete it here
}

void Character::setHealth(int newHealth) {
    health = newHealth;
    if (health > maxHealth) health = maxHealth;
    if (health < 0) health = 0;
}

void Character::adjustHealth(int amount) {
    health += amount;
    if (health > maxHealth) health = maxHealth;
    if (health < 0) health = 0;
}

void Character::setStamina(int newStamina) {
    stamina = newStamina;
    if (stamina > maxStamina) stamina = maxStamina;
    if (stamina < 0) stamina = 0;
}

void Character::adjustStamina(int amount) {
    stamina += amount;
    if (stamina > maxStamina) stamina = maxStamina;
    if (stamina < 0) stamina = 0;
}

void Character::addExperience(int amount) {
    experience += amount;
    if (experience >= experienceToNextLevel) {
        levelUp();
    }
}

void Character::levelUp() {
    level++;
    experience -= experienceToNextLevel;
    experienceToNextLevel = (int)(experienceToNextLevel * 1.5f);

    // Increase stats with level up
    maxHealth += 10;
    health = maxHealth;
    maxStamina += 5;
    stamina = maxStamina;
}

void Character::addItem(const Item& item) { inventory.push_back(item); }

void Character::removeItem(int index) {
    if (index >= 0 && index < inventory.size()) {
        inventory.erase(inventory.begin() + index);
    }
}

bool Character::useItem(int index) {
    if (index >= 0 && index < inventory.size()) {
        Item& item = inventory[index];

        if (item.isConsumable) {
            // Apply item effects - could be more complex based on item type
            if (item.name == "Health Potion") {
                adjustHealth(25);
            } else if (item.name == "Stamina Potion") {
                adjustStamina(25);
            }

            // Remove the item after use
            removeItem(index);
            return true;
        }
    }
    return false;
}

bool Character::equipItem(int index) {
    if (index >= 0 && index < inventory.size()) {
        Item& item = inventory[index];

        if (item.isEquippable) {
            // Simple equipment system - could be more complex with equipment slots
            std::string slot = "weapon";  // Default slot

            // Store the index of equipped item
            equippedItems[slot] = index;
            return true;
        }
    }
    return false;
}

bool Character::unequipItem(const std::string& slot) {
    auto it = equippedItems.find(slot);
    if (it != equippedItems.end()) {
        equippedItems.erase(it);
        return true;
    }
    return false;
}

void Character::setWeapon(Weapon* weapon) {
    // Unequip current weapon if any
    if (equippedWeapon) {
        equippedWeapon->setEquipped(false);
    }

    // Equip new weapon
    equippedWeapon = weapon;
    if (equippedWeapon) {
        equippedWeapon->setEquipped(true);
    }
}

void Character::attackWithWeapon(std::vector<Enemy*>& enemies) {
    if (equippedWeapon) {
        if (equippedWeapon->attack(enemies, position, lookDirection)) {
            // Attack hit something, play animation
            setAnimationState(ATTACKING);
        }
    }
}

void Character::handleInput(float movementSpeed, float jumpForce) {
    // Check for sprint input
    isRunning = IsKeyDown(KEY_LEFT_SHIFT) && stamina > 0;

    // Apply movement with sprint modifier if running
    float actualSpeed = isRunning ? movementSpeed * 1.7f : movementSpeed;

    // Update look direction based on movement
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_S) || IsKeyDown(KEY_A) || IsKeyDown(KEY_D)) {
        if (IsKeyDown(KEY_W))
            lookDirection.z = -1.0f;
        else if (IsKeyDown(KEY_S))
            lookDirection.z = 1.0f;
        else
            lookDirection.z = 0.0f;

        if (IsKeyDown(KEY_A))
            lookDirection.x = -1.0f;
        else if (IsKeyDown(KEY_D))
            lookDirection.x = 1.0f;
        else
            lookDirection.x = 0.0f;

        // Normalize look direction
        float length = sqrtf(lookDirection.x * lookDirection.x + lookDirection.z * lookDirection.z);
        if (length > 0) {
            lookDirection.x /= length;
            lookDirection.z /= length;
        }
    }

    // Pass to parent class movement handler
    Player::handleInput(actualSpeed, jumpForce);

    // Consume stamina when running
    if (isRunning &&
        (IsKeyDown(KEY_W) || IsKeyDown(KEY_A) || IsKeyDown(KEY_S) || IsKeyDown(KEY_D))) {
        adjustStamina(-1);
    }

    // Update animation state based on movement
    if (isOnGround) {
        if (isRunning &&
            (IsKeyDown(KEY_W) || IsKeyDown(KEY_A) || IsKeyDown(KEY_S) || IsKeyDown(KEY_D))) {
            setAnimationState(RUNNING);
        } else if (IsKeyDown(KEY_W) || IsKeyDown(KEY_A) || IsKeyDown(KEY_S) || IsKeyDown(KEY_D)) {
            setAnimationState(WALKING);
        } else {
            setAnimationState(IDLE);
        }
    } else {
        if (velocity.y > 0) {
            setAnimationState(JUMPING);
        } else {
            setAnimationState(FALLING);
        }
    }

    // Attack input (for melee attack)
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        setAnimationState(ATTACKING);
    }
}

void Character::takeDamage(int amount) {
    // Check if character is invulnerable
    if (isInvulnerable) return;

    // Apply damage
    adjustHealth(-amount);

    // Activate invulnerability frames
    isInvulnerable = true;
    invulnerabilityTimer = 0.0f;
}

void Character::update(float deltaTime) {
    // Call parent update for physics
    Player::update(deltaTime);

    // Update weapon
    if (equippedWeapon) {
        equippedWeapon->update(deltaTime);
    }

    // Update invulnerability timer
    if (isInvulnerable) {
        invulnerabilityTimer += deltaTime;
        if (invulnerabilityTimer >= invulnerabilityTime) {
            isInvulnerable = false;
        }
    }

    // Stamina regeneration when not running
    staminaTimer += deltaTime;
    if (!isRunning && staminaTimer >= 0.5f) {
        adjustStamina(1);
        staminaTimer = 0.0f;
    }

    // Animation timer for controlling animation duration
    animationTimer += deltaTime;
    if (animationState == ATTACKING && animationTimer >= 0.5f) {
        // Reset to idle after attack animation
        setAnimationState(IDLE);
    }
}

void Character::draw() const {
    // Draw character with flash effect when invulnerable
    if (isInvulnerable) {
        // Flash between normal color and white
        if ((int)(invulnerabilityTimer * 10) % 2 == 0) {
            // Draw with white color for flash effect
            if (hasTexture) {
                DrawModel(model, position, 1.0f, WHITE);
            } else {
                DrawCube(position, width, height, length, WHITE);
            }
        } else {
            // Draw normally
            CubeObject::draw();
        }
    } else {
        // Draw normally
        CubeObject::draw();
    }

    // Draw equipped weapon if any
    if (equippedWeapon) {
        equippedWeapon->draw(position, lookDirection);
    }
}

void Character::setAnimationState(int state) {
    if (animationState != state) {
        animationState = state;
        animationTimer = 0.0f;
    }
}
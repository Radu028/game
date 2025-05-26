#include "ai/NPC.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

#include "GameWorld.h"
#include "ai/NPCStates.h"
#include "raymath.h"

NPC::NPC(Vector3 position, std::shared_ptr<Shop> shop)
    : HumanoidCharacter(position),
      targetShop(shop),
      chatSystem(nullptr),
      movementSpeed(3.0f),
      detectionRadius(5.0f),
      interactionRadius(1.0f),
      isActive(true),
      lifetimeTimer(0.0f),
      maxLifetime(60.0f),
      hasDestination(false),
      currentWaypointIndex(0) {
  initializeNPC();

  // Start with idle state
  currentState = std::make_unique<IdleState>();
  currentState->enter(this);
}

NPC::~NPC() {
  if (currentState) {
    currentState->exit(this);
  }
}

void NPC::initializeNPC() {
  // Give NPC unique visual characteristics
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> speedDist(2.5f, 4.0f);

  movementSpeed = speedDist(gen);

  // Randomize some visual aspects if needed
  // For now, NPCs will look like the player but with different behavior
}

void NPC::changeState(std::unique_ptr<NPCState> newState) {
  if (currentState) {
    currentState->exit(this);
  }

  currentState = std::move(newState);

  if (currentState) {
    currentState->enter(this);
  }
}

void NPC::update(float deltaTime) {
  if (!isActive) {
    return;
  }

  // Update base humanoid character
  HumanoidCharacter::update(deltaTime);

  // Update lifetime
  updateLifetime(deltaTime);

  // Update current state
  if (currentState) {
    currentState->update(this, deltaTime);
  }

  // Handle movement towards destination if one is set
  if (hasDestination) {
    moveTowards(currentDestination, deltaTime);
  }

  // Check if NPC should be deactivated due to lifetime
  if (lifetimeTimer >= maxLifetime) {
    setActive(false);
    notifyExited();
  }
}

void NPC::moveTowards(Vector3 target, float deltaTime) {
  Vector3 currentPos = getPosition();
  Vector3 direction = Vector3Subtract(target, currentPos);
  float distanceToTarget = Vector3Length(direction);

  // More generous threshold for reaching target
  if (distanceToTarget > 0.8f) {
    direction = Vector3Normalize(direction);
    Vector3 movement = Vector3Scale(direction, movementSpeed * deltaTime);

    // Debug output for movement - print much less frequently to reduce console
    // spam
    static int movementFrameCount = 0;
    movementFrameCount++;
    if (movementFrameCount % 180 == 0) {
      std::cout << "🎯 NPC moveTowards: direction(" << direction.x << ", "
                << direction.y << ", " << direction.z
                << ") speed: " << movementSpeed << std::endl;
    }

    // Relax obstacle detection when very close to target (within 4 units) or
    // when approaching shop entrance
    bool isCloseToTarget = distanceToTarget < 4.0f;
    bool isApproachingShopEntrance = false;

    // Check if we're approaching the shop entrance - be more permissive with
    // collisions
    if (targetShop) {
      Vector3 entrancePos = targetShop->getEntrancePosition();
      float distanceToEntrance = Vector3Distance(currentPos, entrancePos);
      isApproachingShopEntrance = distanceToEntrance < 8.0f;
    }

    // Check for obstacles ahead using collision prediction, but be more lenient
    // when close to target or approaching shop
    if (!isCloseToTarget && !isApproachingShopEntrance &&
        wouldCollideAfterMovement(direction, deltaTime * 2.0f)) {
      static int obstacleFrameCount = 0;
      obstacleFrameCount++;
      if (obstacleFrameCount % 120 == 0) {
        std::cout << "🚧 NPC detected obstacle, trying to navigate around"
                  << std::endl;
      }

      // Try smaller angle turns (45 degrees instead of 90)
      float angle45 = PI / 4.0f;
      Vector3 leftDirection = {
          direction.x * cos(angle45) - direction.z * sin(angle45), direction.y,
          direction.x * sin(angle45) + direction.z * cos(angle45)};
      leftDirection = Vector3Normalize(leftDirection);

      if (!wouldCollideAfterMovement(leftDirection, deltaTime * 2.0f)) {
        direction = leftDirection;
        if (obstacleFrameCount % 120 == 0) {
          std::cout << "↩️ NPC turning left 45° to avoid obstacle" << std::endl;
        }
      } else {
        // Try turning right 45 degrees
        Vector3 rightDirection = {
            direction.x * cos(-angle45) - direction.z * sin(-angle45),
            direction.y,
            direction.x * sin(-angle45) + direction.z * cos(-angle45)};
        rightDirection = Vector3Normalize(rightDirection);

        if (!wouldCollideAfterMovement(rightDirection, deltaTime * 2.0f)) {
          direction = rightDirection;
          if (obstacleFrameCount % 120 == 0) {
            std::cout << "↪️ NPC turning right 45° to avoid obstacle"
                      << std::endl;
          }
        } else {
          // If still blocked, just move directly toward target when close or
          // approaching shop
          if (isCloseToTarget || isApproachingShopEntrance) {
            if (obstacleFrameCount % 120 == 0) {
              std::cout << "🎯 NPC close to target or approaching shop, "
                           "ignoring obstacles"
                        << std::endl;
            }
          } else {
            // Try a more dramatic turn (135 degrees)
            float angle135 = 3.0f * PI / 4.0f;
            Vector3 escapeDirection = {
                direction.x * cos(angle135) - direction.z * sin(angle135),
                direction.y,
                direction.x * sin(angle135) + direction.z * cos(angle135)};
            direction = Vector3Normalize(escapeDirection);
            if (obstacleFrameCount % 120 == 0) {
              std::cout << "🔄 NPC making escape turn to avoid obstacle"
                        << std::endl;
            }
          }
        }
      }
    }

    // Apply movement forces (similar to how HumanoidCharacter handles input)
    applyMovementForces({direction.x, 0, direction.z}, movementSpeed);
  } else {
    static int reachedTargetFrameCount = 0;
    reachedTargetFrameCount++;
    if (reachedTargetFrameCount % 240 == 0) {
      std::cout << "✅ NPC reached target position" << std::endl;
    }
  }
}

bool NPC::isNearTarget(Vector3 target, float threshold) const {
  return Vector3Distance(getPosition(), target) <= threshold;
}

void NPC::addFruitToInventory(std::shared_ptr<Fruit> fruit) {
  if (fruit &&
      std::find(inventory.begin(), inventory.end(), fruit) == inventory.end()) {
    inventory.push_back(fruit);
  }
}

void NPC::setDestination(Vector3 destination) {
  currentDestination = destination;
  hasDestination = true;
  pathWaypoints.clear();
  currentWaypointIndex = 0;
}

void NPC::addObserver(NPCObserver* observer) {
  if (observer && std::find(observers.begin(), observers.end(), observer) ==
                      observers.end()) {
    observers.push_back(observer);
  }
}

void NPC::removeObserver(NPCObserver* observer) {
  observers.erase(std::remove(observers.begin(), observers.end(), observer),
                  observers.end());
}

void NPC::notifyFruitPicked(std::shared_ptr<Fruit> fruit) {
  for (auto* observer : observers) {
    observer->onNPCFruitPicked(this, fruit);
  }
}

void NPC::notifyExited() {
  for (auto* observer : observers) {
    observer->onNPCExited(this);
  }
}

void NPC::notifyEnteredShop() {
  for (auto* observer : observers) {
    observer->onNPCEnteredShop(this);
  }
}

std::shared_ptr<Fruit> NPC::findNearestFruit() const {
  if (!targetShop) {
    return nullptr;
  }

  return targetShop->findNearestFruit(getPosition());
}

Vector3 NPC::getRandomPositionInShop() const {
  if (!targetShop) {
    return getPosition();
  }

  return targetShop->getRandomInteriorPosition();
}

Vector3 NPC::getExitPosition() const {
  if (!targetShop) {
    // Default exit position
    return {20.0f, 0.5f, 20.0f};
  }

  Vector3 entrance = targetShop->getEntrancePosition();
  // Move away from the shop
  return {entrance.x, entrance.y, entrance.z + 5.0f};
}

bool NPC::isInsideShop() const {
  if (!targetShop) {
    return false;
  }

  return targetShop->isInsideShop(getPosition());
}

void NPC::interact() {
  // NPCs can be interacted with to show their state or inventory
  // For now, this is a placeholder
}

void NPC::sayMessage(const std::string& context) const {
  if (chatSystem && chatSystem->canSpeak()) {
    std::string message;

    if (context == "greeting") {
      message = chatSystem->getRandomGreeting();
    } else if (context == "shopping") {
      message = chatSystem->getRandomShoppingComment();
    } else if (context == "fruit") {
      message = chatSystem->getRandomFruitComment();
    } else if (context == "leaving") {
      message = chatSystem->getRandomLeavingComment();
    } else {
      message = chatSystem->getRandomGeneralComment();
    }

    if (!message.empty()) {
      chatSystem->addMessage(message, getPosition());
    }
  }
}

void NPC::updateLifetime(float deltaTime) { lifetimeTimer += deltaTime; }

Vector3 NPC::calculateRandomColor() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> colorDist(0.3f, 1.0f);

  return {colorDist(gen), colorDist(gen), colorDist(gen)};
}

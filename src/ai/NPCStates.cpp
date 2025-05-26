#include "ai/NPCStates.h"

#include <cmath>
#include <iostream>
#include <random>

#include "ai/NPC.h"
#include "raymath.h"
#include "shop/Shop.h"

// IdleState implementation
void IdleState::enter(NPC* npc) {
  idleTime = 0.0f;
  std::cout << "🧑 NPC IDLE" << std::endl;

  // Add a greeting message
  npc->sayMessage("greeting");
}

void IdleState::update(NPC* npc, float deltaTime) {
  idleTime += deltaTime;

  if (idleTime >= maxIdleTime) {
    std::cout << "🚶 IDLE → MOVING" << std::endl;
    // Transition to moving towards shop
    npc->changeState(std::make_unique<MovingToShopState>());
  }
}

void IdleState::exit(NPC* npc) {
  // Nothing special needed when exiting idle
}

// MovingToShopState implementation
void MovingToShopState::enter(NPC* npc) {
  std::cout << "🚪 MOVING_TO_SHOP" << std::endl;
  auto shop = npc->getTargetShop();
  if (shop) {
    targetPosition = shop->getEntrancePosition();
    hasTarget = true;
    npc->setDestination(targetPosition);
    std::cout << "🎯 Target: (" << targetPosition.x << ", " << targetPosition.y
              << ", " << targetPosition.z << ")" << std::endl;

    // Create smarter waypoints to help NPCs navigate around walls to the
    // entrance
    Vector3 currentPos = npc->getPosition();
    Vector3 shopPos = shop->getPosition();
    Vector3 shopSize = shop->getSize();

    // Strategy: First move to the side of the shop to avoid walls, then
    // approach entrance
    Vector3 sideApproachPoint;
    Vector3 entrancePos = shop->getEntrancePosition();

    // Determine which side to approach from based on NPC's current position
    bool approachFromLeft = currentPos.x < shopPos.x;

    if (approachFromLeft) {
      // Approach from the left side of the shop, in front where entrance is
      sideApproachPoint = {shopPos.x - shopSize.x / 2 - 3.0f, currentPos.y,
                           entrancePos.z};
    } else {
      // Approach from the right side of the shop, in front where entrance is
      sideApproachPoint = {shopPos.x + shopSize.x / 2 + 3.0f, currentPos.y,
                           entrancePos.z};
    }

    // Set initial waypoint for side approach
    intermediateTarget = sideApproachPoint;
    hasIntermediateTarget = true;
    currentStage = WaypointStage::SIDE_APPROACH;
    npc->setDestination(intermediateTarget);

    std::cout << "🗺️ Stage: SIDE_APPROACH, Waypoint: (" << intermediateTarget.x
              << ", " << intermediateTarget.y << ", " << intermediateTarget.z
              << ")" << std::endl;

    // Initialize stuck detection
    lastPosition = currentPos;
    stuckTimer = 0.0f;
    hasUsedAlternateApproach = false;
  }
}

void MovingToShopState::update(NPC* npc, float deltaTime) {
  auto shop = npc->getTargetShop();
  if (!shop || !hasTarget) {
    std::cout << "❌ No shop target" << std::endl;
    npc->changeState(std::make_unique<WanderingState>());
    return;
  }

  Vector3 currentPos = npc->getPosition();
  Vector3 entrancePos = shop->getEntrancePosition();

  // Check if NPC has reached the shop entrance
  float distanceToEntrance = Vector3Distance(currentPos, entrancePos);
  if (distanceToEntrance <
      4.0f) {  // Increased threshold for more generous detection
    std::cout << "🎉 REACHED SHOP! Distance: " << distanceToEntrance
              << std::endl;
    npc->changeState(std::make_unique<ShoppingState>());
    return;
  }

  // Multi-stage waypoint system for smarter pathfinding
  if (hasIntermediateTarget) {
    Vector3 targetPos = intermediateTarget;
    float distanceToWaypoint = Vector3Distance(currentPos, targetPos);

    // Check if we've reached the current waypoint
    if (distanceToWaypoint < 2.0f) {
      std::cout << "✅ Reached waypoint! Stage: " << (int)currentStage
                << ", Distance: " << distanceToWaypoint << std::endl;

      if (currentStage == WaypointStage::SIDE_APPROACH) {
        // Move to final approach waypoint (in front of entrance)
        Vector3 finalApproachPoint = {
            entrancePos.x, currentPos.y,
            entrancePos.z + 6.0f};  // In front of entrance
        intermediateTarget = finalApproachPoint;
        currentStage = WaypointStage::FINAL_APPROACH;
        npc->setDestination(intermediateTarget);
        std::cout << "🗺️ Stage: FINAL_APPROACH, Waypoint: ("
                  << intermediateTarget.x << ", " << intermediateTarget.y
                  << ", " << intermediateTarget.z << ")" << std::endl;
      } else if (currentStage == WaypointStage::FINAL_APPROACH) {
        // Clear intermediate target and go direct to entrance
        hasIntermediateTarget = false;
        currentStage = WaypointStage::DIRECT_TO_ENTRANCE;
        npc->setDestination(entrancePos);
        std::cout << "🗺️ Stage: DIRECT_TO_ENTRANCE, Target: (" << entrancePos.x
                  << ", " << entrancePos.y << ", " << entrancePos.z << ")"
                  << std::endl;
      }
    } else {
      // Keep moving toward current waypoint
      npc->setDestination(targetPos);
    }
  } else {
    // Direct approach to entrance
    npc->setDestination(entrancePos);
  }

  // Stuck detection and recovery
  float moveDistance = Vector3Distance(currentPos, lastPosition);
  if (moveDistance < 0.1f) {
    stuckTimer += deltaTime;

    // Debug output every second when stuck
    static float debugTimer = 0.0f;
    debugTimer += deltaTime;
    if (debugTimer > 1.0f) {
      std::cout
          << "🔄 NPC stuck detection: Distance moved: " << moveDistance
          << ", Stuck timer: " << stuckTimer << ", Current pos: ("
          << currentPos.x << ", " << currentPos.y << ", " << currentPos.z << ")"
          << ", Target: ("
          << (hasIntermediateTarget ? intermediateTarget.x : entrancePos.x)
          << ", "
          << (hasIntermediateTarget ? intermediateTarget.y : entrancePos.y)
          << ", "
          << (hasIntermediateTarget ? intermediateTarget.z : entrancePos.z)
          << ")" << std::endl;
      debugTimer = 0.0f;
    }

    if (stuckTimer > 3.0f) {  // Reduced timeout for faster recovery
      std::cout << "🚫 NPC STUCK! Timer: " << stuckTimer
                << ", trying alternate approach" << std::endl;

      if (!hasUsedAlternateApproach) {
        // Try approaching from the opposite side
        Vector3 shopPos = shop->getPosition();
        Vector3 shopSize = shop->getSize();
        Vector3 entrancePos = shop->getEntrancePosition();
        Vector3 alternateApproach;

        // Switch to opposite side, but stay in front of shop where entrance is
        if (intermediateTarget.x < shopPos.x) {
          alternateApproach = {shopPos.x + shopSize.x / 2 + 3.0f, currentPos.y,
                               entrancePos.z};
        } else {
          alternateApproach = {shopPos.x - shopSize.x / 2 - 3.0f, currentPos.y,
                               entrancePos.z};
        }

        intermediateTarget = alternateApproach;
        hasIntermediateTarget = true;
        currentStage = WaypointStage::SIDE_APPROACH;
        hasUsedAlternateApproach = true;
        npc->setDestination(intermediateTarget);
        std::cout << "🔄 Trying alternate side: (" << intermediateTarget.x
                  << ", " << intermediateTarget.y << ", "
                  << intermediateTarget.z << ")" << std::endl;
      } else {
        // If alternate approach also failed, try wandering
        std::cout << "❌ Both approaches failed, switching to wandering"
                  << std::endl;
        npc->changeState(std::make_unique<WanderingState>());
        return;
      }

      stuckTimer = 0.0f;
    }
  } else {
    stuckTimer = 0.0f;
  }

  lastPosition = currentPos;
}

void MovingToShopState::exit(NPC* npc) {
  std::cout << "🚪 Exiting MOVING_TO_SHOP" << std::endl;
}

// ShoppingState implementation
void ShoppingState::enter(NPC* npc) {
  std::cout << "🛒 SHOPPING" << std::endl;
  shoppingTime = 0.0f;
  fruitSearchTimer = 0.0f;
  hasCurrentTarget = false;

  // Generate random shopping duration between 5-10 seconds
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> timeDist(5.0f, 10.0f);
  maxShoppingTime = timeDist(gen);

  npc->sayMessage("shopping");
}

void ShoppingState::update(NPC* npc, float deltaTime) {
  shoppingTime += deltaTime;
  fruitSearchTimer += deltaTime;

  auto shop = npc->getTargetShop();
  if (!shop) {
    npc->changeState(std::make_unique<WanderingState>());
    return;
  }

  // Look for fruits to examine every 2 seconds
  if (fruitSearchTimer >= 2.0f) {
    auto nearbyFruit = shop->findNearestFruit(npc->getPosition());
    if (nearbyFruit) {
      // Check if fruit is within reasonable distance
      float distance =
          Vector3Distance(npc->getPosition(), nearbyFruit->getPosition());
      if (distance <= 8.0f) {
        currentTarget = nearbyFruit->getPosition();
        hasCurrentTarget = true;
        npc->setDestination(currentTarget);

        std::cout << "🍎 Examining fruit at (" << currentTarget.x << ", "
                  << currentTarget.y << ", " << currentTarget.z << ")"
                  << std::endl;
      }
    }
    fruitSearchTimer = 0.0f;
  }

  // Move toward current fruit target if we have one
  if (hasCurrentTarget) {
    Vector3 currentPos = npc->getPosition();
    float distanceToTarget = Vector3Distance(currentPos, currentTarget);

    if (distanceToTarget < 1.5f) {
      // "Examine" the fruit (just wait a moment)
      hasCurrentTarget = false;
      npc->sayMessage("examine");
      std::cout << "🔍 Examining fruit..." << std::endl;
    }
  }

  // Finish shopping after the allotted time
  if (shoppingTime >= maxShoppingTime) {
    std::cout << "✅ SHOPPING → LEAVING" << std::endl;
    npc->changeState(std::make_unique<LeavingState>());
  }
}

void ShoppingState::exit(NPC* npc) { hasCurrentTarget = false; }

// WanderingState implementation
void WanderingState::enter(NPC* npc) {
  std::cout << "🚶 WANDERING" << std::endl;
  wanderTime = 0.0f;
  hasWanderTarget = false;
}

void WanderingState::update(NPC* npc, float deltaTime) {
  wanderTime += deltaTime;

  if (!hasWanderTarget || wanderTime >= maxWanderTime) {
    // Pick a new random wander target
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> xDist(-25.0f, 25.0f);
    std::uniform_real_distribution<float> zDist(-25.0f, 25.0f);

    wanderTarget = {xDist(gen), npc->getPosition().y, zDist(gen)};
    hasWanderTarget = true;
    wanderTime = 0.0f;
    npc->setDestination(wanderTarget);

    std::cout << "🎯 New wander target: (" << wanderTarget.x << ", "
              << wanderTarget.y << ", " << wanderTarget.z << ")" << std::endl;
  }

  // After wandering for a while, try going to shop again
  if (wanderTime > 10.0f) {
    std::cout << "🔄 WANDERING → MOVING_TO_SHOP" << std::endl;
    npc->changeState(std::make_unique<MovingToShopState>());
  }
}

void WanderingState::exit(NPC* npc) { hasWanderTarget = false; }

// LeavingState implementation
void LeavingState::enter(NPC* npc) {
  std::cout << "👋 LEAVING" << std::endl;
  hasExitTarget = false;

  // Set exit target to edge of the world
  Vector3 currentPos = npc->getPosition();
  exitTarget = {currentPos.x > 0 ? 30.0f : -30.0f, currentPos.y,
                currentPos.z > 0 ? 30.0f : -30.0f};
  hasExitTarget = true;
  npc->setDestination(exitTarget);

  npc->sayMessage("goodbye");
}

void LeavingState::update(NPC* npc, float deltaTime) {
  if (!hasExitTarget) return;

  Vector3 currentPos = npc->getPosition();
  float distanceToExit = Vector3Distance(currentPos, exitTarget);

  if (distanceToExit < 2.0f) {
    std::cout << "🚪 NPC has left the area" << std::endl;
    // Mark for removal by setting NPC as inactive
    npc->setActive(false);
  }
}

void LeavingState::exit(NPC* npc) { hasExitTarget = false; }
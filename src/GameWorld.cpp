#include "GameWorld.h"

#include <algorithm>
#include <limits>

#include "entities/Player.h"
#include "exceptions/CollisionException.h"
#include "exceptions/GameException.h"

GameWorld* GameWorld::instance = nullptr;

GameWorld::GameWorld(Player* player) : player(player) {}

GameWorld* GameWorld::getInstance(Player* player) {
  if (instance == nullptr) {
    if (player == nullptr) {
      throw GameException("Cannot initialize GameWorld with null player");
    }
    instance = new GameWorld(player);
  }
  return instance;
}

void GameWorld::addObject(std::shared_ptr<GameObject> object) {
  if (object == nullptr) {
    throw GameException("Cannot add null object to GameWorld");
  }
  objects.push_back(object);
}

void GameWorld::update(float deltaTime) {
  for (auto& object : objects) {
    object->update(deltaTime);
  }

  try {
    checkCollisions();
  } catch (const CollisionException& e) {
    // Log collision error but continue
    TraceLog(LOG_WARNING, "Collision exception: %s", e.what());
  }
}

void GameWorld::draw() const {
  for (const auto& object : objects) {
    object->draw();
  }
}

void GameWorld::checkCollisions() {
  // Simple collision check between all objects (can be optimized)
  for (size_t i = 0; i < objects.size(); ++i) {
    for (size_t j = i + 1; j < objects.size(); ++j) {
      if (objects[i]->checkCollision(*objects[j])) {
// Handle collision (example: report it, adjust physics, etc.)
// For now, we just throw an exception if debug mode is on
#ifdef DEBUG
        throw CollisionException("Collision detected between objects " +
                                 std::to_string(i) + " and " +
                                 std::to_string(j));
#endif
      }
    }
  }
}

void GameWorld::interactWithNearestObject() {
  if (!player) return;

  std::shared_ptr<GameObject> nearestObject;
  float minDistance = std::numeric_limits<float>::max();

  for (const auto& object : objects) {
    float distance = player->getDistance(*object);
    if (distance < minDistance) {
      minDistance = distance;
      nearestObject = object;
    }
  }

  // Interact with the nearest object if it's within a certain range
  const float INTERACTION_RANGE = 2.0f;
  if (nearestObject && minDistance < INTERACTION_RANGE) {
    nearestObject->interact();
  }
}
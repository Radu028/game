#include "GameWorld.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "ai/NPC.h"
#include "shop/Shop.h"
#include "systems/PhysicsSystem.h"

GameWorld* GameWorld::instance = nullptr;
size_t GameWorld::totalObjectsCreated = 0;
size_t GameWorld::totalObjectsDestroyed = 0;
std::unordered_map<std::string, size_t> GameWorld::objectTypeCount;
std::string GameWorld::creationTimestamp;

GameWorld* GameWorld::getInstance(GameObject* player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(GameObject* player, const std::string& name)
    : player(player), worldName(name) {
  // Set creation timestamp
  auto now = std::time(nullptr);
  std::stringstream ss;
  creationTimestamp = ss.str();

  physicsSystem = std::make_unique<PhysicsSystem>(this);

  if (this->player) {
    physicsSystem->addObject(this->player);
  }
}

GameWorld::~GameWorld() {
  if (instance == this) {
    instance = nullptr;
  }
}

void GameWorld::addObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  objects.push_back(object);
  if (physicsSystem) {
    physicsSystem->addObject(object.get());
  }
}

void GameWorld::removeObject(std::shared_ptr<GameObject> object) {
  if (!object) return;

  if (physicsSystem) {
    physicsSystem->removeObject(object.get());
  }

  objects.erase(std::remove_if(objects.begin(), objects.end(),
                               [&](const std::shared_ptr<GameObject>& obj) {
                                 return obj == object;
                               }),
                objects.end());
}

void GameWorld::update(float deltaTime) {
  for (auto& objSharedPtr : objects) {
    if (objSharedPtr.get() != player) {
      objSharedPtr->update(deltaTime);
    }
  }

  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  if (player) {
  }
}

void GameWorld::draw() const {
  for (const auto& obj : objects) {
    obj->draw();
  }
}

btDiscreteDynamicsWorld* GameWorld::getBulletWorld() const {
  return physicsSystem ? physicsSystem->dynamicsWorld : nullptr;
}

void GameWorld::initializeNavMesh() {
  // Create navigation mesh covering the game world
  Vector3 minBounds = {-30.0f, 0.0f, -30.0f};
  Vector3 maxBounds = {30.0f, 0.0f, 30.0f};
  navigationMesh = std::make_shared<NavMesh>(minBounds, maxBounds, 1.0f);

  navigationMesh->generateNavMesh();

  auto shops = findObjectsOfType<Shop>();
  for (const auto& shop : shops) {
    Vector3 shopPos = shop->getPosition();
    Vector3 shopSize = shop->getSize();

    navigationMesh->addObstacle(shopPos, shopSize);

    // Define shop interior as walkable (overrides obstacle for interior)
    navigationMesh->defineShopInterior(shopPos, shopSize);

    // Define shop entrance area as walkable
    Vector3 entrancePos = shop->getEntrancePosition();
    Vector3 entranceSize = {3.0f, 2.0f, 3.0f};  // Generous entrance area
    navigationMesh->defineShopEntrance(entrancePos, entranceSize);
  }

  // Rebuild connections now that obstacles are properly defined
  navigationMesh->rebuildConnections();

  // Apply any obstacles that were queued before the navmesh existed
  for (const auto& pending : pendingObstacles) {
    if (auto obj = pending.object) {
      Vector3 pos = obj->getPosition();
      Vector3 size = obj->getObstacleSize();
      std::string obstacleType =
          pending.type.empty() ? obj->getObstacleType() : pending.type;
      bool ignoreY = obstacleType == "shelf";
      navigationMesh->markNodesInArea(
          pos, size, false,
          obstacleType == "shelf"  ? 0.7f
          : obstacleType == "wall" ? 0.3f
                                   : 0.5f,
          ignoreY);
    }
  }
  pendingObstacles.clear();

  navigationMesh->rebuildConnections();

  NPC::setNavMesh(navigationMesh);
}

// Obstacle management implementations
void GameWorld::addObjectAsObstacle(std::shared_ptr<GameObject> object,
                                    const std::string& type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    Vector3 pos = object->getPosition();
    Vector3 size = object->getObstacleSize();
    std::string obstacleType = type.empty() ? object->getObstacleType() : type;

    bool ignoreY = obstacleType == "shelf";
    navigationMesh->addObstacle(pos, size, obstacleType, ignoreY);
  } else {
    pendingObstacles.push_back({object, type});
  }
}

void GameWorld::removeObjectObstacle(std::shared_ptr<GameObject> object) {
  if (!object || !navigationMesh) return;

  Vector3 pos = object->getPosition();
  Vector3 size = object->getObstacleSize();

  navigationMesh->removeObstacle(pos, size);
  removeObject(object);
}

void GameWorld::addObstacleAt(Vector3 position, Vector3 size,
                              const std::string& type) {
  if (navigationMesh) {
    bool ignoreY = type == "shelf";
    navigationMesh->addObstacle(position, size, type, ignoreY);
  }
}

void GameWorld::addObjectAsObstacleDeferred(std::shared_ptr<GameObject> object,
                                            const std::string& type) {
  if (!object) return;

  addObject(object);  // Add to world first

  if (navigationMesh) {
    Vector3 pos = object->getPosition();
    Vector3 size = object->getObstacleSize();
    std::string obstacleType = type.empty() ? object->getObstacleType() : type;

    // Add obstacle but don't rebuild connections yet
    bool ignoreY = obstacleType == "shelf";
    navigationMesh->markNodesInArea(
        pos, size, false,
        obstacleType == "shelf"  ? 0.7f
        : obstacleType == "wall" ? 0.3f
                                 : 0.5f,
        ignoreY);

  } else {
    pendingObstacles.push_back({object, type});
  }
}

void GameWorld::finalizeObstacles() {
  if (navigationMesh) {
    navigationMesh->rebuildConnections();
  }
}
#include "GameWorld.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "systems/PhysicsSystem.h"

// Static member definitions
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
  ss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
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
#include "GameWorld.h"

#include <algorithm>  // For std::remove

#include "systems/PhysicsSystem.h"

GameWorld* GameWorld::instance = nullptr;

GameWorld* GameWorld::getInstance(GameObject* player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(GameObject* player) : player(player) {
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
  // Skip player update when using external controller (AdvancedPlayerController)
  // if (player) {
  //   player->update(deltaTime);
  // }

  for (auto& objSharedPtr : objects) {
    if (objSharedPtr.get() != player) {
      objSharedPtr->update(deltaTime);
    }
  }

  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  // Only do visual updates for GameObject types
  if (player) {
    // For GameObject types (like SimpleRagdoll), their update() method handles everything
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
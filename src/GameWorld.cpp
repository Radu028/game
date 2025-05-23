#include "GameWorld.h"

#include <algorithm>  // For std::remove

#include "entities/Player.h"
#include "systems/PhysicsSystem.h"

GameWorld* GameWorld::instance = nullptr;

GameWorld* GameWorld::getInstance(Player* player) {
  if (instance == nullptr) {
    instance = new GameWorld(player);
  }
  return instance;
}

GameWorld::GameWorld(Player* player) : player(player) {
  physicsSystem = std::make_unique<PhysicsSystem>(this);

  if (this->player) {
    physicsSystem->addObject(static_cast<GameObject*>(this->player));
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
  if (player) {
    player->update(deltaTime);
  }

  for (auto& objSharedPtr : objects) {
    if (objSharedPtr.get() != player) {
      objSharedPtr->update(deltaTime);
    }
  }

  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  if (player) {
    player->postPhysicsUpdate(deltaTime);
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
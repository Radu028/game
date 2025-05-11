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
  if (object) {
    objects.push_back(object);
    if (physicsSystem) {
      physicsSystem->addObject(object.get());
    }
  }
}

void GameWorld::removeObject(std::shared_ptr<GameObject> object) {
  if (object && physicsSystem) {
    physicsSystem->removeObject(object.get());
  }
  objects.erase(std::remove(objects.begin(), objects.end(), object),
                objects.end());
}

void GameWorld::update(float deltaTime) {
  if (physicsSystem) {
    physicsSystem->update(deltaTime);
  }

  if (player) {
    player->update(deltaTime);
  }

  for (auto& obj : objects) {
    if (obj && obj.get() != player) {
      obj->update(deltaTime);
    }
  }
}

void GameWorld::draw() const {
  for (const auto& obj : objects) {
    obj->draw();
  }
}
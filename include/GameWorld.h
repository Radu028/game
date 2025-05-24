#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <vector>

#include "objects/GameObject.h"

class PhysicsSystem;
class GameObject;
class Player;

class GameWorld {
 private:
  static GameWorld* instance;

  std::vector<std::shared_ptr<GameObject>> objects;
  GameObject* player;
  std::unique_ptr<PhysicsSystem> physicsSystem;

  GameWorld(GameObject* player);

  GameWorld(const GameWorld&) = delete;
  GameWorld& operator=(const GameWorld&) = delete;
  ~GameWorld();

 public:
  static GameWorld* getInstance(GameObject* player);

  void addObject(std::shared_ptr<GameObject> object);
  void removeObject(std::shared_ptr<GameObject> object);
  void update(float deltaTime);
  void draw() const;
  btDiscreteDynamicsWorld* getBulletWorld() const;
  btDiscreteDynamicsWorld* getDynamicsWorld() const { return getBulletWorld(); }

  const std::vector<std::shared_ptr<GameObject>>& getObjects() const {
    return objects;
  }
};

#endif
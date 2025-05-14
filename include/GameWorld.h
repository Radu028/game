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
  Player* player;
  std::unique_ptr<PhysicsSystem> physicsSystem;

  GameWorld(Player* player);

  GameWorld(const GameWorld&) = delete;
  GameWorld& operator=(const GameWorld&) = delete;
  ~GameWorld();

 public:
  static GameWorld* getInstance(Player* player);

  void addObject(std::shared_ptr<GameObject> object);
  void removeObject(std::shared_ptr<GameObject> object);
  void update(float deltaTime);
  void draw() const;
  void checkCollisions();

  const std::vector<std::shared_ptr<GameObject>>& getObjects() const {
    return objects;
  }
};

#endif
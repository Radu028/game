#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <vector>

#include "CubeObject.h"
#include "GameObject.h"

class Player;

class GameWorld {
   private:
    static GameWorld* instance;

    std::vector<std::shared_ptr<CubeObject>> objects;
    Player* player;

    GameWorld(Player* player);

    GameWorld(const GameWorld&) = delete;
    GameWorld& operator=(const GameWorld&) = delete;

   public:
    static GameWorld* getInstance(Player* player);

    void addObject(std::shared_ptr<CubeObject> object);
    void update(float deltaTime);
    void draw() const;
    void checkCollisions();

    const std::vector<std::shared_ptr<CubeObject>>& getObjects() const { return objects; }
};

#endif
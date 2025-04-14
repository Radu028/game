#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <vector>

#include "GameObject.h"
#include "CubeObject.h"

class Player;

class GameWorld {
   private:
    std::vector<std::shared_ptr<CubeObject>> objects;
    Player* player;

   public:
    GameWorld(Player* player);

    void addObject(std::shared_ptr<CubeObject> object);
    void update(float deltaTime);
    void draw() const;
    void checkCollisions();

    const std::vector<std::shared_ptr<CubeObject>>& getObjects() const { return objects; }
};

#endif
#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <vector>

#include "GameObject.h"
#include "Player.h"

class GameWorld {
   private:
    std::vector<std::shared_ptr<GameObject>> objects;
    Player* player;

   public:
    GameWorld(Player* player);

    void addObject(std::shared_ptr<GameObject> object);
    void update(float deltaTime);
    void draw() const;
    void checkCollisions();
};

#endif
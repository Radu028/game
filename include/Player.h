#ifndef PLAYER_H
#define PLAYER_H

#include <string>

#include "GameObject.h"
#include "raylib.h"

class GameWorld;

class Player : public GameObject {
   private:
    Vector3 velocity;
    bool isOnGround;
    GameWorld* world;

   public:
    Player();

    void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }

    void move(std::string direction, float byValue);

    void moveRight(float byValue);
    void moveLeft(float byValue);
    void moveForward(float byValue);
    void moveBackwards(float byValue);
    void jump(float jumpForce);

    void applyGravity(float gravity);
    void checkGroundCollision(float groundLevel);

    void update(float deltaTime) override;
};

#endif
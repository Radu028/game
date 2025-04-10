#ifndef PLAYER_H
#define PLAYER_H

#include <string>

#include "GameObject.h"
#include "raylib.h"

class GameWorld;

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT };

class Player : public GameObject {
   private:
    Vector3 velocity;
    bool isOnGround;
    GameWorld* world;

   public:
    Player();

    void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }

    void move(Direction direction, float byValue);
    void jump(float jumpForce);

    void applyGravity(float gravity);
    void checkGroundCollision(float groundLevel);

    void update(float deltaTime) override;
};

#endif
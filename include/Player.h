#ifndef PLAYER_H
#define PLAYER_H

#include "GameObject.h"
#include "raylib.h"

class Player : public GameObject {
   private:
    Vector3 velocity;
    bool isOnGround;

   public:
    Player();

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
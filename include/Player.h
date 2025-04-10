#ifndef PLAYER_H
#define PLAYER_H

#include <string>

#include "GameObject.h"
#include "raylib.h"

class GameWorld;

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT, UPWARD };

class Player : public GameObject {
   private:
    Vector3 velocity;
    bool isOnGround;
    GameWorld* world;

    void move(Direction direction, float byValue);
    bool checkCollisionWithWorld() const;
    float findMaxSafePosition(float start, float end, float* positionComponent);
    void moveWithSliding(float start, float end, float* positionComponent);

   public:
    Player();

    void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }

    void handleInput(float movementSpeed, float jumpForce);

    void applyGravity(float gravity);
    void checkGroundCollision(float groundLevel);

    void update(float deltaTime) override;
};

#endif
#ifndef PLAYER_H
#define PLAYER_H

#include <string>

#include "CubeObject.h"
#include "GameObject.h"
#include "raylib.h"

class GameWorld;

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT };

class Player : public CubeObject {
   private:
    Vector3 velocity;
    bool isOnGround;
    GameWorld* world;

    void move(Direction direction, float byValue);
    void jump();
    bool checkCollisionWithWorld() const;
    float findMaxSafePosition(float start, float end, float* positionComponent);
    void moveWithSliding(float start, float end, float* positionComponent);

   public:
    Player(Vector3 position = (Vector3){0.0f, 0.0f, 0.0f},
           Vector3 size = (Vector3){1.0f, 1.0f, 1.0f}, Color color = BLUE);

    void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }

    void handleInput(float movementSpeed);

    void applyGravity(float deltaTime);

    void update(float deltaTime) override;
};

#endif
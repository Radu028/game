#include "raylib.h"

class Player {
   private:
    Vector3 position;
    Vector3 velocity;
    bool isOnGround;

   public:
    Player();

    void moveRight(float byValue);
    void moveLeft(float byValue);
    void moveForward(float byValue);
    void moveBackwards(float byValue);
    void jump(float jumpForce);

    Vector3 getPosition() { return position; }

    void applyGravity(float gravity);
    void checkGroundCollision(float groundLevel);
};
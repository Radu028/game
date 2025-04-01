#include "../include/inputFunctions.h"

void handleInput(Player& player, float movementSpeed, float jumpForce) {
    if (IsKeyDown(KEY_W)) player.moveForward(movementSpeed);
    if (IsKeyDown(KEY_S)) player.moveBackwards(movementSpeed);
    if (IsKeyDown(KEY_A)) player.moveLeft(movementSpeed);
    if (IsKeyDown(KEY_D)) player.moveRight(movementSpeed);
    if (IsKeyPressed(KEY_SPACE)) player.jump(jumpForce);
}
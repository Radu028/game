#include "../include/inputFunctions.h"

void handleInput(Player& player, float movementSpeed, float jumpForce) {
    if (IsKeyDown(KEY_W)) player.move(FORWARD, movementSpeed);
    if (IsKeyDown(KEY_S)) player.move(BACKWARD, movementSpeed);
    if (IsKeyDown(KEY_A)) player.move(LEFT, movementSpeed);
    if (IsKeyDown(KEY_D)) player.move(RIGHT, movementSpeed);
    if (IsKeyPressed(KEY_SPACE)) player.jump(jumpForce);
}
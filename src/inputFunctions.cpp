#include "../include/inputFunctions.h"

void handleInput(Player& player, float movementSpeed, float jumpForce) {
    if (IsKeyDown(KEY_W)) player.move("forward", movementSpeed);
    if (IsKeyDown(KEY_S)) player.move("backward", movementSpeed);
    if (IsKeyDown(KEY_A)) player.move("left", movementSpeed);
    if (IsKeyDown(KEY_D)) player.move("right", movementSpeed);
    if (IsKeyPressed(KEY_SPACE)) player.jump(jumpForce);
}
#include "Player.h"

#include "raylib.h"

void Player::moveForward(float byValue) { this->position.z -= 0.1f; }
void Player::moveBackwards(float byValue) { this->position.z += 0.1f; }
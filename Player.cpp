#include "Player.h"

#include "raylib.h"

Player::Player() {}

void Player::moveForward(float byValue) { this->position.z -= byValue; }
void Player::moveBackwards(float byValue) { this->position.z += byValue; }
void Player::moveLeft(float byValue) { this->position.x -= byValue; }
void Player::moveRight(float byValue) { this->position.x += byValue; }

void Player::jump(float jumpForce) {
    if (this->position.y > 0.5f) return;
    this->velocity.y = jumpForce;
}

void Player::applyGravity(float gravity) {
    this->velocity.y += gravity;
    this->position.y += this->velocity.y;
}
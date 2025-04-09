#include "../include/Player.h"

#include "raylib.h"

Player::Player()
    : GameObject((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, BLUE),
      velocity((Vector3){0.0f, 0.0f, 0.0f}),
      isOnGround(true) {}

void Player::moveForward(float byValue) { this->position.z -= byValue; }
void Player::moveBackwards(float byValue) { this->position.z += byValue; }
void Player::moveLeft(float byValue) { this->position.x -= byValue; }
void Player::moveRight(float byValue) { this->position.x += byValue; }

void Player::jump(float jumpForce) {
    if (!this->isOnGround) return;
    this->velocity.y = jumpForce;
    this->isOnGround = false;
}

void Player::applyGravity(float gravity) {
    this->velocity.y += gravity;
    this->position.y += this->velocity.y;
}

void Player::checkGroundCollision(float groundLevel) {
    if (this->position.y <= groundLevel) {
        this->position.y = groundLevel;
        this->velocity.y = 0;
        this->isOnGround = true;
    } else {
        this->isOnGround = false;
    }
}

void Player::update(float deltaTime) {
    applyGravity(-0.01f);
    checkGroundCollision(0.5f);
}
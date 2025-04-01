#include "../include/Player.h"

#include "raylib.h"

Player::Player() {
    this->position = (Vector3){0.0f, 0.0f, 0.0f};
    this->velocity = (Vector3){0.0f, 0.0f, 0.0f};
    this->isOnGround = false;
}

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
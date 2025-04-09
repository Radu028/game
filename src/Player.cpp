#include "Player.h"

#include "GameWorld.h"
#include "raylib.h"

Player::Player()
    : GameObject((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, BLUE),
      velocity((Vector3){0.0f, 0.0f, 0.0f}),
      isOnGround(true),
      world(nullptr) {}

void Player::move(std::string direction, float byValue) {
    Vector3 oldPosition = this->position;

    if (direction == "forward") {
        this->position.z -= byValue;
    } else if (direction == "backward") {
        this->position.z += byValue;
    } else if (direction == "left") {
        this->position.x -= byValue;
    } else if (direction == "right") {
        this->position.x += byValue;
    } else {
        return;
    }

    if (this->world) {
        for (const auto& obj : this->world->getObjects()) {
            if (this->checkCollision(*obj)) {
                this->position = oldPosition;
                return;
            }
        }
    }
}

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
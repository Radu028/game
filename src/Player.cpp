#include "Player.h"

#include <iostream>

#include "GameWorld.h"
#include "raylib.h"

Player::Player()
    : GameObject((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, BLUE, true),
      velocity((Vector3){0.0f, 0.0f, 0.0f}),
      isOnGround(true),
      world(nullptr) {}

void Player::move(std::string direction, float byValue) {
    Vector3 oldPosition = this->position;

    Vector3 newPosition = this->position;
    if (direction == "forward") {
        newPosition.z -= byValue;
    } else if (direction == "backward") {
        newPosition.z += byValue;
    } else if (direction == "left") {
        newPosition.x -= byValue;
    } else if (direction == "right") {
        newPosition.x += byValue;
    } else {
        return;
    }

    this->position = newPosition;

    bool collision = false;
    if (this->world) {
        for (const auto& obj : this->world->getObjects()) {
            if (this->checkCollision(*obj)) {
                collision = true;
                return;
            }
        }
    }

    if (collision) {
        this->position = oldPosition;
    }

    if (collision) {
        std::cout << "Collision detected! Position reset." << std::endl;
    } else {
        std::cout << "No collision, new position: " << this->position.x << ", " << this->position.y
                  << ", " << this->position.z << std::endl;
    }
}

void Player::jump(float jumpForce) {
    if (!this->isOnGround) return;
    this->velocity.y = jumpForce;
    this->isOnGround = false;
}

void Player::applyGravity(float gravity) {
    Vector3 oldPosition = this->position;

    this->velocity.y += gravity;
    this->position.y += this->velocity.y;

    if (this->world) {
        for (const auto& obj : this->world->getObjects()) {
            if (this->checkCollision(*obj)) {
                this->position = oldPosition;
                this->velocity.y = 0;
                return;
            }
        }
    }
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
    if (this->position.y <= 0.5f) {
        this->position.y = 0.5f;
        this->velocity.y = 0;
        this->isOnGround = true;
    } else {
        this->isOnGround = false;
    }
}
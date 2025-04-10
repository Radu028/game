#include "Player.h"

#include "GameWorld.h"
#include "raylib.h"

Player::Player()
    : GameObject((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, BLUE, true),
      velocity((Vector3){0.0f, 0.0f, 0.0f}),
      isOnGround(true),
      world(nullptr) {}

void Player::handleInput(float movementSpeed, float jumpForce) {
    if (IsKeyDown(KEY_W)) this->move(FORWARD, movementSpeed);
    if (IsKeyDown(KEY_S)) this->move(BACKWARD, movementSpeed);
    if (IsKeyDown(KEY_A)) this->move(LEFT, movementSpeed);
    if (IsKeyDown(KEY_D)) this->move(RIGHT, movementSpeed);
    if (IsKeyPressed(KEY_SPACE)) this->move(UPWARD, jumpForce);
}

void Player::move(Direction direction, float byValue) {
    if (!this->world) return;
    Vector3 oldPosition = this->position;

    Vector3 newPosition = this->position;
    if (direction == FORWARD) {
        newPosition.z -= byValue;
    } else if (direction == BACKWARD) {
        newPosition.z += byValue;
    } else if (direction == LEFT) {
        newPosition.x -= byValue;
    } else if (direction == RIGHT) {
        newPosition.x += byValue;
    } else if (direction == UPWARD && this->isOnGround) {
        this->velocity.y = byValue;
        this->isOnGround = false;
    } else {
        return;
    }

    this->position = newPosition;
    for (const auto& obj : this->world->getObjects()) {
        if (this->checkCollision(*obj)) {
            this->position = oldPosition;
            return;
        }
    }
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
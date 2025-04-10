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
    }

    // Move along X axis with sliding collision
    if (newPosition.x != oldPosition.x) {
        moveWithSliding(oldPosition.x, newPosition.x, &this->position.x);
    }

    // Move along Z axis with sliding collision
    if (newPosition.z != oldPosition.z) {
        moveWithSliding(oldPosition.z, newPosition.z, &this->position.z);
    }
}

bool Player::checkCollisionWithWorld() const {
    for (const auto& obj : this->world->getObjects()) {
        if (this->checkCollision(*obj)) {
            return true;
        }
    }
    return false;
}

float Player::findMaxSafePosition(float start, float end, float* positionComponent) {
    const int MAX_ITERATIONS = 10;  // Maximum number of binary search iterations
    float moveFactor = 0.0f;        // How much of the full movement to apply
    float step = 0.5f;              // Binary search step size
    float originalPosition = *positionComponent;

    // Binary search for maximum safe distance
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        float testPosition = start + (end - start) * (moveFactor + step);

        // Try the test position
        *positionComponent = testPosition;
        bool hasCollision = checkCollisionWithWorld();

        if (hasCollision) {
            // Too far, try a smaller step
            step *= 0.5f;
        } else {
            // Can move further, increase moveFactor
            moveFactor += step;
            step *= 0.5f;
        }
    }

    // Reset position component before returning
    *positionComponent = originalPosition;
    return moveFactor;
}

void Player::moveWithSliding(float start, float end, float* positionComponent) {
    float originalValue = *positionComponent;

    // Try full movement first
    *positionComponent = end;

    // Check if we have a collision
    if (checkCollisionWithWorld()) {
        // Reset position and find maximum safe position
        *positionComponent = start;
        float moveFactor = findMaxSafePosition(start, end, positionComponent);

        // Apply the final safe position
        *positionComponent = start + (end - start) * moveFactor;
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
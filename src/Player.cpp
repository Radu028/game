#include "Player.h"

#include "GameWorld.h"
#include "raylib.h"
#include "CubeObject.h"

extern const float GRAVITY;

Player::Player(Vector3 position, Vector3 size, Color color)
    : CubeObject(position, size.x, size.y, size.z, color, true),
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
    if (!world) return;
    Vector3 oldPosition = position;

    Vector3 newPosition = position;
    if (direction == FORWARD) {
        newPosition.z -= byValue;
    } else if (direction == BACKWARD) {
        newPosition.z += byValue;
    } else if (direction == LEFT) {
        newPosition.x -= byValue;
    } else if (direction == RIGHT) {
        newPosition.x += byValue;
    } else if (direction == UPWARD && isOnGround) {
        velocity.y = byValue;
        isOnGround = false;
    }

    // Move along X axis with sliding collision
    if (newPosition.x != oldPosition.x) {
        moveWithSliding(oldPosition.x, newPosition.x, &position.x);
    }

    // Move along Z axis with sliding collision
    if (newPosition.z != oldPosition.z) {
        moveWithSliding(oldPosition.z, newPosition.z, &position.z);
    }
}

bool Player::checkCollisionWithWorld() const {
    for (const auto& obj : world->getObjects()) {
        if (checkCollision(*obj)) {
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
    if (!world) return;
    Vector3 oldPosition = position;

    velocity.y += gravity;
    position.y += velocity.y;

    if (checkCollisionWithWorld()) {
        position = oldPosition;
        velocity.y = 0;
        return;
    }
}

void Player::update(float deltaTime) {
    if (!world) return;

    Vector3 feetPosition = position;
    feetPosition.y -= height / 2.0f + 0.1f;

    bool onObject = false;
    for (const auto& obj : world->getObjects()) {
        if (obj.get() == this || !obj->getHasCollision()) continue;

        BoundingBox objectBox = obj->getBoundingBox();
        float objectTopY = objectBox.max.y;

        if (feetPosition.y <= objectTopY + 0.1f && feetPosition.x >= objectBox.min.x &&
            feetPosition.x <= objectBox.max.x && feetPosition.z >= objectBox.min.z &&
            feetPosition.z <= objectBox.max.z) {
            onObject = true;
            break;
        }
    }

    isOnGround = onObject;

    if (onObject && velocity.y <= 0) {
        velocity.y = 0;
    }

    applyGravity(GRAVITY);
}
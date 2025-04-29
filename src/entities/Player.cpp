#include "entities/Player.h"

#include <cmath>

#include "GameWorld.h"
#include "Physics.h"
#include "exceptions/CollisionException.h"
#include "raylib.h"
#include "systems/InputSystem.h"

Player::Player(Vector3 position)
    : GameObject(position, true),
      torso(position, {0.5f, 1.5f, 0.3f}, BLUE),
      head({0, 0, 0}, {0.5f, 0.5f, 0.5f}, RED),
      leftArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      rightArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      leftLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      rightLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      velocity({0.0f, 0.0f, 0.0f}),
      isOnGround(true),
      world(nullptr) {}

// Copy constructor
Player::Player(const Player &other)
    : GameObject(other.position, other.hasCollision),
      torso(other.torso),
      head(other.head),
      leftArm(other.leftArm),
      rightArm(other.rightArm),
      leftLeg(other.leftLeg),
      rightLeg(other.rightLeg),
      velocity(other.velocity),
      isOnGround(other.isOnGround),
      world(other.world) {}

std::shared_ptr<GameObject> Player::clone() const {
  return std::make_shared<Player>(*this);
}

BoundingBox Player::getBoundingBox() const {
  // Return the torso's bounding box
  return torso.getBoundingBox();
}

void Player::interact() {
  // Simple interaction - player jumps when interacted with
  if (isOnGround) {
    const_cast<Player *>(this)->jump();
  }
}

void Player::handleInput(float movementSpeed) {
  Vector2 moveAxis = InputSystem::getMovementAxis();

  if (moveAxis.y > 0.0f) move(FORWARD, movementSpeed);
  if (moveAxis.y < 0.0f) move(BACKWARD, movementSpeed);
  if (moveAxis.x < 0.0f) move(LEFT, movementSpeed);
  if (moveAxis.x > 0.0f) move(RIGHT, movementSpeed);

  if (InputSystem::isJumpPressed()) jump();
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

void Player::jump() {
  if (!isOnGround) return;
  velocity.y = PhysicsSettings::JUMP_VELOCITY;
  isOnGround = false;
}

bool Player::checkCollisionWithWorld() const {
  // Asigurăm-ne că torso are poziția actualizată
  torso.setPosition(position);

  for (const auto &obj : world->getObjects()) {
    // Skip player self-collision
    if (obj.get() == dynamic_cast<CubeObject *>(const_cast<Player *>(this)) ||
        !obj->getHasCollision()) {
      continue;
    }

    // Use torso for collision detection
    BoundingBox playerBox = torso.getBoundingBox();
    BoundingBox objBox = obj->getBoundingBox();

    if (CheckCollisionBoxes(playerBox, objBox)) {
      return true;
    }
  }
  return false;
}

float Player::findMaxSafePosition(float start, float end,
                                  float *positionComponent) {
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

void Player::moveWithSliding(float start, float end, float *positionComponent) {
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

void Player::applyGravity(float deltaTime) {
  if (!world) return;

  float clampedDelta = deltaTime;
  if (clampedDelta < PhysicsSettings::MIN_DELTA_TIME)
    clampedDelta = PhysicsSettings::MIN_DELTA_TIME;
  if (clampedDelta > PhysicsSettings::MAX_DELTA_TIME)
    clampedDelta = PhysicsSettings::MAX_DELTA_TIME;

  float scaledDelta = clampedDelta * PhysicsSettings::TIME_SCALE;
  velocity.y += PhysicsSettings::GRAVITY_ACCELERATION * scaledDelta;

  if (velocity.y < PhysicsSettings::MAX_FALL_SPEED) {
    velocity.y = PhysicsSettings::MAX_FALL_SPEED;
  }

  float initialY = position.y;
  float targetY = initialY + velocity.y * scaledDelta;

  position.y = targetY;
  bool collisionDetected = checkCollisionWithWorld();
  position.y = initialY;

  if (!collisionDetected) {
    position.y = targetY;
    return;
  }

  float t0 = 0.0f;
  float t1 = 1.0f;
  float tMid;

  const int MAX_ITERATIONS = 32;
  const float EPSILON = 0.001f;

  for (int i = 0; i < MAX_ITERATIONS && (t1 - t0) > EPSILON; i++) {
    tMid = (t0 + t1) / 2.0f;

    position.y = initialY + (targetY - initialY) * tMid;

    if (checkCollisionWithWorld()) {
      t1 = tMid;
    } else {
      t0 = tMid;
    }
  }

  position.y = initialY + (targetY - initialY) * t0;
  velocity.y = 0;
}

void Player::update(float deltaTime) {
  if (!world) return;

  applyGravity(deltaTime);

  torso.setPosition(position);

  BoundingBox torsoBox = torso.getBoundingBox();
  float halfWidth = (torsoBox.max.x - torsoBox.min.x) / 2.0f;
  float halfLength = (torsoBox.max.z - torsoBox.min.z) / 2.0f;
  float feetY = torsoBox.min.y - 0.01f;

  // Define a grid of points to check on the player's base
  const int GRID_SIZE = 3;  // 3x3 grid (9 puncte)
  bool onObject = false;

  for (const auto &obj : world->getObjects()) {
    if (obj.get() == dynamic_cast<CubeObject *>(this) ||
        !obj->getHasCollision())
      continue;

    BoundingBox objectBox = obj->getBoundingBox();
    float objectTopY = objectBox.max.y;

    // Check grid of points across the entire base of the player
    for (int i = 0; i < GRID_SIZE; i++) {
      for (int j = 0; j < GRID_SIZE; j++) {
        // Calculate position for this grid point (evenly distributed)
        float xOffset = -halfWidth + ((torsoBox.max.x - torsoBox.min.x) * i /
                                      (GRID_SIZE - 1));
        float zOffset = -halfLength + ((torsoBox.max.z - torsoBox.min.z) * j /
                                       (GRID_SIZE - 1));

        Vector3 checkPoint = {position.x + xOffset, feetY,
                              position.z + zOffset};

        if (checkPoint.y <= objectTopY + 0.01f &&
            checkPoint.x >= objectBox.min.x &&
            checkPoint.x <= objectBox.max.x &&
            checkPoint.z >= objectBox.min.z &&
            checkPoint.z <= objectBox.max.z) {
          onObject = true;
          break;
        }
      }
      if (onObject) break;
    }
    if (onObject) break;
  }

  isOnGround = onObject;

  if (onObject && velocity.y <= 0) {
    velocity.y = 0;
  }

  static float animTime = 0.0f;
  if (IsKeyDown(KEY_W) || IsKeyDown(KEY_S) || IsKeyDown(KEY_A) ||
      IsKeyDown(KEY_D)) {
    animTime += deltaTime * 5.0f;

    float swingAngle = sin(animTime) * 30.0f;

    leftArm.setRotation((Vector3){1, 0, 0}, -swingAngle);
    rightArm.setRotation((Vector3){1, 0, 0}, swingAngle);
    leftLeg.setRotation((Vector3){1, 0, 0}, swingAngle);
    rightLeg.setRotation((Vector3){1, 0, 0}, -swingAngle);
  } else {
    leftArm.setRotation((Vector3){1, 0, 0}, 0);
    rightArm.setRotation((Vector3){1, 0, 0}, 0);
    leftLeg.setRotation((Vector3){1, 0, 0}, 0);
    rightLeg.setRotation((Vector3){1, 0, 0}, 0);
    animTime = 0.0f;
  }
}

void Player::draw() const {
  Vector3 torsoPos = position;
  torso.setPosition(torsoPos);

  BoundingBox torsoBox = torso.getBoundingBox();
  float torsoHeight = torsoBox.max.y - torsoBox.min.y;

  torso.draw();

  Vector3 headPos = position;
  headPos.y += torsoHeight / 2 + 0.25f;
  head.setPosition(headPos);
  head.draw();

  float shoulderY = position.y + torsoHeight / 3;
  float shoulderX = torsoBox.max.x - torsoBox.min.x;

  Vector3 leftArmPos = {position.x - shoulderX / 1.5f, shoulderY, position.z};
  Vector3 rightArmPos = {position.x + shoulderX / 1.5f, shoulderY, position.z};

  leftArm.setPosition(leftArmPos);
  rightArm.setPosition(rightArmPos);

  leftArm.draw();
  rightArm.draw();

  float hipY = position.y - torsoHeight / 2;
  float hipX = shoulderX / 2;

  Vector3 leftLegPos = {position.x - hipX / 2, hipY, position.z};
  Vector3 rightLegPos = {position.x + hipX / 2, hipY, position.z};

  leftLeg.setPosition(leftLegPos);
  rightLeg.setPosition(rightLegPos);

  leftLeg.draw();
  rightLeg.draw();
}
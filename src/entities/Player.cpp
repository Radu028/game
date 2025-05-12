#include "entities/Player.h"

#include <cmath>
#include <memory>

#include "GameWorld.h"
#include "raylib.h"
#include "raymath.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"

const float PLAYER_MOVEMENT_SPEED = 5.0f;
const float PLAYER_ANIMATION_SPEED = 10.0f;
const float PLAYER_SWING_ANGLE_DEGREES = 20.0f;

Player::Player(Vector3 position)
    : GameObject(position, true, true, false),
      torso(position, {0.5f, 1.5f, 0.3f}, BLUE),
      head({0, 0, 0}, {0.5f, 0.5f, 0.5f}, RED),
      leftArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      rightArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      leftLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      rightLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      world(nullptr) {}

void Player::handleInput(float movementSpeed) {
  Vector2 moveAxis = InputSystem::getMovementAxis();

  if (moveAxis.y > 0.0f) move(FORWARD, movementSpeed * GetFrameTime());
  if (moveAxis.y < 0.0f) move(BACKWARD, movementSpeed * GetFrameTime());
  if (moveAxis.x < 0.0f) move(LEFT, movementSpeed * GetFrameTime());
  if (moveAxis.x > 0.0f) move(RIGHT, movementSpeed * GetFrameTime());

  if (InputSystem::isJumpPressed()) jump();
}

void Player::move(Direction direction, float byValue) {
  if (!world) return;

  Vector3 oldPosition = position;
  Vector3 newPosition = position;

  if (direction == FORWARD)
    newPosition.z -= byValue;
  else if (direction == BACKWARD)
    newPosition.z += byValue;
  else if (direction == LEFT)
    newPosition.x -= byValue;
  else if (direction == RIGHT)
    newPosition.x += byValue;

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
  Vector3 currentVelocity = getVelocity();
  currentVelocity.y = PhysicsSettings::JUMP_VELOCITY;
  setVelocity(currentVelocity);
  setIsOnGround(false);
}

bool Player::checkCollisionWithWorldHorizontal() const {
  if (!world) return false;

  BodyPart currentTorso = torso;
  currentTorso.setPosition(this->position);
  BoundingBox playerBox = currentTorso.getBoundingBox();

  for (const auto &objSharedPtr : world->getObjects()) {
    const GameObject *otherObj = objSharedPtr.get();
    if (!otherObj || otherObj == this || !otherObj->getHasCollision() ||
        otherObj->getIsStatic()) {
      continue;
    }

    if (CheckCollisionBoxes(playerBox, otherObj->getBoundingBox())) {
      return true;
    }
  }
  return false;
}

void Player::moveWithSliding(float start, float end, float *positionComponent) {
  float originalValue = *positionComponent;

  // Try full movement first
  *positionComponent = end;

  // Check if we have a collision
  if (checkCollisionWithWorldHorizontal()) {
    // Reset position and find maximum safe position
    *positionComponent = originalValue;

    float t0 = 0.0f;
    float t1 = 1.0f;
    float tMid;
    const int MAX_ITERATIONS = 10;

    for (int i = 0; i < MAX_ITERATIONS && (t1 - t0) > EPSILON; i++) {
      tMid = (t0 + t1) / 2.0f;
      *positionComponent = start + (end - start) * tMid;  // Test this position
      if (checkCollisionWithWorldHorizontal()) {
        t1 = tMid;  // Collision, try earlier
      } else {
        t0 = tMid;  // No collision, can go at least this far
      }
    }
    *positionComponent =
        start + (end - start) * t0;  // Apply safest found position
  }
}

void Player::performDetailedGroundCheck() {
  if (!world) {
    setIsOnGround(false);
    return;
  }

  BodyPart currentTorso = torso;
  currentTorso.setPosition(this->position);
  BoundingBox playerBaseBox = currentTorso.getBoundingBox();

  float checkDepth = playerBaseBox.min.y - 0.1f;

  bool foundGround = false;
  for (const auto &objSharedPtr : world->getObjects()) {
    const GameObject *otherObj = objSharedPtr.get();
    if (!otherObj || !otherObj->getHasCollision() ||
        otherObj->getIsStatic() == false) {
      // TODO: Check later this.
      if (!otherObj || !otherObj->getHasCollision()) continue;
    }

    BoundingBox otherBox = otherObj->getBoundingBox();

    bool yAlign = (playerBaseBox.min.y >= otherBox.max.y - 0.05f) &&
                  (playerBaseBox.min.y <= otherBox.max.y + 0.2f);

    bool xzOverlap = (playerBaseBox.max.x > otherBox.min.x &&
                      playerBaseBox.min.x < otherBox.max.x) &&
                     (playerBaseBox.max.z > otherBox.min.z &&
                      playerBaseBox.min.z < otherBox.max.z);

    if (yAlign && xzOverlap) {
      if (std::abs(this->position.y -
                   (playerBaseBox.max.y - playerBaseBox.min.y) / 2 -
                   otherBox.max.y) < 0.1f) {
        foundGround = true;
        Vector3 pPos = getPosition();

        float playerHalfHeight =
            (getBoundingBox().max.y - getBoundingBox().min.y) / 2.0f;
        pPos.y = otherBox.max.y + playerHalfHeight + EPSILON;
        setPosition(pPos);

        Vector3 currentVel = getVelocity();
        if (currentVel.y < 0) {
          currentVel.y = 0;
          setVelocity(currentVel);
        }

        break;  // Found ground
      }
    }
  }
  setIsOnGround(foundGround);
}

void Player::update(float deltaTime) {
  if (!world) return;

  performDetailedGroundCheck();

  handleInput(PLAYER_MOVEMENT_SPEED);

  torso.setPosition(position);

  Vector3 torsoPos = torso.getPosition();
  Vector3 torsoSize = torso.getSize();
  Vector3 headSize = head.getSize();

  float torsoHeight = torsoSize.y;
  head.setPosition({torsoPos.x,
                    torsoPos.y + torsoHeight / 2.0f + headSize.y / 2.0f,
                    torsoPos.z});

  Vector3 leftArmSize = leftArm.getSize();
  float armOffsetX = torsoSize.x / 2.0f + leftArmSize.x / 2.0f;
  leftArm.setPosition({torsoPos.x - armOffsetX, torsoPos.y, torsoPos.z});

  rightArm.setPosition({torsoPos.x + armOffsetX, torsoPos.y, torsoPos.z});

  Vector3 leftLegSize = leftLeg.getSize();
  float legOffsetY = torsoHeight / 2.0f + leftLegSize.y / 2.0f;
  float legOffsetX = torsoSize.x / 4.0f;
  leftLeg.setPosition(
      {torsoPos.x - legOffsetX, torsoPos.y - legOffsetY, torsoPos.z});
  rightLeg.setPosition(
      {torsoPos.x + legOffsetX, torsoPos.y - legOffsetY, torsoPos.z});

  static float animTime = 0.0f;
  Vector2 moveInput = InputSystem::getMovementAxis();
  bool isMoving = (moveInput.x != 0.0f || moveInput.y != 0.0f);
  if (isMoving) {
    animTime += deltaTime * PLAYER_ANIMATION_SPEED;
    float angleDegrees = sin(animTime) * PLAYER_SWING_ANGLE_DEGREES;
    const Vector3 swingAxis = {1.0f, 0.0f, 0.0f};

    leftArm.setRotation(swingAxis, angleDegrees);
    rightArm.setRotation(swingAxis, -angleDegrees);
    leftLeg.setRotation(swingAxis, -angleDegrees);
    rightLeg.setRotation(swingAxis, angleDegrees);
  } else {
    const Vector3 neutralAxis = {1.0f, 0.0f, 0.0f};
    const float neutralAngle = 0.0f;

    leftArm.setRotation(neutralAxis, neutralAngle);
    rightArm.setRotation(neutralAxis, neutralAngle);
    leftLeg.setRotation(neutralAxis, neutralAngle);
    rightLeg.setRotation(neutralAxis, neutralAngle);
  }
}

void Player::draw() const {
  torso.draw();
  head.draw();
  leftArm.draw();
  rightArm.draw();
  leftLeg.draw();
  rightLeg.draw();
}

BoundingBox Player::getBoundingBox() const { return torso.getBoundingBox(); }

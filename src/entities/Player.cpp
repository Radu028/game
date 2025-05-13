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
const float PLAYER_RETURN_TO_NEUTRAL_SPEED = 7.0f;

Player::Player(Vector3 position)
    : GameObject(position, true, true, false),
      torso(position, {0.5f, 1.5f, 0.3f}, BLUE),
      head({0, 0, 0}, {0.5f, 0.5f, 0.5f}, RED),
      leftArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      rightArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN),
      leftLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      rightLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW),
      world(nullptr) {}

void Player::updateBodyPartPositions() {
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
}

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

  // TODO: Maybe move this directly in getBounginBox()?
  std::vector<BoundingBox> playerPartBoxes;
  playerPartBoxes.push_back(torso.getBoundingBox());
  playerPartBoxes.push_back(leftArm.getBoundingBox());
  playerPartBoxes.push_back(rightArm.getBoundingBox());
  playerPartBoxes.push_back(leftLeg.getBoundingBox());
  playerPartBoxes.push_back(rightLeg.getBoundingBox());
  playerPartBoxes.push_back(head.getBoundingBox());

  for (const auto &objSharedPtr : world->getObjects()) {
    const GameObject *otherObj = objSharedPtr.get();
    if (!otherObj || otherObj == this || !otherObj->getHasCollision()) {
      continue;
    }

    BoundingBox otherBox = otherObj->getBoundingBox();

    for (const auto &partBox : playerPartBoxes) {
      if (CheckCollisionBoxes(partBox, otherBox)) {
        return true;
      }
    }
  }
  return false;
}

void Player::moveWithSliding(float start, float end, float *positionComponent) {
  float originalValue = *positionComponent;

  // Try full movement first
  *positionComponent = end;
  updateBodyPartPositions();

  // Check if we have a collision
  if (checkCollisionWithWorldHorizontal()) {
    // Reset position and find maximum safe position
    *positionComponent = originalValue;
    updateBodyPartPositions();

    float t0 = 0.0f;
    float t1 = 1.0f;
    float tMid;
    const int MAX_ITERATIONS = 10;

    for (int i = 0; i < MAX_ITERATIONS && (t1 - t0) > EPSILON; i++) {
      tMid = (t0 + t1) / 2.0f;
      *positionComponent = start + (end - start) * tMid;  // Test this position
      updateBodyPartPositions();
      if (checkCollisionWithWorldHorizontal()) {
        t1 = tMid;  // Collision, try earlier
      } else {
        t0 = tMid;  // No collision, can go at least this far
      }
    }
    *positionComponent =
        start + (end - start) * t0;  // Apply safest found position
    updateBodyPartPositions();
  }
}

void Player::performDetailedGroundCheck() {
  if (!world) {
    setIsOnGround(false);
    return;
  }

  BoundingBox leftLegBox = leftLeg.getBoundingBox();
  BoundingBox rightLegBox = rightLeg.getBoundingBox();

  bool foundGroundThisFrame = false;
  float highestGroundYContact = -std::numeric_limits<float>::infinity();
  Vector3 newPlayerPosition = getPosition();
  bool positionNeedsAdjustment = false;

  for (const auto &objSharedPtr : world->getObjects()) {
    const GameObject *otherObj = objSharedPtr.get();
    // TODO: Check later this. Maybe implement a 2 frame colision check.
    if (!otherObj || otherObj == this || !otherObj->getHasCollision()) {
      continue;
    }

    BoundingBox otherBox = otherObj->getBoundingBox();

    std::vector<const BoundingBox *> legBoxes = {&leftLegBox, &rightLegBox};

    for (const BoundingBox *legBoxPtr : legBoxes) {
      const BoundingBox &currentLegBox = *legBoxPtr;

      bool yAlign = (currentLegBox.min.y >= otherBox.max.y - 0.05f) &&
                    (currentLegBox.min.y <= otherBox.max.y + 0.2f);

      bool xzOverlap = (currentLegBox.max.x > otherBox.min.x &&
                        currentLegBox.min.x < otherBox.max.x) &&
                       (currentLegBox.max.z > otherBox.min.z &&
                        currentLegBox.min.z < otherBox.max.z);

      if (yAlign && xzOverlap) {
        if (std::abs(currentLegBox.min.y - otherBox.max.y) < 0.1f) {
          foundGroundThisFrame = true;
          Vector3 currentPlayerOrigin = getPosition();

          float distPlayerOriginToLegBottom =
              currentPlayerOrigin.y - currentLegBox.min.y;
          float targetPlayerOriginY =
              otherBox.max.y + distPlayerOriginToLegBottom;

          if (targetPlayerOriginY > highestGroundYContact) {
            highestGroundYContact = targetPlayerOriginY;
            newPlayerPosition.y = targetPlayerOriginY + EPSILON;
            positionNeedsAdjustment = true;
          }
        }
      }
    }
  }

  if (foundGroundThisFrame && positionNeedsAdjustment) {
    setPosition(newPlayerPosition);
    Vector3 currentVelocity = getVelocity();
    if (currentVelocity.y < 0) {
      currentVelocity.y = 0;
      setVelocity(currentVelocity);
    }
  }

  setIsOnGround(foundGroundThisFrame);
}

void Player::update(float deltaTime) {
  if (!world) return;

  performDetailedGroundCheck();

  handleInput(PLAYER_MOVEMENT_SPEED);
  updateBodyPartPositions();

  const Vector3 swingAxis = {1.0f, 0.0f, 0.0f};
  static float animTime = 0.0f;
  Vector2 moveInput = InputSystem::getMovementAxis();
  bool isMoving = (moveInput.x != 0.0f || moveInput.y != 0.0f);
  if (isMoving) {
    animTime += deltaTime * PLAYER_ANIMATION_SPEED;
    float angleDegrees = sin(animTime) * PLAYER_SWING_ANGLE_DEGREES;

    leftArm.setRotation(swingAxis, angleDegrees);
    rightArm.setRotation(swingAxis, -angleDegrees);
    leftLeg.setRotation(swingAxis, -angleDegrees);
    rightLeg.setRotation(swingAxis, angleDegrees);
  } else {
    const float NEUTRAL_ANGLE = 0.0f;
    const float ANGLE_TRESHOLD = 0.1f;

    auto returnLimbToNeutral = [&](BodyPart &limb) {
      float currentAngle = limb.getRotationAngle();
      if (std::abs(currentAngle - NEUTRAL_ANGLE) > ANGLE_TRESHOLD) {
        float newAngle = Lerp(currentAngle, NEUTRAL_ANGLE,
                              PLAYER_RETURN_TO_NEUTRAL_SPEED * deltaTime);
        limb.setRotation(swingAxis, newAngle);
      } else {
        limb.setRotation(swingAxis, NEUTRAL_ANGLE);
      }
    };

    returnLimbToNeutral(leftArm);
    returnLimbToNeutral(rightArm);
    returnLimbToNeutral(leftLeg);
    returnLimbToNeutral(rightLeg);
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

BoundingBox Player::getBoundingBox() const {
  BoundingBox combinedBox = torso.getBoundingBox();
  BoundingBox headBox = head.getBoundingBox();
  BoundingBox leftArmBox = leftArm.getBoundingBox();
  BoundingBox rightArmBox = rightArm.getBoundingBox();
  BoundingBox leftLegBox = leftLeg.getBoundingBox();
  BoundingBox rightLegBox = rightLeg.getBoundingBox();

  combinedBox.min = Vector3Min(combinedBox.min, headBox.min);
  combinedBox.max = Vector3Max(combinedBox.max, headBox.max);

  combinedBox.min = Vector3Min(combinedBox.min, leftArmBox.min);
  combinedBox.max = Vector3Max(combinedBox.max, leftArmBox.max);

  combinedBox.min = Vector3Min(combinedBox.min, rightArmBox.min);
  combinedBox.max = Vector3Max(combinedBox.max, rightArmBox.max);

  combinedBox.min = Vector3Min(combinedBox.min, leftLegBox.min);
  combinedBox.max = Vector3Max(combinedBox.max, leftLegBox.max);

  combinedBox.min = Vector3Min(combinedBox.min, rightLegBox.min);
  combinedBox.max = Vector3Max(combinedBox.max, rightLegBox.max);

  return combinedBox;
}

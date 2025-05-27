// HumanoidCharacter.cpp - Solid humanoid character controller (no ragdoll)
#include "entities/HumanoidCharacter.h"

#include <cmath>

#include "GameWorld.h"
#include "raylib.h"
#include "raymath.h"
#include "systems/InputSystem.h"

int HumanoidCharacter::totalCharactersCreated = 0;
int HumanoidCharacter::activeCharacters = 0;

float HumanoidCharacter::calculateDistance(const HumanoidCharacter& char1,
                                           const HumanoidCharacter& char2) {
  Vector3 pos1 = char1.getPosition();
  Vector3 pos2 = char2.getPosition();
  return Vector3Distance(pos1, pos2);
}

Vector3 HumanoidCharacter::getCharacterSpacing(int characterCount) {
  if (characterCount <= 1) return {0.0f, 0.0f, 0.0f};
  float spacing =
      3.0f + (characterCount * 0.5f);  // Increase spacing with more characters
  return {spacing, 0.0f, spacing};
}

bool HumanoidCharacter::areCharactersOverlapping(
    const HumanoidCharacter& char1, const HumanoidCharacter& char2) {
  return calculateDistance(char1, char2) < 2.0f;  // Less than 2 units apart
}

HumanoidCharacter::HumanoidCharacter(Vector3 position)
    : GameObject(position, true, true, false),
      // Initialize body parts with calculated offsets
      head(GameSettings::BodyParts::HEAD_SIZE,
           GameSettings::BodyParts::HEAD_COLOR,
           {0, GameSettings::CharacterCalculations::getHeadBaseYOffset(), 0}),
      torso(GameSettings::BodyParts::TORSO_SIZE,
            GameSettings::BodyParts::TORSO_COLOR,
            {0, GameSettings::CharacterCalculations::getTorsoBaseYOffset(), 0}),
      leftArm(GameSettings::BodyParts::ARM_SIZE,
              GameSettings::BodyParts::ARM_COLOR,
              {-GameSettings::BodyParts::ARM_OFFSET_X,
               GameSettings::CharacterCalculations::getArmBaseYOffset(), 0}),
      rightArm(GameSettings::BodyParts::ARM_SIZE,
               GameSettings::BodyParts::ARM_COLOR,
               {GameSettings::BodyParts::ARM_OFFSET_X,
                GameSettings::CharacterCalculations::getArmBaseYOffset(), 0}),
      leftLeg(GameSettings::BodyParts::LEG_SIZE,
              GameSettings::BodyParts::LEG_COLOR,
              {-GameSettings::BodyParts::LEG_OFFSET_X,
               GameSettings::CharacterCalculations::getLegBaseYOffset(), 0}),
      rightLeg(GameSettings::BodyParts::LEG_SIZE,
               GameSettings::BodyParts::LEG_COLOR,
               {GameSettings::BodyParts::LEG_OFFSET_X,
                GameSettings::CharacterCalculations::getLegBaseYOffset(), 0}),
      // Initialize physics components for body parts
      headPhysics(
          GameSettings::BodyParts::HEAD_SIZE,
          {0, GameSettings::CharacterCalculations::getHeadBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_HEAD),
      torsoPhysics(
          GameSettings::BodyParts::TORSO_SIZE,
          {0, GameSettings::CharacterCalculations::getTorsoBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_TORSO),
      leftArmPhysics(
          GameSettings::BodyParts::ARM_SIZE,
          {-GameSettings::BodyParts::ARM_OFFSET_X,
           GameSettings::CharacterCalculations::getArmBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_ARMS),
      rightArmPhysics(
          GameSettings::BodyParts::ARM_SIZE,
          {GameSettings::BodyParts::ARM_OFFSET_X,
           GameSettings::CharacterCalculations::getArmBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_ARMS),
      leftLegPhysics(
          GameSettings::BodyParts::LEG_SIZE,
          {-GameSettings::BodyParts::LEG_OFFSET_X,
           GameSettings::CharacterCalculations::getLegBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_LEGS),
      rightLegPhysics(
          GameSettings::BodyParts::LEG_SIZE,
          {GameSettings::BodyParts::LEG_OFFSET_X,
           GameSettings::CharacterCalculations::getLegBaseYOffset(), 0},
          GameSettings::Collision::Groups::CHARACTER_LEGS),
      // Eye features positioned on front of head
      leftEye(
          {0.08f, 0.08f, 0.12f}, BLACK,
          {-0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.08f,
           0.25f}),
      rightEye(
          {0.08f, 0.08f, 0.12f}, BLACK,
          {0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.08f,
           0.25f}),
      mouth(
          {0.15f, 0.04f, 0.08f}, MAROON,
          {0, GameSettings::CharacterCalculations::getHeadBaseYOffset() - 0.08f,
           0.25f}),
      // Facial features for character expression
      leftEyebrow(
          {0.10f, 0.02f, 0.06f}, DARKBROWN,
          {-0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.15f,
           0.26f}),
      rightEyebrow(
          {0.10f, 0.02f, 0.06f}, DARKBROWN,
          {0.12f,
           GameSettings::CharacterCalculations::getHeadBaseYOffset() + 0.15f,
           0.26f}) {
  totalCharactersCreated++;
  activeCharacters++;
}

HumanoidCharacter::~HumanoidCharacter() {
  activeCharacters--;

  if (physicsWorld && characterBody) {
    physicsWorld->removeRigidBody(characterBody);
  }

  // Remove all individual physics bodies
  removeAllPhysicsBodies();

  delete characterBody;
  delete characterShape;
  delete motionState;
}

void HumanoidCharacter::createSinglePhysicsBody() {
  // Use dimensions from settings
  float radius = GameSettings::Character::RADIUS;
  float height = GameSettings::Character::HEIGHT;

  characterShape = new btCapsuleShape(radius, height);
  characterShape->setMargin(GameSettings::Collision::SHAPE_MARGIN);

  // Position character body so feet touch the ground
  // position.y represents foot level, physics center is offset accordingly
  btTransform transform;
  transform.setIdentity();

  // Ensure the physics body is properly positioned above ground
  float physicsCenterY =
      GameSettings::CharacterCalculations::getPhysicsCenterYFromGroundY(
          position.y);

  transform.setOrigin(btVector3(position.x, physicsCenterY, position.z));

  motionState = new btDefaultMotionState(transform);

  float mass = GameSettings::Character::MASS;
  btVector3 inertia(0, 0, 0);
  characterShape->calculateLocalInertia(mass, inertia);

  // Create rigid body with physics settings
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState,
                                                  characterShape, inertia);
  rbInfo.m_friction = GameSettings::Collision::FRICTION;
  rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
  rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;

  characterBody = new btRigidBody(rbInfo);

  // Use settings for damping
  characterBody->setDamping(GameSettings::Character::DAMPING_LINEAR,
                            GameSettings::Character::DAMPING_ANGULAR);
  characterBody->setActivationState(DISABLE_DEACTIVATION);
  characterBody->setSleepingThresholds(0.0f, 0.0f);

  // Lock rotation around X and Z axes to prevent tipping
  characterBody->setAngularFactor(btVector3(0, 1, 0));

  // Set gravity
  characterBody->setGravity(
      btVector3(0, GameSettings::Physics::GRAVITY_ACCELERATION, 0));
}

void HumanoidCharacter::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
  physicsWorld = bulletWorld;
  createSinglePhysicsBody();

  // Add the main character body to the world (torso-based movement)
  // Main character body should collide with world objects
  short characterGroup = GameSettings::Collision::Groups::CHARACTER_TORSO;
  short characterMask =
      GameSettings::Collision::Groups::WORLD_OBJECTS;  // Fix: collide with
                                                       // world objects
  physicsWorld->addRigidBody(characterBody, characterGroup, characterMask);

  // Create individual physics bodies for each body part
  createIndividualPhysicsBodies();
}

void HumanoidCharacter::removeFromPhysics(
    btDiscreteDynamicsWorld* bulletWorld) {
  if (bulletWorld && characterBody) {
    bulletWorld->removeRigidBody(characterBody);
  }

  // Remove all individual physics bodies
  removeAllPhysicsBodies();
}

void HumanoidCharacter::update(float deltaTime) {
  if (!characterBody) return;

  if (jumpCooldown > 0.0f) {
    jumpCooldown -= deltaTime;
  }

  updateCharacterState();

  animateCharacter(deltaTime);

  // Update visual positions and facial features (combined for efficiency)
  updateVisualPositions();

  // Synchronize physics bodies with visual positions for collision detection
  synchronizeVisualWithPhysics();

  // Facial features are now updated inside updateVisualPositions()
}

void HumanoidCharacter::updateCharacterState() {
  Vector3 velocity = {0, 0, 0};
  if (characterBody) {
    btVector3 btVel = characterBody->getLinearVelocity();
    velocity = {btVel.getX(), btVel.getY(), btVel.getZ()};
  }

  // Reset jumping state when definitely landed - balanced conditions
  if (isJumping && isOnGround() && abs(velocity.y) < 0.5f) {
    isJumping = false;
  }

  float horizontalSpeed =
      sqrt(velocity.x * velocity.x + velocity.z * velocity.z);

  if (isJumping || !isOnGround() ||
      abs(velocity.y) > 1.0f) {  // Balanced jumping detection
    currentState = JUMPING;
  } else if (horizontalSpeed > 0.15f) {
    currentState = WALKING;
  } else {
    currentState = IDLE;
  }
}

void HumanoidCharacter::animateCharacter(float deltaTime) {
  animationTime += deltaTime;

  switch (currentState) {
    case IDLE:
      // Subtle breathing animation
      head.currentOffset = Vector3Add(
          head.baseOffset, {0, sin(animationTime * 2.0f) * 0.01f, 0});
      torso.currentOffset = torso.baseOffset;
      leftArm.currentOffset = leftArm.baseOffset;
      rightArm.currentOffset = rightArm.baseOffset;
      leftLeg.currentOffset = leftLeg.baseOffset;
      rightLeg.currentOffset = rightLeg.baseOffset;
      break;

    case WALKING: {
      // Walking animation with limb movement
      float walkCycle = animationTime * 8.0f;   // Walking cycle frequency
      float armSwingL = sin(walkCycle) * 0.4f;  // Left arm swing
      float armSwingR =
          sin(walkCycle + M_PI) * 0.4f;         // Right arm swing (opposite)
      float legSwingL = sin(walkCycle) * 0.3f;  // Left leg swing
      float legSwingR =
          sin(walkCycle + M_PI) * 0.3f;  // Right leg swing (opposite)

      // Head bobbing
      head.currentOffset =
          Vector3Add(head.baseOffset, {0, sin(walkCycle * 2.0f) * 0.05f, 0});

      // Torso movement for walking
      torso.currentOffset = Vector3Add(
          torso.baseOffset,
          {sin(walkCycle) * 0.02f, sin(walkCycle * 2.0f) * 0.03f, 0});

      // Arms swing naturally
      leftArm.currentOffset = Vector3Add(
          leftArm.baseOffset, {0, armSwingL * 0.3f, armSwingL * 0.6f});
      rightArm.currentOffset = Vector3Add(
          rightArm.baseOffset, {0, armSwingR * 0.3f, armSwingR * 0.6f});

      // Legs with natural walking motion - lifting and moving forward/back
      float leftLegLift =
          fmax(0, sin(walkCycle)) * 0.15f;  // Only lift when moving forward
      float rightLegLift = fmax(0, sin(walkCycle + M_PI)) *
                           0.15f;  // Only lift when moving forward

      leftLeg.currentOffset =
          Vector3Add(leftLeg.baseOffset,
                     {legSwingL * 0.1f, leftLegLift, legSwingL * 0.2f});
      rightLeg.currentOffset =
          Vector3Add(rightLeg.baseOffset,
                     {legSwingR * 0.1f, rightLegLift, legSwingR * 0.2f});
      break;
    }

    case JUMPING:
      // Jump pose - arms up slightly, legs together
      head.currentOffset = head.baseOffset;
      torso.currentOffset = torso.baseOffset;
      leftArm.currentOffset = Vector3Add(leftArm.baseOffset, {0, 0.1f, 0});
      rightArm.currentOffset = Vector3Add(rightArm.baseOffset, {0, 0.1f, 0});
      leftLeg.currentOffset = Vector3Add(leftLeg.baseOffset, {0.05f, 0, 0});
      rightLeg.currentOffset = Vector3Add(rightLeg.baseOffset, {-0.05f, 0, 0});
      break;
  }
}

void HumanoidCharacter::updateVisualPositions() {
  if (!characterBody) return;

  // Get physics body transform ONCE per frame - SINGLE SOURCE OF TRUTH
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();
  Vector3 physicsPos = {origin.getX(), origin.getY(), origin.getZ()};

  // Update main position once per frame (at feet level)
  // Calculate ground position from physics body center
  position = {origin.getX(),
              GameSettings::CharacterCalculations::getGroundYFromPhysicsCenterY(
                  origin.getY()),
              origin.getZ()};

  // Get rotation (only Y-axis rotation allowed)
  btQuaternion rotation = transform.getRotation();
  float yRotation = atan2(2.0f * (rotation.getW() * rotation.getY() +
                                  rotation.getX() * rotation.getZ()),
                          1.0f - 2.0f * (rotation.getY() * rotation.getY() +
                                         rotation.getZ() * rotation.getZ()));

  // Convert rotation to degrees for BodyPart's setRotation method
  float yRotationDegrees = yRotation * RAD2DEG;

  // Update visual parts positions and rotations
  Vector3 rotatedOffset;

  // Head position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(head.currentOffset, {0, 1, 0}, yRotation);
  head.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  head.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Torso position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(torso.currentOffset, {0, 1, 0}, yRotation);
  torso.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  torso.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Left Arm position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(leftArm.currentOffset, {0, 1, 0}, yRotation);
  leftArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftArm.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Right Arm position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(rightArm.currentOffset, {0, 1, 0}, yRotation);
  rightArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightArm.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Left Leg position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(leftLeg.currentOffset, {0, 1, 0}, yRotation);
  leftLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftLeg.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Right Leg position and rotation
  rotatedOffset =
      Vector3RotateByAxisAngle(rightLeg.currentOffset, {0, 1, 0}, yRotation);
  rightLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightLeg.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Facial features transform
  rotatedOffset =
      Vector3RotateByAxisAngle(leftEye.currentOffset, {0, 1, 0}, yRotation);
  leftEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftEye.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset =
      Vector3RotateByAxisAngle(rightEye.currentOffset, {0, 1, 0}, yRotation);
  rightEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightEye.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset =
      Vector3RotateByAxisAngle(mouth.currentOffset, {0, 1, 0}, yRotation);
  mouth.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  mouth.visual.setRotation({0, 1, 0}, yRotationDegrees);

  // Eyebrow positioning
  rotatedOffset =
      Vector3RotateByAxisAngle(leftEyebrow.currentOffset, {0, 1, 0}, yRotation);
  leftEyebrow.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  leftEyebrow.visual.setRotation({0, 1, 0}, yRotationDegrees);

  rotatedOffset = Vector3RotateByAxisAngle(rightEyebrow.currentOffset,
                                           {0, 1, 0}, yRotation);
  rightEyebrow.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
  rightEyebrow.visual.setRotation({0, 1, 0}, yRotationDegrees);
}

// updateFacialFeatures() method removed - now integrated into
// updateVisualPositions() for efficiency

void HumanoidCharacter::handleInput(float movementSpeed, float cameraAngleX) {
  if (!characterBody) return;

  Vector3 movement = {0, 0, 0};

  // Get input axes
  float inputX = 0.0f;
  float inputZ = 0.0f;

  if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) {
    inputZ += 1.0f;
  }
  if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) {
    inputZ -= 1.0f;
  }
  if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) {
    inputX -= 1.0f;
  }
  if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) {
    inputX += 1.0f;
  }

  float cameraRad = cameraAngleX * DEG2RAD;

  // In Roblox-style movement:
  // - Camera is positioned at angle cameraAngleX around the player
  // - Forward direction is FROM camera position TOWARDS player (negative of
  // camera offset)
  // - Camera offset is: X = sin(angle), Z = cos(angle)
  // - So forward direction is: X = -sin(angle), Z = -cos(angle)

  float forwardX = -sinf(cameraRad);  // Direction towards where camera looks
  float forwardZ = -cosf(cameraRad);

  // Right direction is perpendicular to forward (90 degrees rotation)
  float rightX =
      cosf(cameraRad);  // Corrected: positive for proper right direction
  float rightZ =
      -sinf(cameraRad);  // Corrected: negative for proper right direction

  // Calculate movement relative to camera direction (Roblox-style)
  movement.x = inputZ * forwardX + inputX * rightX;
  movement.z = inputZ * forwardZ + inputX * rightZ;

  // Jump input
  if (IsKeyPressed(KEY_SPACE)) {
    jump();
  }

  // Apply movement
  if (Vector3Length(movement) > 0.1f) {
    movement = Vector3Normalize(movement);
    applyMovementForces(movement, movementSpeed);
  }
}

void HumanoidCharacter::applyMovementForces(Vector3 movement, float speed) {
  if (!characterBody) return;

  btVector3 currentVelocity = characterBody->getLinearVelocity();

  // INSTANT STOP when no input - NO MORE SLIDING!
  if (Vector3Length(movement) < 0.01f) {
    // STOP IMMEDIATELY - set horizontal velocity to zero
    btVector3 stopVelocity(0.0f, currentVelocity.getY(), 0.0f);
    characterBody->setLinearVelocity(stopVelocity);
    return;
  }

  // Better movement with collision sliding - preserve Y velocity for jumping
  btVector3 targetVelocity(movement.x * speed, currentVelocity.getY(),
                           movement.z * speed);

  // Apply movement force instead of setting velocity directly for better
  // collision response
  btVector3 velocityDiff = targetVelocity - currentVelocity;
  velocityDiff.setY(0);  // Don't interfere with jumping/gravity

  // Apply movement impulse for better collision handling and sliding
  btVector3 movementImpulse =
      velocityDiff * GameSettings::Character::MASS * 0.1f;
  characterBody->applyCentralImpulse(movementImpulse);

  // Limit maximum horizontal speed to prevent sliding
  btVector3 vel = characterBody->getLinearVelocity();
  float horizontalSpeed =
      sqrt(vel.getX() * vel.getX() + vel.getZ() * vel.getZ());
  if (horizontalSpeed > speed) {
    float scale = speed / horizontalSpeed;
    characterBody->setLinearVelocity(
        btVector3(vel.getX() * scale, vel.getY(), vel.getZ() * scale));
  }

  // Smooth rotation towards movement direction - PHYSICS-FRIENDLY approach
  if (Vector3Length(movement) > 0.1f) {
    float targetRotation = atan2(movement.x, movement.z);

    // Get current rotation
    btTransform transform;
    characterBody->getMotionState()->getWorldTransform(transform);
    btQuaternion currentRotation = transform.getRotation();

    // Extract current Y rotation
    float currentYRotation =
        atan2(2.0f * (currentRotation.getW() * currentRotation.getY() +
                      currentRotation.getX() * currentRotation.getZ()),
              1.0f - 2.0f * (currentRotation.getY() * currentRotation.getY() +
                             currentRotation.getZ() * currentRotation.getZ()));

    float angleDiff = targetRotation - currentYRotation;

    while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2 * M_PI;

    // Fast but gradual rotation - smooth and controlled
    float rotationSpeed = GameSettings::Character::TURN_SPEED *
                          0.05f;  // Smoother rotation to prevent stuttering

    // Apply rotation as angular velocity instead of direct transform - smoother
    // physics
    if (abs(angleDiff) >
        0.01f) {  // Only apply if there's meaningful difference
      float targetAngularVelocity =
          angleDiff * rotationSpeed * 60.0f;  // Scale for smooth rotation
      characterBody->setAngularVelocity(btVector3(0, targetAngularVelocity, 0));
    } else {
      // Close enough - stop rotation
      characterBody->setAngularVelocity(btVector3(0, 0, 0));
    }
  } else {
    // No movement - stop rotation
    characterBody->setAngularVelocity(btVector3(0, 0, 0));
  }
}

void HumanoidCharacter::moveTowards(Vector3 target, float deltaTime) {
  // Calculate direction to target
  Vector3 direction = Vector3Subtract(target, position);
  direction.y = 0;  // Only move on horizontal plane

  float distance = Vector3Length(direction);

  if (distance < 0.5f) {
    applyMovementForces({0, 0, 0}, 0);
    return;
  }

  // Normalize direction and apply movement
  direction = Vector3Normalize(direction);
  float speed = GameSettings::Character::MOVEMENT_SPEED;

  applyMovementForces(direction, speed);
}

void HumanoidCharacter::jump() {
  if (!characterBody) {
    return;
  }

  if (jumpCooldown > 0.0f) {
    return;
  }

  // Check if character is on ground - balanced check
  bool onGround = isOnGround();

  if (!onGround) {
    return;  // Cannot jump if not on ground
  }

  // Additional check: Don't allow jumping if moving up too fast (already
  // jumping)
  btVector3 velocity = characterBody->getLinearVelocity();

  if (velocity.getY() > 2.0f) {
    return;  // Already moving up fast, don't allow another jump
  }

  // Jump force
  btVector3 jumpImpulse(0, GameSettings::Character::JUMP_IMPULSE * 2.0f, 0);
  characterBody->applyCentralImpulse(jumpImpulse);

  isJumping = true;
  jumpCooldown =
      0.2f;  // Moderate cooldown to prevent rapid spam but allow normal jumping
}

bool HumanoidCharacter::isOnGround() const {
  if (!characterBody || !physicsWorld) {
    return false;
  }

  // Raycast downward to check for ground
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  // Ground detection using raycast
  float groundContactOffset =
      GameSettings::CharacterCalculations::getGroundContactOffset();
  btVector3 from =
      origin + btVector3(0, groundContactOffset + 0.05f,
                         0);  // Start slightly above ground contact
  btVector3 to = origin + btVector3(0, groundContactOffset - 0.25f,
                                    0);  // Check below ground contact

  btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
  // Use the proper collision groups for raycast
  rayCallback.m_collisionFilterGroup =
      GameSettings::Collision::Groups::CHARACTER_TORSO;
  rayCallback.m_collisionFilterMask =
      GameSettings::Collision::Groups::WORLD_OBJECTS;  // Only hit world objects

  physicsWorld->rayTest(from, to, rayCallback);

  // Balanced velocity check - allow for small physics fluctuations but prevent
  // air jumps
  btVector3 velocity = characterBody->getLinearVelocity();
  bool velocityCheck =
      velocity.getY() < 1.0f && velocity.getY() > -1.0f;  // More reasonable

  bool hasHit = rayCallback.hasHit();
  bool result = hasHit && velocityCheck;

  return result;
}

Vector3 HumanoidCharacter::getFeetPosition() const {
  if (!characterBody) return position;

  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  // Return feet position (ground contact point)
  return {origin.getX(),
          GameSettings::CharacterCalculations::getGroundYFromPhysicsCenterY(
              origin.getY()),
          origin.getZ()};
}

Vector3 HumanoidCharacter::getTorsoPosition() const {
  if (!characterBody) return position;

  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btVector3 origin = transform.getOrigin();

  return {origin.getX(), origin.getY(), origin.getZ()};
}

void HumanoidCharacter::draw() const {
  // Draw all visual parts
  head.visual.draw();
  torso.visual.draw();
  leftArm.visual.draw();
  rightArm.visual.draw();
  leftLeg.visual.draw();
  rightLeg.visual.draw();

  // Draw facial features for character expression
  leftEye.visual.draw();
  rightEye.visual.draw();
  mouth.visual.draw();
  leftEyebrow.visual.draw();
  rightEyebrow.visual.draw();
}

void HumanoidCharacter::drawCollisionBoxes() const {
  // Draw wireframe boxes around each body part for debugging
  BoundingBox headBox = getPartBoundingBox(head);
  BoundingBox torsoBox = getPartBoundingBox(torso);
  BoundingBox leftArmBox = getPartBoundingBox(leftArm);
  BoundingBox rightArmBox = getPartBoundingBox(rightArm);
  BoundingBox leftLegBox = getPartBoundingBox(leftLeg);
  BoundingBox rightLegBox = getPartBoundingBox(rightLeg);

  // Draw collision boxes in different colors per collision group
  DrawBoundingBox(headBox, PURPLE);      // Head group
  DrawBoundingBox(torsoBox, BLUE);       // Torso group
  DrawBoundingBox(leftArmBox, GREEN);    // Arms group
  DrawBoundingBox(rightArmBox, GREEN);   // Arms group
  DrawBoundingBox(leftLegBox, ORANGE);   // Legs group
  DrawBoundingBox(rightLegBox, ORANGE);  // Legs group

  // Draw physics body positions as small spheres if they exist
  if (headPhysics.body) {
    btTransform transform = headPhysics.body->getWorldTransform();
    btVector3 origin = transform.getOrigin();
    DrawSphere({origin.x(), origin.y(), origin.z()}, 0.05f, PURPLE);
  }
  if (torsoPhysics.body) {
    btTransform transform = torsoPhysics.body->getWorldTransform();
    btVector3 origin = transform.getOrigin();
    DrawSphere({origin.x(), origin.y(), origin.z()}, 0.05f, BLUE);
  }
  if (leftLegPhysics.body) {
    btTransform transform = leftLegPhysics.body->getWorldTransform();
    btVector3 origin = transform.getOrigin();
    DrawSphere({origin.x(), origin.y(), origin.z()}, 0.05f, ORANGE);
  }
  if (rightLegPhysics.body) {
    btTransform transform = rightLegPhysics.body->getWorldTransform();
    btVector3 origin = transform.getOrigin();
    DrawSphere({origin.x(), origin.y(), origin.z()}, 0.05f, ORANGE);
  }
}

BoundingBox HumanoidCharacter::getBoundingBox() const {
  Vector3 torsoPos = getTorsoPosition();
  Vector3 min = {torsoPos.x - 0.5f, torsoPos.y - 1.0f, torsoPos.z - 0.5f};
  Vector3 max = {torsoPos.x + 0.5f, torsoPos.y + 1.0f, torsoPos.z + 0.5f};
  return {min, max};
}

// Individual body part collision detection methods
Vector3 HumanoidCharacter::getPartWorldPosition(
    const HumanoidVisualPart& part) const {
  Vector3 characterPos = getPosition();

  // Get character rotation (yaw only for simplicity)
  btTransform transform;
  characterBody->getMotionState()->getWorldTransform(transform);
  btQuaternion rotation = transform.getRotation();
  float yaw = rotation.getAngle() * rotation.getAxis().getY();

  // Apply rotation to the part's current offset
  float cosYaw = cos(yaw);
  float sinYaw = sin(yaw);

  Vector3 rotatedOffset = {
      part.currentOffset.x * cosYaw - part.currentOffset.z * sinYaw,
      part.currentOffset.y,
      part.currentOffset.x * sinYaw + part.currentOffset.z * cosYaw};

  return Vector3Add(characterPos, rotatedOffset);
}

BoundingBox HumanoidCharacter::getPartBoundingBox(
    const HumanoidVisualPart& part) const {
  Vector3 partPos = getPartWorldPosition(part);
  Vector3 halfSize = Vector3Scale(part.visual.getSize(), 0.5f);

  Vector3 min = Vector3Subtract(partPos, halfSize);
  Vector3 max = Vector3Add(partPos, halfSize);

  return {min, max};
}

bool HumanoidCharacter::checkPartCollision(const HumanoidVisualPart& part,
                                           const BoundingBox& obstacle) const {
  BoundingBox partBox = getPartBoundingBox(part);
  return CheckCollisionBoxes(partBox, obstacle);
}

bool HumanoidCharacter::checkAnyPartCollision(
    const BoundingBox& obstacle) const {
  // Check all major body parts for collision
  if (checkPartCollision(head, obstacle)) return true;
  if (checkPartCollision(torso, obstacle)) return true;
  if (checkPartCollision(leftArm, obstacle)) return true;
  if (checkPartCollision(rightArm, obstacle)) return true;
  if (checkPartCollision(leftLeg, obstacle)) return true;
  if (checkPartCollision(rightLeg, obstacle)) return true;

  return false;
}

bool HumanoidCharacter::wouldCollideAfterMovement(Vector3 movement,
                                                  float deltaTime) const {
  if (!world) return false;  // No world to check against

  // Calculate predicted position after movement
  btVector3 currentVelocity = characterBody->getLinearVelocity();
  Vector3 predictedMovement = {movement.x * deltaTime,
                               0,  // Don't predict Y movement for simplicity
                               movement.z * deltaTime};

  // Temporarily store current character position
  Vector3 originalPos = getPosition();

  // Create temporary character parts at predicted positions
  HumanoidVisualPart tempHead = head;
  HumanoidVisualPart tempTorso = torso;
  HumanoidVisualPart tempLeftArm = leftArm;
  HumanoidVisualPart tempRightArm = rightArm;
  HumanoidVisualPart tempLeftLeg = leftLeg;
  HumanoidVisualPart tempRightLeg = rightLeg;

  // Offset the parts by predicted movement
  tempHead.currentOffset =
      Vector3Add(tempHead.currentOffset, predictedMovement);
  tempTorso.currentOffset =
      Vector3Add(tempTorso.currentOffset, predictedMovement);
  tempLeftArm.currentOffset =
      Vector3Add(tempLeftArm.currentOffset, predictedMovement);
  tempRightArm.currentOffset =
      Vector3Add(tempRightArm.currentOffset, predictedMovement);
  tempLeftLeg.currentOffset =
      Vector3Add(tempLeftLeg.currentOffset, predictedMovement);
  tempRightLeg.currentOffset =
      Vector3Add(tempRightLeg.currentOffset, predictedMovement);

  // Check predicted positions against world obstacles
  Vector3 predictedPosition = Vector3Add(originalPos, predictedMovement);

  // Create bounding boxes for predicted character parts
  Vector3 headPos = Vector3Add(predictedPosition, tempHead.currentOffset);
  Vector3 torsoPos = Vector3Add(predictedPosition, tempTorso.currentOffset);
  Vector3 leftArmPos = Vector3Add(predictedPosition, tempLeftArm.currentOffset);
  Vector3 rightArmPos =
      Vector3Add(predictedPosition, tempRightArm.currentOffset);
  Vector3 leftLegPos = Vector3Add(predictedPosition, tempLeftLeg.currentOffset);
  Vector3 rightLegPos =
      Vector3Add(predictedPosition, tempRightLeg.currentOffset);

  // Get bounding boxes for each part at predicted positions
  BoundingBox predictedHeadBox = {
      {headPos.x - tempHead.visual.getSize().x / 2,
       headPos.y - tempHead.visual.getSize().y / 2,
       headPos.z - tempHead.visual.getSize().z / 2},
      {headPos.x + tempHead.visual.getSize().x / 2,
       headPos.y + tempHead.visual.getSize().y / 2,
       headPos.z + tempHead.visual.getSize().z / 2}};

  BoundingBox predictedTorsoBox = {
      {torsoPos.x - tempTorso.visual.getSize().x / 2,
       torsoPos.y - tempTorso.visual.getSize().y / 2,
       torsoPos.z - tempTorso.visual.getSize().z / 2},
      {torsoPos.x + tempTorso.visual.getSize().x / 2,
       torsoPos.y + tempTorso.visual.getSize().y / 2,
       torsoPos.z + tempTorso.visual.getSize().z / 2}};

  // Check against world objects (simple implementation)
  // This is a basic collision prediction - in a full implementation,
  // you would iterate through all world objects and check collisions
  const auto& objects = world->getObjects();
  for (const auto& obj : objects) {
    if (obj.get() == this) continue;  // Skip self

    BoundingBox objBox = obj->getBoundingBox();

    // Check if any predicted part would collide
    if (CheckCollisionBoxes(predictedHeadBox, objBox) ||
        CheckCollisionBoxes(predictedTorsoBox, objBox)) {
      return true;
    }
  }

  return false;
}

// Multi-body physics system implementation
void HumanoidCharacter::createIndividualPhysicsBodies() {
  if (!physicsWorld || !characterBody) return;

  // Get character position for relative positioning
  btTransform characterTransform = characterBody->getWorldTransform();
  btVector3 characterPos = characterTransform.getOrigin();

  // Create physics bodies for each body part
  createIndividualPhysicsBody(
      headPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      torsoPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      leftArmPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      rightArmPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      leftLegPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
  createIndividualPhysicsBody(
      rightLegPhysics, {characterPos.x(), characterPos.y(), characterPos.z()});
}

void HumanoidCharacter::createIndividualPhysicsBody(HumanoidPhysicsPart& part,
                                                    Vector3 worldPosition) {
  if (!physicsWorld) return;

  // Create box shape for the body part
  btVector3 halfExtents(part.size.x / 2.0f, part.size.y / 2.0f,
                        part.size.z / 2.0f);
  part.shape = new btBoxShape(halfExtents);
  part.shape->setMargin(GameSettings::Collision::SHAPE_MARGIN);

  // Calculate world position for this part
  Vector3 partWorldPos = Vector3Add(worldPosition, part.baseOffset);

  // Create transform
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(partWorldPos.x, partWorldPos.y, partWorldPos.z));

  // Create motion state
  part.motionState = new btDefaultMotionState(transform);

  // Create rigid body with very low mass (kinematic-like behavior)
  float mass = 0.1f;  // Very light so they don't interfere with main movement
  btVector3 inertia(0, 0, 0);
  part.shape->calculateLocalInertia(mass, inertia);

  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, part.motionState,
                                                  part.shape, inertia);
  rbInfo.m_friction = GameSettings::Collision::FRICTION;
  rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
  rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;

  part.body = new btRigidBody(rbInfo);

  // Set as kinematic for controlled movement
  part.body->setCollisionFlags(part.body->getCollisionFlags() |
                               btCollisionObject::CF_KINEMATIC_OBJECT);
  part.body->setActivationState(DISABLE_DEACTIVATION);

  // Add to physics world with proper collision filtering
  // Individual body parts should collide with world objects only
  physicsWorld->addRigidBody(part.body, part.collisionGroup,
                             GameSettings::Collision::Groups::WORLD_OBJECTS);
}

void HumanoidCharacter::updatePhysicsBodyPosition(HumanoidPhysicsPart& part,
                                                  Vector3 worldPosition) {
  if (!part.body) return;

  // Calculate new world position for this part
  Vector3 partWorldPos = Vector3Add(worldPosition, part.baseOffset);

  // Get character rotation to apply to physics bodies too
  btTransform characterTransform;
  characterBody->getMotionState()->getWorldTransform(characterTransform);
  btQuaternion characterRotation = characterTransform.getRotation();

  // Update the physics body position AND rotation
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(partWorldPos.x, partWorldPos.y, partWorldPos.z));
  transform.setRotation(characterRotation);  // Apply same rotation as character

  part.body->setWorldTransform(transform);
  part.body->getMotionState()->setWorldTransform(transform);
}

void HumanoidCharacter::synchronizeVisualWithPhysics() {
  if (!characterBody) return;

  // Get main character position
  btTransform characterTransform = characterBody->getWorldTransform();
  btVector3 characterPos = characterTransform.getOrigin();
  Vector3 worldPos = {characterPos.x(), characterPos.y(), characterPos.z()};

  // Update each physics body to follow the visual parts
  updatePhysicsBodyPosition(headPhysics, worldPos);
  updatePhysicsBodyPosition(torsoPhysics, worldPos);
  updatePhysicsBodyPosition(leftArmPhysics, worldPos);
  updatePhysicsBodyPosition(rightArmPhysics, worldPos);
  updatePhysicsBodyPosition(leftLegPhysics, worldPos);
  updatePhysicsBodyPosition(rightLegPhysics, worldPos);
}

void HumanoidCharacter::removeAllPhysicsBodies() {
  if (!physicsWorld) return;

  // Remove all individual physics bodies from the world
  if (headPhysics.body) {
    physicsWorld->removeRigidBody(headPhysics.body);
    headPhysics.cleanup();
  }
  if (torsoPhysics.body) {
    physicsWorld->removeRigidBody(torsoPhysics.body);
    torsoPhysics.cleanup();
  }
  if (leftArmPhysics.body) {
    physicsWorld->removeRigidBody(leftArmPhysics.body);
    leftArmPhysics.cleanup();
  }
  if (rightArmPhysics.body) {
    physicsWorld->removeRigidBody(rightArmPhysics.body);
    rightArmPhysics.cleanup();
  }
  if (leftLegPhysics.body) {
    physicsWorld->removeRigidBody(leftLegPhysics.body);
    leftLegPhysics.cleanup();
  }
  if (rightLegPhysics.body) {
    physicsWorld->removeRigidBody(rightLegPhysics.body);
    rightLegPhysics.cleanup();
  }
}

void HumanoidCharacter::constrainBodyPartsToCharacter() {
  // This method can be used to add constraints between body parts
  // For now, we use kinematic bodies that follow the main character
  // In future implementations, you could add spring constraints
  // or other joint types to create more physics interactions
}

std::unique_ptr<GameObject> HumanoidCharacter::clone() const {
  return std::make_unique<HumanoidCharacter>(*this);
}

// HumanoidCharacter.cpp - Solid humanoid character controller (no ragdoll)
#include "entities/HumanoidCharacter.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raylib.h"
#include "raymath.h"
#include <cmath>

HumanoidCharacter::HumanoidCharacter(Vector3 position)
    : GameObject(position, true, true, false),
      // Roblox-style proportions - more blocky and separated
      head({0.6f, 0.6f, 0.6f}, YELLOW, {0, 0.8f, 0}),  // Bigger cubic head
      torso({0.8f, 1.0f, 0.5f}, BLUE, {0, 0.0f, 0}),   // Wider, taller torso  
      leftArm({0.3f, 0.8f, 0.3f}, YELLOW, {-0.6f, 0.2f, 0}),  // Thicker arms, more separated
      rightArm({0.3f, 0.8f, 0.3f}, YELLOW, {0.6f, 0.2f, 0}),   // Thicker arms, more separated
      leftLeg({0.35f, 0.8f, 0.35f}, DARKBLUE, {-0.25f, -0.85f, 0}),  // Thicker legs, better separated
      rightLeg({0.35f, 0.8f, 0.35f}, DARKBLUE, {0.25f, -0.85f, 0}),   // Thicker legs, better separated
      // Facial features - positioned on front of head with better proportions
      leftEye({0.1f, 0.1f, 0.1f}, BLACK, {-0.15f, 0.9f, 0.3f}),  // Bigger eyes
      rightEye({0.1f, 0.1f, 0.1f}, BLACK, {0.15f, 0.9f, 0.3f}),   // Bigger eyes  
      mouth({0.2f, 0.05f, 0.05f}, DARKGRAY, {0, 0.75f, 0.3f}) {   // Bigger mouth
}

HumanoidCharacter::~HumanoidCharacter() {
    if (physicsWorld && characterBody) {
        physicsWorld->removeRigidBody(characterBody);
    }
    
    delete characterBody;
    delete characterShape;
    delete motionState;
}

void HumanoidCharacter::createSinglePhysicsBody() {
    // Use realistic dimensions from settings
    float radius = GameSettings::Character::RADIUS;
    float height = GameSettings::Character::HEIGHT;
    
    characterShape = new btCapsuleShape(radius, height);
    characterShape->setMargin(GameSettings::Collision::SHAPE_MARGIN);
    
    // Set initial transform - place character so feet are on ground
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(position.x, position.y + height/2, position.z));
    
    // Create motion state
    motionState = new btDefaultMotionState(transform);
    
    // Use realistic mass from settings
    float mass = GameSettings::Character::MASS;
    btVector3 inertia(0, 0, 0);
    characterShape->calculateLocalInertia(mass, inertia);
    
    // Create rigid body with optimized settings
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, characterShape, inertia);
    rbInfo.m_friction = GameSettings::Collision::FRICTION;
    rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
    rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;
    
    characterBody = new btRigidBody(rbInfo);
    
    // Use settings for damping
    characterBody->setDamping(GameSettings::Character::DAMPING_LINEAR, GameSettings::Character::DAMPING_ANGULAR);
    characterBody->setActivationState(DISABLE_DEACTIVATION);
    characterBody->setSleepingThresholds(0.0f, 0.0f);
    
    // Lock rotation around X and Z axes to prevent tipping
    characterBody->setAngularFactor(btVector3(0, 1, 0));
    
    // Set gravity
    characterBody->setGravity(btVector3(0, GameSettings::Physics::GRAVITY_ACCELERATION, 0));
}

void HumanoidCharacter::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    physicsWorld = bulletWorld;
    createSinglePhysicsBody();
    
    // Add only the main physics body to the world
    physicsWorld->addRigidBody(characterBody);
}

void HumanoidCharacter::removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    if (bulletWorld && characterBody) {
        bulletWorld->removeRigidBody(characterBody);
    }
}

void HumanoidCharacter::update(float deltaTime) {
    if (!characterBody) return;
    
    // Update jump cooldown
    if (jumpCooldown > 0.0f) {
        jumpCooldown -= deltaTime;
    }
    
    // Update character state based on movement
    updateCharacterState();
    
    // Animate the character parts
    animateCharacter(deltaTime);
    
    // Update visual positions and facial features (combined for efficiency)
    updateVisualPositions();
    
    // Facial features are now updated inside updateVisualPositions()
}

void HumanoidCharacter::updateCharacterState() {
    Vector3 velocity = {0, 0, 0};
    if (characterBody) {
        btVector3 btVel = characterBody->getLinearVelocity();
        velocity = {btVel.getX(), btVel.getY(), btVel.getZ()};
    }
    
    // Reset jumping state when landed with more lenient conditions
    if (isJumping && isOnGround() && velocity.y < 0.2f && velocity.y > -0.2f) {
        isJumping = false;
    }
    
    // Determine character state
    float horizontalSpeed = sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
    
    if (isJumping || !isOnGround() || abs(velocity.y) > 0.3f) {  // Less strict jumping detection
        currentState = JUMPING;
    } else if (horizontalSpeed > 0.15f) {  // Lower threshold for walking
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
            head.currentOffset = Vector3Add(head.baseOffset, {0, sin(animationTime * 2.0f) * 0.01f, 0});
            torso.currentOffset = torso.baseOffset;
            leftArm.currentOffset = leftArm.baseOffset;
            rightArm.currentOffset = rightArm.baseOffset;
            leftLeg.currentOffset = leftLeg.baseOffset;
            rightLeg.currentOffset = rightLeg.baseOffset;
            break;
            
        case WALKING:
            {
                // Much more pronounced walking animation with realistic limb movement
                float walkCycle = animationTime * 8.0f;  // Walking cycle frequency
                float armSwingL = sin(walkCycle) * 0.4f;  // Left arm swing
                float armSwingR = sin(walkCycle + M_PI) * 0.4f;  // Right arm swing (opposite)
                float legSwingL = sin(walkCycle) * 0.3f;  // Left leg swing
                float legSwingR = sin(walkCycle + M_PI) * 0.3f;  // Right leg swing (opposite)
                
                // Head bobbing - subtle but visible
                head.currentOffset = Vector3Add(head.baseOffset, {0, sin(walkCycle * 2.0f) * 0.05f, 0});
                
                // Torso slight movement for natural walking
                torso.currentOffset = Vector3Add(torso.baseOffset, {sin(walkCycle) * 0.02f, sin(walkCycle * 2.0f) * 0.03f, 0});
                
                // Arms swing naturally - forward/back movement with slight vertical
                leftArm.currentOffset = Vector3Add(leftArm.baseOffset, {0, armSwingL * 0.3f, armSwingL * 0.6f});
                rightArm.currentOffset = Vector3Add(rightArm.baseOffset, {0, armSwingR * 0.3f, armSwingR * 0.6f});
                
                // Legs with natural walking motion - lifting and moving forward/back
                float leftLegLift = fmax(0, sin(walkCycle)) * 0.15f;  // Only lift when moving forward
                float rightLegLift = fmax(0, sin(walkCycle + M_PI)) * 0.15f;  // Only lift when moving forward
                
                leftLeg.currentOffset = Vector3Add(leftLeg.baseOffset, {legSwingL * 0.1f, leftLegLift, legSwingL * 0.2f});
                rightLeg.currentOffset = Vector3Add(rightLeg.baseOffset, {legSwingR * 0.1f, rightLegLift, legSwingR * 0.2f});
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
    
    // Update main position ONCE per frame (at feet level) - SINGLE UPDATE
    position = {origin.getX(), origin.getY() - GameSettings::Character::HEIGHT/2, origin.getZ()};
    
    // Get rotation (only Y-axis rotation is allowed)
    btQuaternion rotation = transform.getRotation();
    float yRotation = atan2(2.0f * (rotation.getW() * rotation.getY() + rotation.getX() * rotation.getZ()),
                           1.0f - 2.0f * (rotation.getY() * rotation.getY() + rotation.getZ() * rotation.getZ()));
    
    // Update each visual part position relative to physics body
    Vector3 rotatedOffset;
    
    // Head
    rotatedOffset = Vector3RotateByAxisAngle(head.currentOffset, {0, 1, 0}, yRotation);
    head.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Torso
    rotatedOffset = Vector3RotateByAxisAngle(torso.currentOffset, {0, 1, 0}, yRotation);
    torso.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Left Arm
    rotatedOffset = Vector3RotateByAxisAngle(leftArm.currentOffset, {0, 1, 0}, yRotation);
    leftArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Right Arm
    rotatedOffset = Vector3RotateByAxisAngle(rightArm.currentOffset, {0, 1, 0}, yRotation);
    rightArm.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Left Leg
    rotatedOffset = Vector3RotateByAxisAngle(leftLeg.currentOffset, {0, 1, 0}, yRotation);
    leftLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Right Leg
    rotatedOffset = Vector3RotateByAxisAngle(rightLeg.currentOffset, {0, 1, 0}, yRotation);
    rightLeg.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Update facial features using the SAME transform (no redundant calls)
    // Left Eye
    rotatedOffset = Vector3RotateByAxisAngle(leftEye.currentOffset, {0, 1, 0}, yRotation);
    leftEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Right Eye
    rotatedOffset = Vector3RotateByAxisAngle(rightEye.currentOffset, {0, 1, 0}, yRotation);
    rightEye.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
    
    // Mouth
    rotatedOffset = Vector3RotateByAxisAngle(mouth.currentOffset, {0, 1, 0}, yRotation);
    mouth.visual.setPosition(Vector3Add(physicsPos, rotatedOffset));
}

// updateFacialFeatures() method removed - now integrated into updateVisualPositions() for efficiency

void HumanoidCharacter::handleInput(float movementSpeed) {
    if (!characterBody) return;
    
    Vector3 movement = {0, 0, 0};
    
    // Get input
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) {
        movement.z -= 1.0f;
    }
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) {
        movement.z += 1.0f;
    }
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) {
        movement.x -= 1.0f;
    }
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) {
        movement.x += 1.0f;
    }
    
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
    
    // Get current velocity
    btVector3 currentVelocity = characterBody->getLinearVelocity();
    
    // INSTANT STOP when no input - NO MORE SLIDING!
    if (Vector3Length(movement) < 0.01f) {
        // STOP IMMEDIATELY - set horizontal velocity to zero
        btVector3 stopVelocity(0.0f, currentVelocity.getY(), 0.0f);
        characterBody->setLinearVelocity(stopVelocity);
        return;
    }
    
    // When moving, set target velocity directly for responsive movement
    btVector3 targetVelocity(movement.x * speed, currentVelocity.getY(), movement.z * speed);
    characterBody->setLinearVelocity(targetVelocity);
    
    // Smooth rotation towards movement direction - NO OSCILLATION
    if (Vector3Length(movement) > 0.1f) {
        float targetRotation = atan2(movement.x, movement.z);
        
        // Get current rotation
        btTransform transform;
        characterBody->getMotionState()->getWorldTransform(transform);
        btQuaternion currentRotation = transform.getRotation();
        
        // Extract current Y rotation
        float currentYRotation = atan2(2.0f * (currentRotation.getW() * currentRotation.getY() + currentRotation.getX() * currentRotation.getZ()),
                                     1.0f - 2.0f * (currentRotation.getY() * currentRotation.getY() + currentRotation.getZ() * currentRotation.getZ()));
        
        // Calculate rotation difference
        float angleDiff = targetRotation - currentYRotation;
        
        // Handle angle wrapping
        while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
        while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
        
        // Smooth interpolation to target rotation - NO OVERSHOOTING
        float rotationSpeed = GameSettings::Character::TURN_SPEED * 0.016f; // Fixed timestep ~60fps
        float newYRotation;
        
        if (abs(angleDiff) < rotationSpeed) {
            // Close enough - snap to target to prevent oscillation
            newYRotation = targetRotation;
        } else {
            // Smooth rotation towards target
            newYRotation = currentYRotation + (angleDiff > 0 ? rotationSpeed : -rotationSpeed);
        }
        
        // Apply the new rotation directly (no torque - prevents oscillation)
        btQuaternion newRotation;
        newRotation.setEulerZYX(0, newYRotation, 0);
        transform.setRotation(newRotation);
        characterBody->getMotionState()->setWorldTransform(transform);
    }
}

void HumanoidCharacter::jump() {
    if (!characterBody || jumpCooldown > 0.0f || !isOnGround()) return;  // Only jump when on ground
    
    // Realistic jump force - reduced from 3x to 2x for more realistic jumping
    btVector3 jumpImpulse(0, GameSettings::Character::JUMP_IMPULSE * 2.0f, 0);  // 2x for realistic jumping
    characterBody->applyCentralImpulse(jumpImpulse);
    
    isJumping = true;
    jumpCooldown = 0.1f;  // Very responsive jumping
}

bool HumanoidCharacter::isOnGround() const {
    if (!characterBody || !physicsWorld) return false;
    
    // Raycast downward to check for ground
    btTransform transform;
    characterBody->getMotionState()->getWorldTransform(transform);
    btVector3 origin = transform.getOrigin();
    
    // Cast from bottom of capsule down slightly - more generous detection
    float halfHeight = GameSettings::Character::HEIGHT / 2;
    btVector3 from = origin + btVector3(0, -halfHeight + 0.05f, 0); // Start closer to bottom
    btVector3 to = origin + btVector3(0, -halfHeight - 0.3f, 0); // Check further down
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
    rayCallback.m_collisionFilterGroup = 1;
    rayCallback.m_collisionFilterMask = -1;
    
    physicsWorld->rayTest(from, to, rayCallback);
    
    // More lenient velocity check for jumping - allow jumping even with some vertical movement
    btVector3 velocity = characterBody->getLinearVelocity();
    bool velocityCheck = velocity.getY() < 2.0f && velocity.getY() > -2.0f;  // More lenient
    
    return rayCallback.hasHit() && velocityCheck;
}

Vector3 HumanoidCharacter::getFeetPosition() const {
    if (!characterBody) return position;
    
    btTransform transform;
    characterBody->getMotionState()->getWorldTransform(transform);
    btVector3 origin = transform.getOrigin();
    
    // Return feet position (bottom of capsule)
    float halfHeight = GameSettings::Character::HEIGHT / 2;
    return {origin.getX(), origin.getY() - halfHeight, origin.getZ()};
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
    
    // Draw facial features
    leftEye.visual.draw();
    rightEye.visual.draw();
    mouth.visual.draw();
}

void HumanoidCharacter::drawCollisionBoxes() const {
    // Draw wireframe boxes around each body part for debugging
    BoundingBox headBox = getPartBoundingBox(head);
    BoundingBox torsoBox = getPartBoundingBox(torso);
    BoundingBox leftArmBox = getPartBoundingBox(leftArm);
    BoundingBox rightArmBox = getPartBoundingBox(rightArm);
    BoundingBox leftLegBox = getPartBoundingBox(leftLeg);
    BoundingBox rightLegBox = getPartBoundingBox(rightLeg);
    
    // Draw collision boxes in red wireframe
    DrawBoundingBox(headBox, RED);
    DrawBoundingBox(torsoBox, RED);
    DrawBoundingBox(leftArmBox, RED);
    DrawBoundingBox(rightArmBox, RED);
    DrawBoundingBox(leftLegBox, RED);
    DrawBoundingBox(rightLegBox, RED);
}

BoundingBox HumanoidCharacter::getBoundingBox() const {
    Vector3 torsoPos = getTorsoPosition();
    Vector3 min = {torsoPos.x - 0.5f, torsoPos.y - 1.0f, torsoPos.z - 0.5f};
    Vector3 max = {torsoPos.x + 0.5f, torsoPos.y + 1.0f, torsoPos.z + 0.5f};
    return {min, max};
}

// Individual body part collision detection methods
Vector3 HumanoidCharacter::getPartWorldPosition(const HumanoidVisualPart& part) const {
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
        part.currentOffset.x * sinYaw + part.currentOffset.z * cosYaw
    };
    
    return Vector3Add(characterPos, rotatedOffset);
}

BoundingBox HumanoidCharacter::getPartBoundingBox(const HumanoidVisualPart& part) const {
    Vector3 partPos = getPartWorldPosition(part);
    Vector3 halfSize = Vector3Scale(part.visual.getSize(), 0.5f);
    
    Vector3 min = Vector3Subtract(partPos, halfSize);
    Vector3 max = Vector3Add(partPos, halfSize);
    
    return {min, max};
}

bool HumanoidCharacter::checkPartCollision(const HumanoidVisualPart& part, const BoundingBox& obstacle) const {
    BoundingBox partBox = getPartBoundingBox(part);
    return CheckCollisionBoxes(partBox, obstacle);
}

bool HumanoidCharacter::checkAnyPartCollision(const BoundingBox& obstacle) const {
    // Check all major body parts for collision
    if (checkPartCollision(head, obstacle)) return true;
    if (checkPartCollision(torso, obstacle)) return true;
    if (checkPartCollision(leftArm, obstacle)) return true;
    if (checkPartCollision(rightArm, obstacle)) return true;
    if (checkPartCollision(leftLeg, obstacle)) return true;
    if (checkPartCollision(rightLeg, obstacle)) return true;
    
    return false;
}

bool HumanoidCharacter::wouldCollideAfterMovement(Vector3 movement, float deltaTime) const {
    if (!world) return false; // No world to check against
    
    // Calculate predicted position after movement
    btVector3 currentVelocity = characterBody->getLinearVelocity();
    Vector3 predictedMovement = {
        movement.x * deltaTime,
        0, // Don't predict Y movement for simplicity
        movement.z * deltaTime
    };
    
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
    tempHead.currentOffset = Vector3Add(tempHead.currentOffset, predictedMovement);
    tempTorso.currentOffset = Vector3Add(tempTorso.currentOffset, predictedMovement);
    tempLeftArm.currentOffset = Vector3Add(tempLeftArm.currentOffset, predictedMovement);
    tempRightArm.currentOffset = Vector3Add(tempRightArm.currentOffset, predictedMovement);
    tempLeftLeg.currentOffset = Vector3Add(tempLeftLeg.currentOffset, predictedMovement);
    tempRightLeg.currentOffset = Vector3Add(tempRightLeg.currentOffset, predictedMovement);
    
    // TODO: Check against world obstacles
    // This would require access to world collision objects
    // For now, return false (no collision predicted)
    
    return false;
}



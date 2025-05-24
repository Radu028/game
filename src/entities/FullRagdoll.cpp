// FullRagdoll.cpp - Professional ragdoll implementation with individual physics bodies
#include "entities/FullRagdoll.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raymath.h"
#include <cmath>

FullRagdoll::FullRagdoll(Vector3 position)
    : GameObject(position, true, true, false),
      head(GameSettings::BodyParts::HEAD_SIZE, GameSettings::BodyParts::HEAD_COLOR),
      torso(GameSettings::BodyParts::TORSO_SIZE, GameSettings::BodyParts::TORSO_COLOR),
      leftArm(GameSettings::BodyParts::ARM_SIZE, GameSettings::BodyParts::ARM_COLOR),
      rightArm(GameSettings::BodyParts::ARM_SIZE, GameSettings::BodyParts::ARM_COLOR),
      leftLeg(GameSettings::BodyParts::LEG_SIZE, GameSettings::BodyParts::LEG_COLOR),
      rightLeg(GameSettings::BodyParts::LEG_SIZE, GameSettings::BodyParts::LEG_COLOR) {
}

FullRagdoll::~FullRagdoll() {
}

void FullRagdoll::createRagdollPart(RagdollPart& part, Vector3 position, Vector3 size, 
                                   int collisionGroup, int collidesWith, float mass) {
    // Create box shape for the body part
    part.shape = new btBoxShape(btVector3(size.x * 0.5f, size.y * 0.5f, size.z * 0.5f));
    part.shape->setMargin(GameSettings::Collision::SHAPE_MARGIN);
    
    // Set initial transform
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(position.x, position.y, position.z));
    
    // Create motion state
    part.motionState = new btDefaultMotionState(transform);
    
    // Calculate inertia
    btVector3 inertia(0, 0, 0);
    if (mass > 0.0f) {
        part.shape->calculateLocalInertia(mass, inertia);
    }
    
    // Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, part.motionState, part.shape, inertia);
    rbInfo.m_friction = GameSettings::Collision::FRICTION;
    rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
    rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;
    
    part.body = new btRigidBody(rbInfo);
    part.body->setDamping(GameSettings::Character::DAMPING_LINEAR, GameSettings::Character::DAMPING_ANGULAR);
    part.body->setActivationState(DISABLE_DEACTIVATION);
    
    // Add to physics world with collision filtering
    physicsWorld->addRigidBody(part.body, collisionGroup, collidesWith);
    
    // Set visual position
    part.visual.setPosition(position);
}

void FullRagdoll::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    physicsWorld = bulletWorld;
    
    // Calculate body part positions based on Lego proportions
    Vector3 basePos = this->position;
    
    // Calculate heights for proper stacking
    float legHeight = GameSettings::BodyParts::LEG_SIZE.y;
    float torsoHeight = GameSettings::BodyParts::TORSO_SIZE.y;
    float headHeight = GameSettings::BodyParts::HEAD_SIZE.y;
    float armHeight = GameSettings::BodyParts::ARM_SIZE.y;
    
    // Ground level calculation - feet should touch ground
    float groundLevel = basePos.y;
    
    // Position each body part
    Vector3 legPos = {basePos.x, groundLevel + legHeight * 0.5f, basePos.z};
    Vector3 torsoPos = {basePos.x, groundLevel + legHeight + torsoHeight * 0.5f, basePos.z};
    Vector3 headPos = {basePos.x, groundLevel + legHeight + torsoHeight + headHeight * 0.5f + GameSettings::BodyParts::HEAD_OFFSET_Y, basePos.z};
    Vector3 armPos = {basePos.x, groundLevel + legHeight + torsoHeight * 0.75f, basePos.z}; // Arms at shoulder level
    
    // Create torso (main body - heavier for stability)
    createRagdollPart(torso, torsoPos, GameSettings::BodyParts::TORSO_SIZE, 
                     COL_RAGDOLL_TORSO, TORSO_COLLIDES_WITH, GameSettings::Character::MASS * 0.5f);
    
    // Create head
    Vector3 headPosActual = headPos;
    createRagdollPart(head, headPosActual, GameSettings::BodyParts::HEAD_SIZE, 
                     COL_RAGDOLL_HEAD, HEAD_COLLIDES_WITH, GameSettings::Character::MASS * 0.15f);
    
    // Create arms
    Vector3 leftArmPos = {armPos.x - GameSettings::BodyParts::ARM_OFFSET_X, armPos.y, armPos.z};
    Vector3 rightArmPos = {armPos.x + GameSettings::BodyParts::ARM_OFFSET_X, armPos.y, armPos.z};
    createRagdollPart(leftArm, leftArmPos, GameSettings::BodyParts::ARM_SIZE, 
                     COL_RAGDOLL_ARMS, ARMS_COLLIDE_WITH, GameSettings::Character::MASS * 0.1f);
    createRagdollPart(rightArm, rightArmPos, GameSettings::BodyParts::ARM_SIZE, 
                     COL_RAGDOLL_ARMS, ARMS_COLLIDE_WITH, GameSettings::Character::MASS * 0.1f);
    
    // Create legs
    Vector3 leftLegPos = {legPos.x - GameSettings::BodyParts::LEG_OFFSET_X, legPos.y, legPos.z};
    Vector3 rightLegPos = {legPos.x + GameSettings::BodyParts::LEG_OFFSET_X, legPos.y, legPos.z};
    createRagdollPart(leftLeg, leftLegPos, GameSettings::BodyParts::LEG_SIZE, 
                     COL_RAGDOLL_LEGS, LEGS_COLLIDE_WITH, GameSettings::Character::MASS * 0.15f);
    createRagdollPart(rightLeg, rightLegPos, GameSettings::BodyParts::LEG_SIZE, 
                     COL_RAGDOLL_LEGS, LEGS_COLLIDE_WITH, GameSettings::Character::MASS * 0.15f);
    
    // Create constraints to keep body parts together
    createConstraints();
    
    // Initial visual update
    updateVisualFromPhysics();
}

void FullRagdoll::createConstraints() {
    // ULTRA-MEGA strong constraint settings for MAXIMUM stability
    const float constraintBreakingThreshold = 100000.0f; // Extreme threshold - almost unbreakable
    const bool disableCollisionsBetweenLinkedBodies = true;
    
    // Head to torso constraint (neck) - ROCK SOLID
    btVector3 headPivot(0, -GameSettings::BodyParts::HEAD_SIZE.y * 0.5f - GameSettings::BodyParts::HEAD_OFFSET_Y, 0);
    btVector3 torsoPivot(0, GameSettings::BodyParts::TORSO_SIZE.y * 0.5f, 0);
    
    btPoint2PointConstraint* neckConstraint = new btPoint2PointConstraint(*head.body, *torso.body, headPivot, torsoPivot);
    neckConstraint->setBreakingImpulseThreshold(constraintBreakingThreshold);
    // Make constraint ROCK SOLID
    neckConstraint->m_setting.m_tau = 0.999f;      // Nearly rigid constraint
    neckConstraint->m_setting.m_damping = 0.999f;   // Maximum damping
    physicsWorld->addConstraint(neckConstraint, disableCollisionsBetweenLinkedBodies);
    constraints.push_back(neckConstraint);
    
    // Left arm to torso constraint (shoulder) - VERY STRONG
    btVector3 leftArmPivot(GameSettings::BodyParts::ARM_SIZE.x * 0.5f, GameSettings::BodyParts::ARM_SIZE.y * 0.3f, 0);
    btVector3 torsoLeftPivot(-GameSettings::BodyParts::TORSO_SIZE.x * 0.5f, GameSettings::BodyParts::TORSO_SIZE.y * 0.3f, 0);
    
    btPoint2PointConstraint* leftShoulderConstraint = new btPoint2PointConstraint(*leftArm.body, *torso.body, leftArmPivot, torsoLeftPivot);
    leftShoulderConstraint->setBreakingImpulseThreshold(constraintBreakingThreshold);
    leftShoulderConstraint->m_setting.m_tau = 0.95f;
    leftShoulderConstraint->m_setting.m_damping = 0.95f;
    physicsWorld->addConstraint(leftShoulderConstraint, disableCollisionsBetweenLinkedBodies);
    constraints.push_back(leftShoulderConstraint);
    
    // Right arm to torso constraint (shoulder) - VERY STRONG
    btVector3 rightArmPivot(-GameSettings::BodyParts::ARM_SIZE.x * 0.5f, GameSettings::BodyParts::ARM_SIZE.y * 0.3f, 0);
    btVector3 torsoRightPivot(GameSettings::BodyParts::TORSO_SIZE.x * 0.5f, GameSettings::BodyParts::TORSO_SIZE.y * 0.3f, 0);
    
    btPoint2PointConstraint* rightShoulderConstraint = new btPoint2PointConstraint(*rightArm.body, *torso.body, rightArmPivot, torsoRightPivot);
    rightShoulderConstraint->setBreakingImpulseThreshold(constraintBreakingThreshold);
    rightShoulderConstraint->m_setting.m_tau = 0.95f;
    rightShoulderConstraint->m_setting.m_damping = 0.95f;
    physicsWorld->addConstraint(rightShoulderConstraint, disableCollisionsBetweenLinkedBodies);
    constraints.push_back(rightShoulderConstraint);
    
    // Left leg to torso constraint (hip) - ABSOLUTELY UNBREAKABLE
    btVector3 leftLegPivot(GameSettings::BodyParts::LEG_SIZE.x * 0.25f, GameSettings::BodyParts::LEG_SIZE.y * 0.5f, 0);
    btVector3 torsoLeftHipPivot(-GameSettings::BodyParts::TORSO_SIZE.x * 0.25f, -GameSettings::BodyParts::TORSO_SIZE.y * 0.5f, 0);
    
    btPoint2PointConstraint* leftHipConstraint = new btPoint2PointConstraint(*leftLeg.body, *torso.body, leftLegPivot, torsoLeftHipPivot);
    leftHipConstraint->setBreakingImpulseThreshold(constraintBreakingThreshold * 10.0f); // Unbreakable
    leftHipConstraint->m_setting.m_tau = 0.9999f;      // ABSOLUTELY rigid constraint
    leftHipConstraint->m_setting.m_damping = 0.9999f;   // MAXIMUM damping
    physicsWorld->addConstraint(leftHipConstraint, disableCollisionsBetweenLinkedBodies);
    constraints.push_back(leftHipConstraint);
    
    // Right leg to torso constraint (hip) - ABSOLUTELY UNBREAKABLE
    btVector3 rightLegPivot(-GameSettings::BodyParts::LEG_SIZE.x * 0.25f, GameSettings::BodyParts::LEG_SIZE.y * 0.5f, 0);
    btVector3 torsoRightHipPivot(GameSettings::BodyParts::TORSO_SIZE.x * 0.25f, -GameSettings::BodyParts::TORSO_SIZE.y * 0.5f, 0);
    
    btPoint2PointConstraint* rightHipConstraint = new btPoint2PointConstraint(*rightLeg.body, *torso.body, rightLegPivot, torsoRightHipPivot);
    rightHipConstraint->setBreakingImpulseThreshold(constraintBreakingThreshold * 10.0f); // Unbreakable
    rightHipConstraint->m_setting.m_tau = 0.9999f;      // ABSOLUTELY rigid constraint
    rightHipConstraint->m_setting.m_damping = 0.9999f;   // MAXIMUM damping
    physicsWorld->addConstraint(rightHipConstraint, disableCollisionsBetweenLinkedBodies);
    constraints.push_back(rightHipConstraint);
}

void FullRagdoll::removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    // Remove constraints first
    for (auto* constraint : constraints) {
        bulletWorld->removeConstraint(constraint);
        delete constraint;
    }
    constraints.clear();
    
    // Helper function to clean up a ragdoll part
    auto cleanupPart = [bulletWorld](RagdollPart& part) {
        if (part.body) {
            bulletWorld->removeRigidBody(part.body);
            delete part.body;
            part.body = nullptr;
        }
        if (part.motionState) {
            delete part.motionState;
            part.motionState = nullptr;
        }
        if (part.shape) {
            delete part.shape;
            part.shape = nullptr;
        }
    };
    
    cleanupPart(head);
    cleanupPart(torso);
    cleanupPart(leftArm);
    cleanupPart(rightArm);
    cleanupPart(leftLeg);
    cleanupPart(rightLeg);
}

void FullRagdoll::updateVisualFromPhysics() {
    // Helper function to update visual from physics body
    auto updatePartVisual = [](RagdollPart& part) {
        if (!part.body) return;
        
        btTransform transform;
        part.body->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();
        btQuaternion rot = transform.getRotation();
        
        part.visual.setPosition({pos.x(), pos.y(), pos.z()});
        
        // Convert quaternion to euler angles for rotation
        float yaw, pitch, roll;
        rot.getEulerZYX(yaw, pitch, roll);
        // BodyPart only supports single axis rotation, use the most significant one
        float maxAngle = fabs(yaw);
        Vector3 rotAxis = {0, 1, 0};
        float rotAngle = yaw * RAD2DEG;
        
        if (fabs(pitch) > maxAngle) {
            maxAngle = fabs(pitch);
            rotAxis = {1, 0, 0};
            rotAngle = pitch * RAD2DEG;
        }
        if (fabs(roll) > maxAngle) {
            rotAxis = {0, 0, 1};
            rotAngle = roll * RAD2DEG;
        }
        
        part.visual.setRotation(rotAxis, rotAngle);
    };
    
    updatePartVisual(head);
    updatePartVisual(torso);
    updatePartVisual(leftArm);
    updatePartVisual(rightArm);
    updatePartVisual(leftLeg);
    updatePartVisual(rightLeg);
    
    // Update main position based on torso
    if (torso.body) {
        btTransform transform;
        torso.body->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();
        this->position = {pos.x(), pos.y(), pos.z()};
    }
}

void FullRagdoll::applyMovementForces(Vector3 movement, float speed) {
    if (!torso.body) return;
    
    // Apply forces to torso and legs for movement
    btVector3 force(movement.x * speed, 0, movement.z * speed);
    
    // Main movement force on torso
    torso.body->applyCentralForce(force * 0.7f);
    
    // Leg forces for walking
    if (leftLeg.body) leftLeg.body->applyCentralForce(force * 0.15f);
    if (rightLeg.body) rightLeg.body->applyCentralForce(force * 0.15f);
    
    // Keep torso more upright
    btVector3 angularVel = torso.body->getAngularVelocity();
    torso.body->setAngularVelocity(btVector3(angularVel.x() * 0.5f, angularVel.y(), angularVel.z() * 0.5f));
}

Vector3 FullRagdoll::getTorsoPosition() const {
    if (!torso.body) return this->position;
    
    btTransform transform;
    torso.body->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    return {pos.x(), pos.y(), pos.z()};
}

Vector3 FullRagdoll::getFeetPosition() const {
    // Get average position of both legs at their bottom
    Vector3 feetPos = {0, 0, 0};
    int legCount = 0;
    
    if (leftLeg.body) {
        btTransform transform;
        leftLeg.body->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();
        feetPos.x += pos.x();
        feetPos.y += pos.y() - GameSettings::BodyParts::LEG_SIZE.y * 0.5f; // Bottom of leg
        feetPos.z += pos.z();
        legCount++;
    }
    
    if (rightLeg.body) {
        btTransform transform;
        rightLeg.body->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();
        feetPos.x += pos.x();
        feetPos.y += pos.y() - GameSettings::BodyParts::LEG_SIZE.y * 0.5f; // Bottom of leg
        feetPos.z += pos.z();
        legCount++;
    }
    
    if (legCount > 0) {
        feetPos.x /= legCount;
        feetPos.y /= legCount;
        feetPos.z /= legCount;
    }
    
    return feetPos;
}

bool FullRagdoll::isOnGround() const {
    if (!world || (!leftLeg.body && !rightLeg.body)) return false;
    
    // Simplified and more robust ground detection
    auto checkLegGroundContact = [this](btRigidBody* legBody) -> bool {
        if (!legBody) return false;
        
        btTransform transform;
        legBody->getMotionState()->getWorldTransform(transform);
        btVector3 pos = transform.getOrigin();
        
        // Ray from bottom of leg downward - more generous range
        float rayStartY = pos.y() - GameSettings::BodyParts::LEG_SIZE.y * 0.5f + 0.1f; // Start slightly above bottom
        float rayEndY = rayStartY - 0.5f; // Check 0.5 units down
        
        btVector3 rayStart(pos.x(), rayStartY, pos.z());
        btVector3 rayEnd(pos.x(), rayEndY, pos.z());
        
        btCollisionWorld::ClosestRayResultCallback rayCallback(rayStart, rayEnd);
        rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
        rayCallback.m_collisionFilterMask = COL_GROUND | COL_OBJECTS;
        
        world->getBulletWorld()->rayTest(rayStart, rayEnd, rayCallback);
        
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != legBody) {
            float hitDistance = rayCallback.m_closestHitFraction * 0.5f;
            // More lenient ground detection - if we hit something within 0.3 units
            return hitDistance < 0.3f;
        }
        
        return false;
    };
    
    // Check both legs and torso for more robust detection
    bool legGroundContact = checkLegGroundContact(leftLeg.body) || checkLegGroundContact(rightLeg.body);
    
    // Also check if torso is low enough (backup detection)
    bool torsoLowEnough = false;
    if (torso.body) {
        btTransform torsoTransform;
        torso.body->getMotionState()->getWorldTransform(torsoTransform);
        btVector3 torsoPos = torsoTransform.getOrigin();
        torsoLowEnough = torsoPos.y() < 2.5f; // If torso is reasonably low, consider on ground
    }
    
    return legGroundContact || torsoLowEnough;
}

void FullRagdoll::handleInput(float movementSpeed) {
    if (!torso.body) return;

    Vector2 moveAxis = InputSystem::getMovementAxis();

    // Jump input
    if (InputSystem::isJumpPressed()) {
        bool onGround = isOnGround();
        if (onGround) {
            jump();
        }
    }

    // Movement input
    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f) {
        Vector3 movement = {moveAxis.x, 0, -moveAxis.y};
        float lengthSquared = movement.x * movement.x + movement.z * movement.z;
        if (lengthSquared > 1.0f) {
            float invLength = 1.0f / sqrtf(lengthSquared);
            movement.x *= invLength;
            movement.z *= invLength;
        }
        
        applyMovementForces(movement, movementSpeed * GameSettings::Character::MOVEMENT_FORCE);
    } else if (isOnGround()) {
        // Apply damping when not moving
        if (torso.body) {
            btVector3 velocity = torso.body->getLinearVelocity();
            velocity.setX(velocity.x() * 0.8f);
            velocity.setZ(velocity.z() * 0.8f);
            torso.body->setLinearVelocity(velocity);
        }
    }
}

void FullRagdoll::jump() {
    if (!isOnGround()) return;
    
    // Apply MUCH stronger upward impulse to all body parts
    btVector3 jumpImpulse(0, GameSettings::Character::JUMP_IMPULSE * 4.0f, 0); // Massive jump force
    
    if (torso.body) {
        torso.body->applyCentralImpulse(jumpImpulse * 0.9f); // 90% of force on torso
        torso.body->activate(true);
    }
    
    if (leftLeg.body) {
        leftLeg.body->applyCentralImpulse(jumpImpulse * 0.4f); // Strong leg push
        leftLeg.body->activate(true);
    }
    
    if (rightLeg.body) {
        rightLeg.body->applyCentralImpulse(jumpImpulse * 0.4f); // Strong leg push
        rightLeg.body->activate(true);
    }
    
    // Add head impulse for more realistic jump
    if (head.body) {
        head.body->applyCentralImpulse(jumpImpulse * 0.2f); // Significant head boost
        head.body->activate(true);
    }
    
    // Add arm impulse too
    if (leftArm.body) {
        leftArm.body->applyCentralImpulse(jumpImpulse * 0.1f);
        leftArm.body->activate(true);
    }
    
    if (rightArm.body) {
        rightArm.body->applyCentralImpulse(jumpImpulse * 0.1f);
        rightArm.body->activate(true);
    }
}

void FullRagdoll::animateLimbs(float deltaTime) {
    // For a realistic ragdoll, animation is handled by physics
    // We can add subtle forces for walking animation if needed
    if (!torso.body) return;
    
    btVector3 velocity = torso.body->getLinearVelocity();
    float speedSquared = velocity.x() * velocity.x() + velocity.z() * velocity.z();
    bool isMoving = speedSquared > GameSettings::Animation::SPEED_THRESHOLD;
    
    if (isMoving && isOnGround()) {
        // Add subtle walking forces
        animationTime += deltaTime * sqrtf(speedSquared) * GameSettings::Animation::SPEED_MULTIPLIER;
        
        float walkCycle = sinf(animationTime);
        float walkForce = walkCycle * 50.0f;
        
        // Alternate leg movement
        if (leftLeg.body && rightLeg.body) {
            leftLeg.body->applyCentralForce(btVector3(0, walkForce, 0));
            rightLeg.body->applyCentralForce(btVector3(0, -walkForce, 0));
        }
        
        // Subtle arm swing
        if (leftArm.body && rightArm.body) {
            leftArm.body->applyTorque(btVector3(walkForce * 0.1f, 0, 0));
            rightArm.body->applyTorque(btVector3(-walkForce * 0.1f, 0, 0));
        }
    }
}

void FullRagdoll::update(float deltaTime) {
    // Apply active stabilization to keep character upright
    applyStabilizationForces();
    
    updateVisualFromPhysics();
    animateLimbs(deltaTime);
}

void FullRagdoll::draw() const {
    head.visual.draw();
    torso.visual.draw();
    leftArm.visual.draw();
    rightArm.visual.draw();
    leftLeg.visual.draw();
    rightLeg.visual.draw();
}

BoundingBox FullRagdoll::getBoundingBox() const {
    Vector3 torsoPos = getTorsoPosition();
    float extent = 2.0f; // Conservative bounding box
    
    return {
        {torsoPos.x - extent, torsoPos.y - extent, torsoPos.z - extent},
        {torsoPos.x + extent, torsoPos.y + extent, torsoPos.z + extent}
    };
}

void FullRagdoll::applyStabilizationForces() {
    if (!torso.body || !leftLeg.body || !rightLeg.body) return;
    
    // EXTREMELY STRONG active stabilization to keep character upright and stable
    
    // 1. Keep torso upright by correcting rotation with MASSIVE force
    btTransform torsoTransform;
    torso.body->getMotionState()->getWorldTransform(torsoTransform);
    btQuaternion torsoRot = torsoTransform.getRotation();
    
    // Calculate how much the torso is tilted from vertical
    btVector3 upVector = btMatrix3x3(torsoRot) * btVector3(0, 1, 0);
    btVector3 targetUp(0, 1, 0);
    
    // Apply MASSIVE corrective torque to keep torso upright
    btVector3 correctionTorque = upVector.cross(targetUp) * 500.0f; // Massive increase from 100.0f
    torso.body->applyTorque(correctionTorque);
    
    // 2. Stabilize legs to support the torso with ENORMOUS forces
    btTransform leftLegTransform, rightLegTransform;
    leftLeg.body->getMotionState()->getWorldTransform(leftLegTransform);
    rightLeg.body->getMotionState()->getWorldTransform(rightLegTransform);
    
    // Apply HUGE upward forces to legs to support the torso ALWAYS
    float supportForce = 200.0f; // Massive increase from 50.0f
    leftLeg.body->applyCentralForce(btVector3(0, supportForce, 0));
    rightLeg.body->applyCentralForce(btVector3(0, supportForce, 0));
    
    // Keep legs vertical and stable with MASSIVE forces
    btQuaternion leftLegRot = leftLegTransform.getRotation();
    btQuaternion rightLegRot = rightLegTransform.getRotation();
    
    btVector3 leftLegUp = btMatrix3x3(leftLegRot) * btVector3(0, 1, 0);
    btVector3 rightLegUp = btMatrix3x3(rightLegRot) * btVector3(0, 1, 0);
    
    // Apply MASSIVE corrective torques to keep legs vertical
    btVector3 leftLegCorrection = leftLegUp.cross(targetUp) * 300.0f; // Massive increase from 75.0f
    btVector3 rightLegCorrection = rightLegUp.cross(targetUp) * 300.0f; // Massive increase from 75.0f
    
    leftLeg.body->applyTorque(leftLegCorrection);
    rightLeg.body->applyTorque(rightLegCorrection);
    
    // 3. Keep head stable too
    if (head.body) {
        btTransform headTransform;
        head.body->getMotionState()->getWorldTransform(headTransform);
        btQuaternion headRot = headTransform.getRotation();
        
        btVector3 headUp = btMatrix3x3(headRot) * btVector3(0, 1, 0);
        btVector3 headCorrection = headUp.cross(targetUp) * 150.0f; // Strong head stabilization
        head.body->applyTorque(headCorrection);
        
        // Keep head above torso
        btVector3 headPos = headTransform.getOrigin();
        btVector3 torsoPos = torsoTransform.getOrigin();
        if (headPos.y() < torsoPos.y() + 0.5f) {
            head.body->applyCentralForce(btVector3(0, 100.0f, 0)); // Push head up
        }
    }
    
    // 4. Keep arms in reasonable positions
    if (leftArm.body && rightArm.body) {
        btTransform leftArmTransform, rightArmTransform;
        leftArm.body->getMotionState()->getWorldTransform(leftArmTransform);
        rightArm.body->getMotionState()->getWorldTransform(rightArmTransform);
        
        btQuaternion leftArmRot = leftArmTransform.getRotation();
        btQuaternion rightArmRot = rightArmTransform.getRotation();
        
        btVector3 leftArmUp = btMatrix3x3(leftArmRot) * btVector3(0, 1, 0);
        btVector3 rightArmUp = btMatrix3x3(rightArmRot) * btVector3(0, 1, 0);
        
        btVector3 leftArmCorrection = leftArmUp.cross(targetUp) * 50.0f;
        btVector3 rightArmCorrection = rightArmUp.cross(targetUp) * 50.0f;
        
        leftArm.body->applyTorque(leftArmCorrection);
        rightArm.body->applyTorque(rightArmCorrection);
    }
    
    // 5. Dampen excessive angular velocities for stability
    float dampingFactor = 0.7f; // More aggressive damping
    
    btVector3 torsoAngVel = torso.body->getAngularVelocity();
    torso.body->setAngularVelocity(torsoAngVel * dampingFactor);
    
    btVector3 leftLegAngVel = leftLeg.body->getAngularVelocity();
    leftLeg.body->setAngularVelocity(leftLegAngVel * dampingFactor);
    
    btVector3 rightLegAngVel = rightLeg.body->getAngularVelocity();
    rightLeg.body->setAngularVelocity(rightLegAngVel * dampingFactor);
    
    if (head.body) {
        btVector3 headAngVel = head.body->getAngularVelocity();
        head.body->setAngularVelocity(headAngVel * dampingFactor);
    }
}

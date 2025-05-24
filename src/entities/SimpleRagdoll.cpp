// SimpleRagdoll.cpp - Simple character with single physics body
#include "entities/SimpleRagdoll.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raymath.h"
#include <cmath>

SimpleRagdoll::SimpleRagdoll(Vector3 position)
    : GameObject(position, true, true, false),
      // Initialize visual body parts (proportional to character)
      torso({0, 0, 0}, {0.6f, 1.0f, 0.3f}, BLUE, false),
      head({0, 0, 0}, {0.4f, 0.4f, 0.4f}, RED, false),
      leftArm({0, 0, 0}, {0.2f, 0.6f, 0.2f}, GREEN, false),
      rightArm({0, 0, 0}, {0.2f, 0.6f, 0.2f}, GREEN, false),
      leftLeg({0, 0, 0}, {0.25f, 0.8f, 0.25f}, YELLOW, false),
      rightLeg({0, 0, 0}, {0.25f, 0.8f, 0.25f}, YELLOW, false) {
}

SimpleRagdoll::~SimpleRagdoll() {
    // Physics cleanup handled by removeFromPhysics
}

void SimpleRagdoll::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    // Create a capsule shape for the character (good for character controllers)
    physicsShape = new btCapsuleShape(CHARACTER_RADIUS, CHARACTER_HEIGHT - 2*CHARACTER_RADIUS);
    
    // CRITICAL: Add collision margin to prevent feet from being exactly on floor surface
    physicsShape->setMargin(0.1f); // 10cm margin to create gap between character and floor
    
    // Set position above ground - minimal clearance since we now have collision margin
    // Floor top is at Y=0.50, so character center should be at Y = 0.50 + CHARACTER_HEIGHT/2
    Vector3 physicsPos = this->position;
    float calculatedY = 0.50f + CHARACTER_HEIGHT / 2 + 0.05f; // Position at floor + half height + small clearance
    physicsPos.y = calculatedY;
    
    // Create motion state
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(physicsPos.x, physicsPos.y, physicsPos.z));
    motionState = new btDefaultMotionState(transform);
    
    // Calculate inertia
    btVector3 inertia(0, 0, 0);
    physicsShape->calculateLocalInertia(CHARACTER_MASS, inertia);
    
    // Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rbInfo(CHARACTER_MASS, motionState, physicsShape, inertia);
    
    // Set physics properties for character - REDUCED FRICTION to prevent sticking
    rbInfo.m_friction = 0.1f;        // Very low friction to reduce surface adhesion
    rbInfo.m_rollingFriction = 0.0f; // No rolling friction 
    rbInfo.m_restitution = 0.0f;     // No bouncing
    
    physicsBody = new btRigidBody(rbInfo);
    
    // Configure character physics - More responsive settings
    physicsBody->setDamping(DAMPING_LINEAR, DAMPING_ANGULAR);
    physicsBody->setActivationState(DISABLE_DEACTIVATION); // Always active
    physicsBody->setAngularFactor(btVector3(0, 1, 0)); // Only allow Y-axis rotation (turning)
    
    // CRITICAL: Allow movement in all directions (especially Y for jumping!)
    physicsBody->setLinearFactor(btVector3(1, 1, 1)); // Allow X, Y, Z movement
    
    // CRITICAL: Set contact processing threshold to prevent collision interference
    physicsBody->setContactProcessingThreshold(0.0f); // Process all contacts
    
    // Use world gravity instead of custom gravity
    // physicsBody->setGravity(btVector3(0, -15.0f, 0)); // Removed: was overriding world gravity
    

    

    

    

    
    // Add to physics world
    bulletWorld->addRigidBody(physicsBody);
    
    // CRITICAL: Explicitly set position AFTER adding to world to override any settling
    btTransform finalTransform;
    finalTransform.setIdentity();
    finalTransform.setOrigin(btVector3(physicsPos.x, physicsPos.y, physicsPos.z));
    physicsBody->setWorldTransform(finalTransform);
    physicsBody->getMotionState()->setWorldTransform(finalTransform);
    
    // Force activation and clear any settling velocities
    physicsBody->activate(true);
    physicsBody->setLinearVelocity(btVector3(0, 0, 0));
    physicsBody->setAngularVelocity(btVector3(0, 0, 0));
    

    
    // Update visual positions immediately
    updateVisualFromPhysics();
    

}

void SimpleRagdoll::removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    if (physicsBody) {
        bulletWorld->removeRigidBody(physicsBody);
        delete physicsBody;
        physicsBody = nullptr;
    }
    if (motionState) {
        delete motionState;
        motionState = nullptr;
    }
    if (physicsShape) {
        delete physicsShape;
        physicsShape = nullptr;
    }
}

void SimpleRagdoll::updateVisualFromPhysics() {
    if (!physicsBody) return;
    
    // Get physics body position and rotation
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 physicsPos = transform.getOrigin();
    btQuaternion rotation = transform.getRotation();
    
    // Calculate body parts positions relative to physics body center
    Vector3 centerPos = {physicsPos.x(), physicsPos.y(), physicsPos.z()};
    
    // Head: above center
    Vector3 headPos = {
        centerPos.x,
        centerPos.y + CHARACTER_HEIGHT/2 - 0.2f, // Near top of capsule
        centerPos.z
    };
    
    // Torso: at center
    Vector3 torsoPos = centerPos;
    
    // Arms: at shoulder level
    float shoulderY = centerPos.y + CHARACTER_HEIGHT/4;
    Vector3 leftArmPos = {
        centerPos.x - 0.4f,
        shoulderY,
        centerPos.z
    };
    Vector3 rightArmPos = {
        centerPos.x + 0.4f,
        shoulderY,
        centerPos.z
    };
    
    // Legs: below center
    float legY = centerPos.y - CHARACTER_HEIGHT/4;
    Vector3 leftLegPos = {
        centerPos.x - 0.15f,
        legY,
        centerPos.z
    };
    Vector3 rightLegPos = {
        centerPos.x + 0.15f,
        legY,
        centerPos.z
    };
    
    // Update visual parts
    head.setPosition(headPos);
    torso.setPosition(torsoPos);
    leftArm.setPosition(leftArmPos);
    rightArm.setPosition(rightArmPos);
    leftLeg.setPosition(leftLegPos);
    rightLeg.setPosition(rightLegPos);
    
    // Apply character rotation to visual parts
    float yaw = 0.0f;
    rotation.getEulerZYX(yaw, yaw, yaw); // Get Y rotation
    Vector3 rotAxis = {0, 1, 0};
    
    head.setRotation(rotAxis, yaw * RAD2DEG);
    torso.setRotation(rotAxis, yaw * RAD2DEG);
    leftArm.setRotation(rotAxis, yaw * RAD2DEG);
    rightArm.setRotation(rotAxis, yaw * RAD2DEG);
    leftLeg.setRotation(rotAxis, yaw * RAD2DEG);
    rightLeg.setRotation(rotAxis, yaw * RAD2DEG);
    
    // Update GameObject position
    this->position = centerPos;
}

void SimpleRagdoll::animateLimbs(float deltaTime) {
    if (!physicsBody) return;
    
    // Get movement velocity for animation
    btVector3 velocity = physicsBody->getLinearVelocity();
    float speed = Vector3Length({velocity.x(), 0, velocity.z()}); // Horizontal speed only
    bool isMoving = speed > 0.5f; // Threshold for animation
    
    if (isMoving) {
        animationTime += deltaTime * speed * 2.0f; // Animation speed based on movement speed
        
        float armSwing = sin(animationTime) * 20.0f; // Degrees
        float legSwing = sin(animationTime) * 25.0f;
        
        // Apply animation with setRotation (not addRotation)
        leftArm.setRotation({1, 0, 0}, armSwing);
        rightArm.setRotation({1, 0, 0}, -armSwing);
        leftLeg.setRotation({1, 0, 0}, -legSwing);
        rightLeg.setRotation({1, 0, 0}, legSwing);
    } else {
        // Return to neutral pose when not moving
        leftArm.setRotation({1, 0, 0}, 0.0f);
        rightArm.setRotation({1, 0, 0}, 0.0f);
        leftLeg.setRotation({1, 0, 0}, 0.0f);
        rightLeg.setRotation({1, 0, 0}, 0.0f);
    }
}

Vector3 SimpleRagdoll::getPhysicsPosition() const {
    if (!physicsBody) return this->position;
    
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    return {pos.x(), pos.y(), pos.z()};
}

Vector3 SimpleRagdoll::getFeetPosition() const {
    Vector3 centerPos = getPhysicsPosition();
    // Return position at the bottom of the capsule (feet level)
    centerPos.y -= CHARACTER_HEIGHT / 2;
    return centerPos;
}

bool SimpleRagdoll::isOnGround() const {
    if (!physicsBody || !world) return false;
    
    // Get current position
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    
    // Cast ray downward from bottom of capsule - improved for better surface detection
    float rayStartY = pos.y() - CHARACTER_HEIGHT/2 + 0.1f; // Start slightly above bottom
    float rayEndY = rayStartY - 0.2f; // 20cm below (shorter, more precise)
    
    btVector3 rayStart(pos.x(), rayStartY, pos.z());
    btVector3 rayEnd(pos.x(), rayEndY, pos.z());
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayStart, rayEnd);
    
    // Don't exclude any objects - we want to detect ALL surfaces
    rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
    rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
    
    world->getBulletWorld()->rayTest(rayStart, rayEnd, rayCallback);
    
    // Check if we hit something that's not ourselves
    if (rayCallback.hasHit() && rayCallback.m_collisionObject != physicsBody) {
        float hitDistance = rayCallback.m_closestHitFraction * 0.2f; // Distance to ground in meters
        btVector3 velocity = physicsBody->getLinearVelocity();
        

        
        // We're on ground if we hit something close and not moving up fast
        return hitDistance < 0.20f && velocity.y() < 2.0f; // Increased tolerance to 20cm for elevated character
    }
    
    return false;
}

void SimpleRagdoll::handleInput(float movementSpeed) {
    if (!physicsBody) return;

    Vector2 moveAxis = InputSystem::getMovementAxis();
    


    // HANDLE JUMP FIRST - before movement can override Y velocity
    if (InputSystem::isJumpPressed()) {
        bool onGround = isOnGround();
               
        if (onGround) {
            jump();
        }
    }

    // Apply movement forces AFTER jump (preserve Y velocity from jump)
    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f) {
        // Calculate movement direction in world space
        Vector3 movement = {moveAxis.x, 0, -moveAxis.y}; // Z is inverted for forward
        movement = Vector3Normalize(movement);
        
        // Get current velocity (including any Y velocity from jumping)
        btVector3 velocity = physicsBody->getLinearVelocity();
        
        // Store Y velocity before modification
        float preservedY = velocity.y();
        
        // Set horizontal velocity directly for more responsive movement
        float targetSpeed = movementSpeed;
        velocity.setX(movement.x * targetSpeed);
        velocity.setZ(movement.z * targetSpeed);
        
        // PRESERVE Y velocity (gravity/jumping) - don't touch it!
        velocity.setY(preservedY); // Explicitly preserve Y velocity
        physicsBody->setLinearVelocity(velocity);
        

        

        
    } else if (isOnGround()) {
        // Stop horizontal movement when no input
        btVector3 velocity = physicsBody->getLinearVelocity();
        velocity.setX(0);
        velocity.setZ(0);
        // PRESERVE Y velocity (gravity/jumping) - don't touch it!
        physicsBody->setLinearVelocity(velocity);
    }
}

void SimpleRagdoll::jump() {
    if (!physicsBody) return;
    
    // Reset Y velocity to ensure clean jump
    btVector3 velocity = physicsBody->getLinearVelocity();
    velocity.setY(0); // Clear any existing Y velocity
    physicsBody->setLinearVelocity(velocity);
    
    // ALTERNATIVE APPROACH: Set velocity directly instead of impulse
    // This bypasses collision resolution that might suppress impulse forces
    btVector3 jumpVelocity = physicsBody->getLinearVelocity();
    jumpVelocity.setY(8.0f); // Direct upward velocity (strong enough to overcome collision suppression)
    physicsBody->setLinearVelocity(jumpVelocity);
    
    // Also apply impulse for additional force
    float jumpForce = JUMP_IMPULSE * 2.0f; // Double the impulse force
    btVector3 jumpImpulse(0, jumpForce, 0);
    physicsBody->applyCentralImpulse(jumpImpulse);
    
    // Force the physics body to wake up and be active
    physicsBody->activate(true);
    
    // Temporarily reduce contact processing to allow movement
    physicsBody->setContactProcessingThreshold(1.0f); // Ignore small contacts temporarily
}

void SimpleRagdoll::update(float deltaTime) {
    // Update visual from physics
    updateVisualFromPhysics();
    
    // Animate limbs
    animateLimbs(deltaTime);
}

void SimpleRagdoll::draw() const {
    // Draw all visual body parts
    torso.draw();
    head.draw();
    leftArm.draw();
    rightArm.draw();
    leftLeg.draw();
    rightLeg.draw();
    
    // Debug: Draw physics body outline
    if (physicsBody) {
        Vector3 pos = getPhysicsPosition();
        // Draw wireframe capsule for debugging
        DrawCylinderWires(pos, CHARACTER_RADIUS, CHARACTER_RADIUS, CHARACTER_HEIGHT, 8, GREEN);
    }
}

BoundingBox SimpleRagdoll::getBoundingBox() const {
    Vector3 pos = getPhysicsPosition();
    float halfHeight = CHARACTER_HEIGHT / 2;
    
    return {
        {pos.x - CHARACTER_RADIUS, pos.y - halfHeight, pos.z - CHARACTER_RADIUS},
        {pos.x + CHARACTER_RADIUS, pos.y + halfHeight, pos.z + CHARACTER_RADIUS}
    };
}

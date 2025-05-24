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
    
    printf("[DEBUG] SimpleRagdoll created at position: (%.2f, %.2f, %.2f)\n", 
           position.x, position.y, position.z);
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
    
    printf("[DEBUG] Position calculation: floor=0.50 + height/2=%.2f + clearance=0.05 = %.2f\n", 
           CHARACTER_HEIGHT/2, calculatedY);
    printf("[DEBUG] Setting character center at Y=%.2f (was Y=%.2f) with collision margin=0.10\n", 
           physicsPos.y, this->position.y);
    
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
    
    // Debug physics body properties
    printf("[DEBUG] Physics body mass: %.2f, isStaticObject: %s, isKinematicObject: %s\n",
           physicsBody->getMass(),
           physicsBody->isStaticObject() ? "true" : "false",
           physicsBody->isKinematicObject() ? "true" : "false");
    
    // Debug linear and angular factors
    btVector3 linearFactor = physicsBody->getLinearFactor();
    btVector3 angularFactor = physicsBody->getAngularFactor();
    printf("[DEBUG] Linear factor: (%.2f, %.2f, %.2f), Angular factor: (%.2f, %.2f, %.2f)\n",
           linearFactor.x(), linearFactor.y(), linearFactor.z(),
           angularFactor.x(), angularFactor.y(), angularFactor.z());
    
    // Debug damping settings
    printf("[DEBUG] Linear damping: %.3f, Angular damping: %.3f\n",
           physicsBody->getLinearDamping(), physicsBody->getAngularDamping());
    
    // Debug gravity settings
    btVector3 bodyGravity = physicsBody->getGravity();
    btVector3 worldGravity = bulletWorld->getGravity();
    printf("[DEBUG] Body gravity: (%.2f, %.2f, %.2f), World gravity: (%.2f, %.2f, %.2f)\n",
           bodyGravity.x(), bodyGravity.y(), bodyGravity.z(),
           worldGravity.x(), worldGravity.y(), worldGravity.z());
    
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
    
    printf("[DEBUG] Final position after world add: Center Y=%.2f, Feet Y=%.2f\n", 
           physicsPos.y, physicsPos.y - CHARACTER_HEIGHT/2);
    
    // Update visual positions immediately
    updateVisualFromPhysics();
    
    printf("[DEBUG] SimpleRagdoll physics setup complete - Capsule character controller\n");
    printf("[DEBUG] Character height: %.2f, radius: %.2f, mass: %.2f\n", 
           CHARACTER_HEIGHT, CHARACTER_RADIUS, CHARACTER_MASS);
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
        
        // Debug ground detection with hit object info
        static int groundDebugCounter = 0;
        if (++groundDebugCounter % 60 == 0) {
            const btRigidBody* hitBody = btRigidBody::upcast(rayCallback.m_collisionObject);
            printf("[GROUND] Hit distance: %.3f, Y velocity: %.2f, Hit object: %p\n", 
                   hitDistance, velocity.y(), hitBody);
        }
        
        // We're on ground if we hit something close and not moving up fast
        return hitDistance < 0.20f && velocity.y() < 2.0f; // Increased tolerance to 20cm for elevated character
    }
    
    return false;
}

bool SimpleRagdoll::isOnGroundContact() const {
    if (!physicsBody || !world) return false;
    
    // Check for contact points with other objects
    btDiscreteDynamicsWorld* dynamicsWorld = world->getBulletWorld();
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    
    // Get our character's bottom position
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 characterPos = transform.getOrigin();
    float characterBottom = characterPos.y() - CHARACTER_HEIGHT/2;
    
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        
        // Check if one of the bodies is our character
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        
        if (obA == physicsBody || obB == physicsBody) {
            int numContacts = contactManifold->getNumContacts();
            for (int j = 0; j < numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.05f) { // Very close contact
                    // Get contact point
                    btVector3 contactPoint = (obA == physicsBody) ? pt.getPositionWorldOnA() : pt.getPositionWorldOnB();
                    
                    // Check if contact is near the bottom of the character (ground contact)
                    float contactHeight = contactPoint.y();
                    if (contactHeight <= characterBottom + 0.2f) { // Within 20cm of feet
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

void SimpleRagdoll::handleInput(float movementSpeed) {
    if (!physicsBody) return;

    Vector2 moveAxis = InputSystem::getMovementAxis();
    
    // Debug: Print current velocity and position with more detail
    btVector3 currentVel = physicsBody->getLinearVelocity();
    Vector3 currentPos = getPhysicsPosition();
    Vector3 feetPos = getFeetPosition();
    static int debugCounter = 0;
    if (++debugCounter % 60 == 0) { // Every second
        printf("[DEBUG] Center: (%.2f, %.2f, %.2f), Feet: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f), Ground: %s\n",
               currentPos.x, currentPos.y, currentPos.z,
               feetPos.x, feetPos.y, feetPos.z,
               currentVel.x(), currentVel.y(), currentVel.z(),
               isOnGround() ? "YES" : "NO");
        
        // Check for penetration with floor (floor top is at Y=0.50, character should be above)
        if (feetPos.y < 0.52f) { // Allow 2cm margin
            printf("[DEBUG] WARNING: Character feet too close to floor! Feet Y=%.3f, Floor top=0.500\n", feetPos.y);
        }
    }

    // HANDLE JUMP FIRST - before movement can override Y velocity
    if (InputSystem::isJumpPressed()) {
        bool raycastGround = isOnGround();
        bool contactGround = isOnGroundContact();
        bool canJump = raycastGround || contactGround;
        
        printf("[JUMP] Jump pressed! Raycast: %s, Contact: %s, Can jump: %s\n", 
               raycastGround ? "YES" : "NO", 
               contactGround ? "YES" : "NO",
               canJump ? "YES" : "NO");
               
        if (canJump) {
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
        
        // Debug: Check if Y velocity was preserved
        btVector3 afterVel = physicsBody->getLinearVelocity();
        if (std::abs(preservedY) > 0.1f) { // Only log when there's significant Y velocity
            printf("[MOVEMENT] Preserved Y velocity: %.3f -> %.3f (should be same)\n", 
                   preservedY, afterVel.y());
        }
        
        // Remove the dual impulse - just use velocity setting for cleaner movement
        static int moveDebugCounter = 0;
        if (++moveDebugCounter % 30 == 0) {
            printf("[INPUT] Moving with velocity: (%.2f, %.2f, %.2f)\n", 
                   velocity.x(), velocity.y(), velocity.z());
        }
        
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
    
    // Get current velocity and position for debugging
    btVector3 currentVel = physicsBody->getLinearVelocity();
    Vector3 currentPos = getPhysicsPosition();
    
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
    
    // Check velocity immediately after applying impulse
    btVector3 newVel = physicsBody->getLinearVelocity();
    
    printf("[JUMP] Character jumped! Pos: (%.2f, %.2f, %.2f), PrevVel: (%.2f, %.2f, %.2f), Jump impulse: %.2f\n",
           currentPos.x, currentPos.y, currentPos.z,
           currentVel.x(), currentVel.y(), currentVel.z(), jumpForce);
    printf("[JUMP] Velocity AFTER impulse: (%.2f, %.2f, %.2f)\n",
           newVel.x(), newVel.y(), newVel.z());
    
    // Debug: Check if velocity persists after a short delay
    printf("[JUMP] Applied direct velocity: %.2f, plus impulse: %.2f\n", 8.0f, jumpForce);
}

void SimpleRagdoll::update(float deltaTime) {
    // Update visual from physics
    updateVisualFromPhysics();
    
    // Animate limbs
    animateLimbs(deltaTime);
    
    // Debug output
    static float debugTimer = 0;
    debugTimer += deltaTime;
    if (debugTimer > 2.0f) { // Every 2 seconds
        Vector3 pos = getPhysicsPosition();
        bool onGround = isOnGround();
        
        if (physicsBody) {
            btVector3 velocity = physicsBody->getLinearVelocity();
            printf("[DEBUG] Pos: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f), Ground: %s\n",
                   pos.x, pos.y, pos.z,
                   velocity.x(), velocity.y(), velocity.z(),
                   onGround ? "YES" : "NO");
        }
        debugTimer = 0;
    }
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

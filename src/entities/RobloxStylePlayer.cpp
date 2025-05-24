// RobloxStylePlayer.cpp - True multi-part character implementation
#include "entities/RobloxStylePlayer.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raymath.h"
#include <cmath>

RobloxStylePlayer::RobloxStylePlayer(Vector3 position)
    : GameObject(position, true, true, false),
      // Initialize visual body parts with temporary positions (will be updated by physics)
      torso({0, 0, 0}, {0.8f, 1.2f, 0.4f}, BLUE, false),
      head({0, 0, 0}, {0.8f, 0.8f, 0.8f}, RED, false),  // Use cube size for head
      leftArm({0, 0, 0}, {0.3f, 0.8f, 0.3f}, GREEN, false),
      rightArm({0, 0, 0}, {0.3f, 0.8f, 0.3f}, GREEN, false),
      leftLeg({0, 0, 0}, {0.3f, 0.8f, 0.3f}, YELLOW, false),
      rightLeg({0, 0, 0}, {0.3f, 0.8f, 0.3f}, YELLOW, false) {
    
    printf("[DEBUG] RobloxStylePlayer created at ground position: (%.2f, %.2f, %.2f)\n", 
           position.x, position.y, position.z);
}

RobloxStylePlayer::~RobloxStylePlayer() {
    // Physics cleanup is handled by BodyPartPhysics destructors
}

void RobloxStylePlayer::createBodyPartPhysics(BodyPartPhysics& physics, const Vector3& size, 
                                            const Vector3& position, float mass, bool isSphere) {
    // Create collision shape
    if (isSphere) {
        physics.shape = new btSphereShape(size.x); // size.x is radius for sphere
    } else {
        physics.shape = new btBoxShape(btVector3(size.x/2, size.y/2, size.z/2));
    }
    
    // Create motion state
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(position.x, position.y, position.z));
    physics.motionState = new btDefaultMotionState(transform);
    
    // Calculate inertia
    btVector3 inertia(0, 0, 0);
    if (mass > 0) {
        physics.shape->calculateLocalInertia(mass, inertia);
    }
    
    // Create rigid body
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, physics.motionState, physics.shape, inertia);
    
    // Set physics properties for character parts
    rbInfo.m_friction = 0.7f;
    rbInfo.m_rollingFriction = 0.1f;
    rbInfo.m_restitution = 0.1f;
    
    physics.rigidBody = new btRigidBody(rbInfo);
    
    // Configure for character physics
    physics.rigidBody->setDamping(DAMPING_LINEAR, DAMPING_ANGULAR);
    physics.rigidBody->setActivationState(DISABLE_DEACTIVATION);
    
    // Prevent parts from sleeping
    physics.rigidBody->forceActivationState(ACTIVE_TAG);
    
    printf("[DEBUG] Created body part physics at (%.2f, %.2f, %.2f) with mass %.1f\n", 
           position.x, position.y, position.z, mass);
}

void RobloxStylePlayer::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    // Calculate proper positions for Roblox-style character
    // Ground position is where feet should touch - assume ground is at Y=0.50 (top of floor box)
    Vector3 groundPos = this->position;
    groundPos.y = 0.50f; // Floor top surface, not center
    
    // CRITICAL: Position body parts so FEET touch ground surface
    float legHeight = dimensions.legSize.y;
    float torsoHeight = dimensions.torsoSize.y;
    
    // Legs: positioned so their BOTTOM touches ground surface
    Vector3 leftLegPos = {
        groundPos.x - 0.2f,
        groundPos.y + legHeight/2,  // Center = ground + half leg height
        groundPos.z
    };
    Vector3 rightLegPos = {
        groundPos.x + 0.2f,
        groundPos.y + legHeight/2,  // Center = ground + half leg height
        groundPos.z
    };
    
    // Torso: positioned above legs
    Vector3 torsoPos = {
        groundPos.x,
        groundPos.y + legHeight + torsoHeight/2,  // Above legs
        groundPos.z
    };
    
    // Head: positioned above torso
    Vector3 headPos = {
        groundPos.x,
        groundPos.y + legHeight + torsoHeight + dimensions.headRadius,  // Above torso
        groundPos.z
    };
    
    // Arms: positioned at torso level
    float armY = groundPos.y + legHeight + torsoHeight * 0.7f;  // Upper torso level
    Vector3 leftArmPos = {
        groundPos.x - (dimensions.torsoSize.x/2 + dimensions.armSize.x/2),
        armY,
        groundPos.z
    };
    Vector3 rightArmPos = {
        groundPos.x + (dimensions.torsoSize.x/2 + dimensions.armSize.x/2),
        armY,
        groundPos.z
    };
    
    // Create physics for each body part
    createBodyPartPhysics(leftLegPhysics, dimensions.legSize, leftLegPos, LEG_MASS);
    createBodyPartPhysics(rightLegPhysics, dimensions.legSize, rightLegPos, LEG_MASS);
    createBodyPartPhysics(torsoPhysics, dimensions.torsoSize, torsoPos, TORSO_MASS);
    createBodyPartPhysics(headPhysics, {dimensions.headRadius, 0, 0}, headPos, HEAD_MASS, true);
    createBodyPartPhysics(leftArmPhysics, dimensions.armSize, leftArmPos, ARM_MASS);
    createBodyPartPhysics(rightArmPhysics, dimensions.armSize, rightArmPos, ARM_MASS);
    
    // Add all body parts to physics world
    bulletWorld->addRigidBody(leftLegPhysics.rigidBody);
    bulletWorld->addRigidBody(rightLegPhysics.rigidBody);
    bulletWorld->addRigidBody(torsoPhysics.rigidBody);
    bulletWorld->addRigidBody(headPhysics.rigidBody);
    bulletWorld->addRigidBody(leftArmPhysics.rigidBody);
    bulletWorld->addRigidBody(rightArmPhysics.rigidBody);
    
    // Create constraints to keep body parts together
    createConstraints(bulletWorld);
    
    // CRITICAL: Update visual positions immediately after physics setup
    updateVisualFromPhysics();
    
    // IMPORTANT: Fix character positioning - ensure feet are properly on ground
    Vector3 currentFeet = getFeetPosition();
    if (currentFeet.y < groundPos.y) {
        printf("[DEBUG] Character feet below ground (%.2f < %.2f), repositioning...\n", 
               currentFeet.y, groundPos.y);
        setFeetOnGround(groundPos);
        updateVisualFromPhysics(); // Update again after repositioning
    }
    
    printf("[DEBUG] RobloxStylePlayer physics setup complete!\n");
    printf("[DEBUG] Feet position: Y=%.2f, Torso center: Y=%.2f, Head center: Y=%.2f\n",
           currentFeet.y, torsoPos.y, headPos.y);
    printf("[DEBUG] Character height: %.2f (legs: %.2f + torso: %.2f + head: %.2f)\n",
           legHeight + torsoHeight + dimensions.headRadius * 2,
           legHeight, torsoHeight, dimensions.headRadius * 2);
}

void RobloxStylePlayer::createConstraints(btDiscreteDynamicsWorld* world) {
    // Instead of complex constraints, let's make only the torso dynamic and other parts kinematic
    // This allows the character to move as a unit while maintaining visual structure
    
    // Make all body parts except torso kinematic (they won't be affected by physics forces directly)
    leftLegPhysics.rigidBody->setCollisionFlags(leftLegPhysics.rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rightLegPhysics.rigidBody->setCollisionFlags(rightLegPhysics.rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    headPhysics.rigidBody->setCollisionFlags(headPhysics.rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    leftArmPhysics.rigidBody->setCollisionFlags(leftArmPhysics.rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rightArmPhysics.rigidBody->setCollisionFlags(rightArmPhysics.rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    
    // Ensure torso is definitely dynamic and properly configured
    torsoPhysics.rigidBody->setCollisionFlags(torsoPhysics.rigidBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    torsoPhysics.rigidBody->setActivationState(ACTIVE_TAG);
    torsoPhysics.rigidBody->forceActivationState(ACTIVE_TAG);
    
    // Reduce damping for better movement response
    torsoPhysics.rigidBody->setDamping(0.1f, 0.3f); // Lower linear damping
    
    // Debug torso configuration
    float mass = 1.0f / torsoPhysics.rigidBody->getInvMass();
    printf("[DEBUG] Torso configuration - Mass: %.2f, Kinematic: %s, Active: %s\n",
           mass,
           (torsoPhysics.rigidBody->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) ? "YES" : "NO",
           torsoPhysics.rigidBody->getActivationState() == ACTIVE_TAG ? "YES" : "NO");
    
    // Only the torso will be dynamic and respond to movement forces
    // The other parts will be positioned relative to the torso in updateVisualFromPhysics()
    
    printf("[DEBUG] Set torso as dynamic body, other parts as kinematic\n");
}

void RobloxStylePlayer::removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    // Remove constraints first
    for (auto& constraint : constraints) {
        if (constraint->constraint) {
            bulletWorld->removeConstraint(constraint->constraint);
        }
    }
    constraints.clear();
    
    // Remove rigid bodies from world
    if (leftLegPhysics.rigidBody) bulletWorld->removeRigidBody(leftLegPhysics.rigidBody);
    if (rightLegPhysics.rigidBody) bulletWorld->removeRigidBody(rightLegPhysics.rigidBody);
    if (torsoPhysics.rigidBody) bulletWorld->removeRigidBody(torsoPhysics.rigidBody);
    if (headPhysics.rigidBody) bulletWorld->removeRigidBody(headPhysics.rigidBody);
    if (leftArmPhysics.rigidBody) bulletWorld->removeRigidBody(leftArmPhysics.rigidBody);
    if (rightArmPhysics.rigidBody) bulletWorld->removeRigidBody(rightArmPhysics.rigidBody);
    
    // Cleanup is handled by BodyPartPhysics destructors
}

void RobloxStylePlayer::updateVisualFromPhysics() {
    // Since only torso is dynamic and others are kinematic, 
    // position all body parts relative to the torso
    if (!torsoPhysics.rigidBody) return;
    
    // Get the torso position (the main dynamic body)
    btTransform torsoTransform;
    torsoPhysics.rigidBody->getMotionState()->getWorldTransform(torsoTransform);
    btVector3 torsoPos = torsoTransform.getOrigin();
    
    // Update torso visual
    torso.setPosition({torsoPos.x(), torsoPos.y(), torsoPos.z()});
    torso.setRotation({0, 1, 0}, 0);
    
    // Position other parts relative to torso (maintaining original offsets)
    Vector3 torsoCenter = {torsoPos.x(), torsoPos.y(), torsoPos.z()};
    
    // Head: above torso
    Vector3 headPos = {
        torsoCenter.x,
        torsoCenter.y + dimensions.torsoSize.y/2 + dimensions.headRadius,
        torsoCenter.z
    };
    head.setPosition(headPos);
    head.setRotation({0, 1, 0}, 0);
    
    // Arms: at shoulder level
    float armY = torsoCenter.y + dimensions.torsoSize.y/4;
    Vector3 leftArmPos = {
        torsoCenter.x - (dimensions.torsoSize.x/2 + dimensions.armSize.x/2),
        armY,
        torsoCenter.z
    };
    Vector3 rightArmPos = {
        torsoCenter.x + (dimensions.torsoSize.x/2 + dimensions.armSize.x/2),
        armY,
        torsoCenter.z
    };
    leftArm.setPosition(leftArmPos);
    rightArm.setPosition(rightArmPos);
    leftArm.setRotation({0, 1, 0}, 0);
    rightArm.setRotation({0, 1, 0}, 0);
    
    // Legs: below torso, but ensure they don't go through ground
    // Leg center should be at torso_bottom + leg_height/2 to have feet on ground
    float torsoBottom = torsoCenter.y - dimensions.torsoSize.y/2;
    Vector3 leftLegPos = {
        torsoCenter.x - 0.2f,
        torsoBottom + dimensions.legSize.y/2,  // Feet will be at torsoBottom level
        torsoCenter.z
    };
    Vector3 rightLegPos = {
        torsoCenter.x + 0.2f,
        torsoBottom + dimensions.legSize.y/2,  // Feet will be at torsoBottom level
        torsoCenter.z
    };
    
    // Check ground collision for legs - don't let feet go below ground
    if (world) {
        auto checkLegGroundCollision = [&](Vector3& legPos) {
            float legBottomY = legPos.y - dimensions.legSize.y/2;
            
            // Raycast from leg center downward to find ground
            btVector3 start(legPos.x, legPos.y, legPos.z);
            btVector3 end(legPos.x, legPos.y - dimensions.legSize.y, legPos.z);
            
            btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
            world->getBulletWorld()->rayTest(start, end, rayCallback);
            
            if (rayCallback.hasHit()) {
                float groundY = rayCallback.m_hitPointWorld.y();
                float minLegY = groundY + dimensions.legSize.y/2; // Leg center must be above ground
                
                if (legPos.y < minLegY) {
                    legPos.y = minLegY;
                    // Also adjust torso if needed to maintain body proportions
                    float requiredTorsoY = minLegY + dimensions.legSize.y/2 + dimensions.torsoSize.y/2;
                    if (torsoCenter.y < requiredTorsoY) {
                        // Gently push torso up to maintain body proportions
                        torsoCenter.y = requiredTorsoY;
                        // Apply upward force to physics body to maintain consistency
                        if (torsoPhysics.rigidBody) {
                            btTransform torsoTransform;
                            torsoTransform.setIdentity();
                            torsoTransform.setOrigin(btVector3(torsoCenter.x, torsoCenter.y, torsoCenter.z));
                            torsoPhysics.rigidBody->setWorldTransform(torsoTransform);
                        }
                    }
                }
            }
        };
        
        checkLegGroundCollision(leftLegPos);
        checkLegGroundCollision(rightLegPos);
    }
    
    leftLeg.setPosition(leftLegPos);
    rightLeg.setPosition(rightLegPos);
    leftLeg.setRotation({0, 1, 0}, 0);
    rightLeg.setRotation({0, 1, 0}, 0);
    
    // Now update the kinematic bodies to follow the visual positions
    auto updateKinematicBody = [](BodyPartPhysics& physics, const Vector3& position) {
        if (physics.rigidBody) {
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(position.x, position.y, position.z));
            physics.rigidBody->setWorldTransform(transform);
            physics.rigidBody->getMotionState()->setWorldTransform(transform);
        }
    };
    
    updateKinematicBody(headPhysics, headPos);
    updateKinematicBody(leftArmPhysics, leftArmPos);
    updateKinematicBody(rightArmPhysics, rightArmPos);
    updateKinematicBody(leftLegPhysics, leftLegPos);
    updateKinematicBody(rightLegPhysics, rightLegPos);
    
    // Update GameObject position to center of mass for compatibility
    Vector3 centerOfMass = {torsoPos.x(), torsoPos.y(), torsoPos.z()};
    this->position = centerOfMass;
}

Vector3 RobloxStylePlayer::getCurrentCenterOfMass() const {
    // Calculate center of mass from all body parts
    float totalMass = TORSO_MASS + HEAD_MASS + ARM_MASS * 2 + LEG_MASS * 2;
    Vector3 com = {0, 0, 0};
    
    auto addToCoM = [&](const BodyPartPhysics& physics, float mass) {
        if (physics.rigidBody) {
            btTransform transform;
            physics.rigidBody->getMotionState()->getWorldTransform(transform);
            btVector3 pos = transform.getOrigin();
            com.x += pos.x() * mass;
            com.y += pos.y() * mass;
            com.z += pos.z() * mass;
        }
    };
    
    addToCoM(torsoPhysics, TORSO_MASS);
    addToCoM(headPhysics, HEAD_MASS);
    addToCoM(leftArmPhysics, ARM_MASS);
    addToCoM(rightArmPhysics, ARM_MASS);
    addToCoM(leftLegPhysics, LEG_MASS);
    addToCoM(rightLegPhysics, LEG_MASS);
    
    com.x /= totalMass;
    com.y /= totalMass;
    com.z /= totalMass;
    
    return com;
}

Vector3 RobloxStylePlayer::getFeetPosition() const {
    // Get the lowest Y position from both legs
    float minY = 1000.0f;
    Vector3 feetPos = {0, 0, 0};
    int legCount = 0;
    
    auto checkLeg = [&](const BodyPartPhysics& legPhysics) {
        if (legPhysics.rigidBody) {
            btTransform transform;
            legPhysics.rigidBody->getMotionState()->getWorldTransform(transform);
            btVector3 pos = transform.getOrigin();
            
            // Bottom of leg capsule
            float legBottom = pos.y() - dimensions.legSize.y/2;
            if (legBottom < minY) {
                minY = legBottom;
            }
            
            feetPos.x += pos.x();
            feetPos.z += pos.z();
            legCount++;
        }
    };
    
    checkLeg(leftLegPhysics);
    checkLeg(rightLegPhysics);
    
    if (legCount > 0) {
        feetPos.x /= legCount;  // Average X position
        feetPos.y = minY;       // Lowest Y position
        feetPos.z /= legCount;  // Average Z position
    }
    
    return feetPos;
}

void RobloxStylePlayer::setFeetOnGround(Vector3 groundPosition) {
    // Move entire character so feet touch the specified ground position
    Vector3 currentFeet = getFeetPosition();
    Vector3 offset = {
        groundPosition.x - currentFeet.x,
        groundPosition.y - currentFeet.y,
        groundPosition.z - currentFeet.z
    };
    
    // Apply offset to all body parts
    auto moveBodyPart = [&](BodyPartPhysics& physics) {
        if (physics.rigidBody) {
            btTransform transform;
            physics.rigidBody->getMotionState()->getWorldTransform(transform);
            btVector3 pos = transform.getOrigin();
            pos += btVector3(offset.x, offset.y, offset.z);
            transform.setOrigin(pos);
            physics.rigidBody->setWorldTransform(transform);
            physics.rigidBody->getMotionState()->setWorldTransform(transform);
        }
    };
    
    moveBodyPart(torsoPhysics);
    moveBodyPart(headPhysics);
    moveBodyPart(leftArmPhysics);
    moveBodyPart(rightArmPhysics);
    moveBodyPart(leftLegPhysics);
    moveBodyPart(rightLegPhysics);
    
    printf("[DEBUG] Character repositioned: feet now at Y=%.2f\n", groundPosition.y);
}

bool RobloxStylePlayer::isOnGround() const {
    if (!world || !torsoPhysics.rigidBody) return false;
    
    // Use torso position for more reliable ground detection
    btTransform torsoTransform;
    torsoPhysics.rigidBody->getMotionState()->getWorldTransform(torsoTransform);
    btVector3 torsoPos = torsoTransform.getOrigin();
    
    // Calculate feet position: torso bottom = where feet should be with new leg positioning
    float feetY = torsoPos.y() - dimensions.torsoSize.y/2;
    
    // Start raycast from slightly above feet position
    btVector3 start = btVector3(torsoPos.x(), feetY + 0.05f, torsoPos.z());
    btVector3 end = btVector3(torsoPos.x(), feetY - 0.15f, torsoPos.z()); // 15cm below feet
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
    rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
    
    world->getBulletWorld()->rayTest(start, end, rayCallback);
    
    if (rayCallback.hasHit()) {
        float hitDistance = (rayCallback.m_hitPointWorld - start).length();
        
        // Check vertical velocity - not falling too fast
        btVector3 velocity = torsoPhysics.rigidBody->getLinearVelocity();
        bool isStable = std::abs(velocity.y()) < 2.0f; // Allow some vertical movement
        
        return hitDistance < 0.15f && isStable; // Within 15cm and stable
    }
    return false;
}

void RobloxStylePlayer::handleInput(float movementSpeed) {
    if (!torsoPhysics.rigidBody) return;
    
    Vector2 moveAxis = InputSystem::getMovementAxis();
    
    // Simplified debug output - only show when there's input
    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f || InputSystem::isJumpPressed()) {
        printf("[INPUT] Movement: (%.2f, %.2f), Jump: %s\n", 
               moveAxis.x, moveAxis.y, 
               InputSystem::isJumpPressed() ? "YES" : "NO");
        
        // Debug torso position and velocity
        btTransform torsoTransform;
        torsoPhysics.rigidBody->getMotionState()->getWorldTransform(torsoTransform);
        btVector3 torsoPos = torsoTransform.getOrigin();
        btVector3 velocity = torsoPhysics.rigidBody->getLinearVelocity();
        printf("[DEBUG] Torso position: (%.2f, %.2f, %.2f), velocity: (%.2f, %.2f, %.2f)\n",
               torsoPos.x(), torsoPos.y(), torsoPos.z(),
               velocity.x(), velocity.y(), velocity.z());
    }
    
    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f) {
        // Improved movement with better force calculation
        float forceMultiplier = MOVEMENT_FORCE; // Simplified - always use full force for testing
        
        // Apply movement force to torso (main body)
        btVector3 force(moveAxis.x * forceMultiplier, 0, -moveAxis.y * forceMultiplier);
        torsoPhysics.rigidBody->applyCentralForce(force);
        printf("[DEBUG] Applying force: (%.2f, %.2f, %.2f)\n", 
               force.x(), force.y(), force.z());
        
        // Simplified velocity limiting with higher max speed
        btVector3 velocity = torsoPhysics.rigidBody->getLinearVelocity();
        float maxSpeed = movementSpeed * 10.0f; // Increased max speed significantly
        
        // Clamp horizontal velocity
        if (std::abs(velocity.x()) > maxSpeed) {
            velocity.setX(velocity.x() > 0 ? maxSpeed : -maxSpeed);
        }
        if (std::abs(velocity.z()) > maxSpeed) {
            velocity.setZ(velocity.z() > 0 ? maxSpeed : -maxSpeed);
        }
        
        torsoPhysics.rigidBody->setLinearVelocity(velocity);
    } else if (isOnGround()) {
        // Apply friction when not moving and on ground
        btVector3 velocity = torsoPhysics.rigidBody->getLinearVelocity();
        velocity.setX(velocity.x() * 0.85f); // Damping
        velocity.setZ(velocity.z() * 0.85f);
        torsoPhysics.rigidBody->setLinearVelocity(velocity);
    }
    
    // Jump - only when on ground
    if (InputSystem::isJumpPressed()) {
        bool onGround = isOnGround();
        printf("[INPUT] Jump pressed! On ground: %s\n", onGround ? "YES" : "NO");
        if (onGround) {
            jump();
        }
    }
}

void RobloxStylePlayer::jump() {
    if (!torsoPhysics.rigidBody) return;
    
    // Apply jump impulse to main body
    btVector3 jumpImpulse(0, JUMP_IMPULSE, 0);
    torsoPhysics.rigidBody->applyCentralImpulse(jumpImpulse);
    
    // Reset any downward velocity to ensure clean jump
    btVector3 velocity = torsoPhysics.rigidBody->getLinearVelocity();
    if (velocity.y() < 0) {
        velocity.setY(0);
        torsoPhysics.rigidBody->setLinearVelocity(velocity);
    }
    
    printf("[JUMP] Character jumped! Applied impulse: %.2f\n", JUMP_IMPULSE);
}

// Individual collision detection for each body part
bool RobloxStylePlayer::checkHeadCollision() const {
    if (!headPhysics.rigidBody || !world) return false;
    
    btTransform transform;
    headPhysics.rigidBody->getMotionState()->getWorldTransform(transform);
    btVector3 headPos = transform.getOrigin();
    
    // Check collisions around head sphere
    float radius = dimensions.headRadius + 0.05f; // Slightly larger for detection
    btVector3 directions[] = {
        btVector3(radius, 0, 0), btVector3(-radius, 0, 0),
        btVector3(0, radius, 0), btVector3(0, -radius, 0),
        btVector3(0, 0, radius), btVector3(0, 0, -radius)
    };
    
    for (const auto& dir : directions) {
        btCollisionWorld::ClosestRayResultCallback rayCallback(headPos, headPos + dir);
        world->getBulletWorld()->rayTest(headPos, headPos + dir, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != headPhysics.rigidBody) {
            return true;
        }
    }
    return false;
}

bool RobloxStylePlayer::checkTorsoCollision() const {
    if (!torsoPhysics.rigidBody || !world) return false;
    
    btTransform transform;
    torsoPhysics.rigidBody->getMotionState()->getWorldTransform(transform);
    btVector3 torsoPos = transform.getOrigin();
    
    // Check collisions around torso box
    Vector3 halfSize = {dimensions.torsoSize.x/2 + 0.05f, 
                       dimensions.torsoSize.y/2 + 0.05f, 
                       dimensions.torsoSize.z/2 + 0.05f};
    
    btVector3 directions[] = {
        btVector3(halfSize.x, 0, 0), btVector3(-halfSize.x, 0, 0),
        btVector3(0, 0, halfSize.z), btVector3(0, 0, -halfSize.z)
    };
    
    for (const auto& dir : directions) {
        btCollisionWorld::ClosestRayResultCallback rayCallback(torsoPos, torsoPos + dir);
        world->getBulletWorld()->rayTest(torsoPos, torsoPos + dir, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != torsoPhysics.rigidBody) {
            return true;
        }
    }
    return false;
}

bool RobloxStylePlayer::checkArmCollision(bool isLeft) const {
    const BodyPartPhysics& armPhysics = isLeft ? leftArmPhysics : rightArmPhysics;
    if (!armPhysics.rigidBody || !world) return false;
    
    btTransform transform;
    armPhysics.rigidBody->getMotionState()->getWorldTransform(transform);
    btVector3 armPos = transform.getOrigin();
    
    // Check collisions around arm
    Vector3 halfSize = {dimensions.armSize.x/2 + 0.02f, 
                       dimensions.armSize.y/2 + 0.02f, 
                       dimensions.armSize.z/2 + 0.02f};
    
    btVector3 directions[] = {
        btVector3(isLeft ? -halfSize.x : halfSize.x, 0, 0), // Outward
        btVector3(0, halfSize.y, 0), btVector3(0, -halfSize.y, 0),
        btVector3(0, 0, halfSize.z), btVector3(0, 0, -halfSize.z)
    };
    
    for (const auto& dir : directions) {
        btCollisionWorld::ClosestRayResultCallback rayCallback(armPos, armPos + dir);
        world->getBulletWorld()->rayTest(armPos, armPos + dir, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != armPhysics.rigidBody) {
            return true;
        }
    }
    return false;
}

bool RobloxStylePlayer::checkLegCollision(bool isLeft) const {
    const BodyPartPhysics& legPhysics = isLeft ? leftLegPhysics : rightLegPhysics;
    if (!legPhysics.rigidBody || !world) return false;
    
    btTransform transform;
    legPhysics.rigidBody->getMotionState()->getWorldTransform(transform);
    btVector3 legPos = transform.getOrigin();
    
    // Check collisions around leg
    Vector3 halfSize = {dimensions.legSize.x/2 + 0.02f, 
                       dimensions.legSize.y/2 + 0.02f, 
                       dimensions.legSize.z/2 + 0.02f};
    
    btVector3 directions[] = {
        btVector3(isLeft ? -halfSize.x : halfSize.x, 0, 0), // Outward
        btVector3(0, 0, halfSize.z), btVector3(0, 0, -halfSize.z)
    };
    
    for (const auto& dir : directions) {
        btCollisionWorld::ClosestRayResultCallback rayCallback(legPos, legPos + dir);
        world->getBulletWorld()->rayTest(legPos, legPos + dir, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != legPhysics.rigidBody) {
            return true;
        }
    }
    return false;
}

void RobloxStylePlayer::animateLimbs(float deltaTime) {
    Vector2 moveInput = InputSystem::getMovementAxis();
    bool isMoving = (moveInput.x != 0.0f || moveInput.y != 0.0f);
    
    if (isMoving) {
        animationTime += deltaTime * 5.0f; // Animation speed
        
        float armSwing = sin(animationTime) * 15.0f; // 15 degree swing
        float legSwing = sin(animationTime) * 20.0f; // 20 degree swing
        
        // Apply subtle animation to visual parts
        leftArm.setRotation({1, 0, 0}, armSwing);
        rightArm.setRotation({1, 0, 0}, -armSwing);
        leftLeg.setRotation({1, 0, 0}, -legSwing);
        rightLeg.setRotation({1, 0, 0}, legSwing);
    } else {
        // Return to neutral position when not moving
        const float returnSpeed = 5.0f;
        
        auto returnToNeutral = [&](BodyPart& part) {
            float currentAngle = part.getRotationAngle();
            if (std::abs(currentAngle) > 0.1f) {
                float newAngle = Lerp(currentAngle, 0.0f, returnSpeed * deltaTime);
                part.setRotation({1, 0, 0}, newAngle);
            } else {
                part.setRotation({1, 0, 0}, 0.0f);
            }
        };
        
        returnToNeutral(leftArm);
        returnToNeutral(rightArm);
        returnToNeutral(leftLeg);
        returnToNeutral(rightLeg);
    }
    
    wasMovingLastFrame = isMoving;
}

void RobloxStylePlayer::update(float deltaTime) {
    // Update visual components from physics (CRITICAL for visibility)
    updateVisualFromPhysics();
    
    // Animate limbs
    animateLimbs(deltaTime);
    
    // Debug output
    static float debugTimer = 0;
    debugTimer += deltaTime;
    if (debugTimer > 1.0f) { // Every second
        Vector3 feet = getFeetPosition();
        Vector3 com = getCurrentCenterOfMass();
        
        // Additional debug for ground detection
        bool onGround = isOnGround();
        if (!onGround) {
            // Debug why we're not detecting ground
            btTransform leftTransform, rightTransform;
            leftLegPhysics.rigidBody->getMotionState()->getWorldTransform(leftTransform);
            rightLegPhysics.rigidBody->getMotionState()->getWorldTransform(rightTransform);
            
            btVector3 leftLegPos = leftTransform.getOrigin();
            btVector3 rightLegPos = rightTransform.getOrigin();
            btVector3 leftVel = leftLegPhysics.rigidBody->getLinearVelocity();
            btVector3 rightVel = rightLegPhysics.rigidBody->getLinearVelocity();
            
            printf("[DEBUG] Ground Detection Debug:\n");
            printf("  Left Leg: Pos Y=%.2f, Vel Y=%.2f\n", leftLegPos.y(), leftVel.y());
            printf("  Right Leg: Pos Y=%.2f, Vel Y=%.2f\n", rightLegPos.y(), rightVel.y());
            printf("  Feet bottom: Y=%.2f (should be ~0.50 for floor contact)\n", leftLegPos.y() - dimensions.legSize.y/2);
        }
        
        printf("[DEBUG] Feet: Y=%.2f, Center of Mass: Y=%.2f, On Ground: %s\n", 
               feet.y, com.y, onGround ? "YES" : "NO");
        debugTimer = 0;
    }
}

void RobloxStylePlayer::draw() const {
    // Draw all body parts
    torso.draw();
    head.draw();
    leftArm.draw();
    rightArm.draw();
    leftLeg.draw();
    rightLeg.draw();
    
    // Debug: Draw a sphere at character center for testing
    Vector3 com = getCurrentCenterOfMass();
    DrawSphere(com, 0.1f, PURPLE);  // Small purple sphere to show character position
}

BoundingBox RobloxStylePlayer::getBoundingBox() const {
    // Calculate bounding box from all visible parts
    BoundingBox box = torso.getBoundingBox();
    
    auto expandBox = [&](const BodyPart& part) {
        BoundingBox partBox = part.getBoundingBox();
        box.min = Vector3Min(box.min, partBox.min);
        box.max = Vector3Max(box.max, partBox.max);
    };
    
    expandBox(head);
    expandBox(leftArm);
    expandBox(rightArm);
    expandBox(leftLeg);
    expandBox(rightLeg);
    
    return box;
}

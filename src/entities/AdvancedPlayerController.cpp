// AdvancedPlayerController.cpp - Professional Character Controller using Bullet Physics
#include "entities/AdvancedPlayerController.h"
#include "systems/InputSystem.h"
#include <cmath>
#include "raymath.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

namespace {
constexpr float ADV_PLAYER_CAPSULE_RADIUS = 0.3f;
constexpr float ADV_PLAYER_CAPSULE_HEIGHT = 1.4f;
constexpr float ADV_PLAYER_CAPSULE_HALF_HEIGHT = ADV_PLAYER_CAPSULE_HEIGHT / 2.0f;
// Proper offset from ground to center - this is exactly half the capsule height
constexpr float GROUND_TO_CENTER_OFFSET = ADV_PLAYER_CAPSULE_HALF_HEIGHT; // 0.70f - exact physics requirement
constexpr float GRAVITY = -9.81f;
constexpr float JUMP_VELOCITY = 6.0f;
constexpr float STEP_HEIGHT = 0.1f; // Reduced step height to prevent unwanted stepping down
constexpr float MAX_SLOPE = 45.0f; // Maximum slope angle in degrees
}

AdvancedPlayerController::AdvancedPlayerController(btDiscreteDynamicsWorld* dynamicsWorld)
    : world(dynamicsWorld), characterController(nullptr), ghostObject(nullptr), 
      position{0,0,0}, velocity{0,0,0}, onGround(false) {
    
    // Create capsule shape for character
    convexShape = new btCapsuleShape(ADV_PLAYER_CAPSULE_RADIUS, ADV_PLAYER_CAPSULE_HEIGHT);
    
    // Create ghost object for character controller
    ghostObject = new btPairCachingGhostObject();
    ghostObject->setCollisionShape(convexShape);
    ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
    
    // Set initial position - player base at ground level (Y=0.50)
    position = {0, 0.50f, 0}; // Base exactly on floor surface
    btTransform startTransform;
    startTransform.setIdentity();
    // Floor collision box extends from Y=-0.5 to Y=+0.5, so top surface is at Y=0.5
    // Character center should be at Y=1.2 (0.5 + 0.7) for feet on floor surface
    float floorTopSurface = 0.50f; // Top of floor collision box
    startTransform.setOrigin(btVector3(position.x, floorTopSurface + GROUND_TO_CENTER_OFFSET, position.z));
    ghostObject->setWorldTransform(startTransform);
    
    // Create kinematic character controller - this is the professional way
    characterController = new btKinematicCharacterController(
        ghostObject, 
        convexShape, 
        STEP_HEIGHT,  // Maximum step height (35cm)
        btVector3(0, 1, 0)  // Up vector
    );
    
    // Configure character controller for proper ground behavior
    characterController->setMaxSlope(btRadians(MAX_SLOPE)); // 45 degree max slope
    characterController->setGravity(btVector3(0, GRAVITY, 0));
    characterController->setMaxJumpHeight(2.0f); // Maximum jump height
    characterController->setJumpSpeed(JUMP_VELOCITY);
    characterController->setFallSpeed(15.0f); // Terminal velocity
    
    // Additional settings for better ground contact
    characterController->setMaxPenetrationDepth(0.2f); // Allow some penetration for stable contact
    characterController->setUseGhostSweepTest(false); // Use more accurate collision detection
    
    // Add to physics world with proper collision groups
    world->addCollisionObject(ghostObject, btBroadphaseProxy::CharacterFilter, 
                             btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
    world->addAction(characterController);
    
    printf("[DEBUG] Professional Character Controller created!\n");
    printf("[DEBUG] Ground level: %.2f, Character center: %.2f, Offset: %.2f\n", 
           position.y, position.y + GROUND_TO_CENTER_OFFSET, GROUND_TO_CENTER_OFFSET);
    printf("[DEBUG] Character should have feet exactly on ground at Y=%.2f\n", position.y);
    
    // Force an immediate update to make sure the character controller settles
    world->stepSimulation(1.0f/60.0f, 1);
    printf("[DEBUG] Character controller initialized and stepped once\n");
}

AdvancedPlayerController::~AdvancedPlayerController() {
    if (characterController) {
        world->removeAction(characterController);
        delete characterController;
    }
    if (ghostObject) {
        world->removeCollisionObject(ghostObject);
        delete ghostObject;
    }
    delete convexShape;
}

void AdvancedPlayerController::setPosition(const Vector3& pos) {
    position = pos;
    if (ghostObject && characterController) {
        btTransform t;
        t.setIdentity();
        // Position ghost object center at ground + half height for proper character behavior
        t.setOrigin(btVector3(pos.x, pos.y + GROUND_TO_CENTER_OFFSET, pos.z));
        ghostObject->setWorldTransform(t);
        characterController->warp(btVector3(pos.x, pos.y + GROUND_TO_CENTER_OFFSET, pos.z));
    }
}

void AdvancedPlayerController::syncPlayerVisual(Player& player) {
    // Position represents the ground level where player's feet should be
    player.setPosition(position);
}

void AdvancedPlayerController::update(float deltaTime) {
    if (!characterController || !ghostObject) return;
    
    // Get current position from character controller
    btTransform trans = ghostObject->getWorldTransform();
    btVector3 currentCenter = trans.getOrigin();
    
    // Calculate ground position (character's feet position)
    Vector3 newPosition = {
        currentCenter.x(), 
        currentCenter.y() - GROUND_TO_CENTER_OFFSET, // Subtract half height to get ground level
        currentCenter.z()
    };
    
    position = newPosition;
    
    // Update ground contact status
    onGround = characterController->onGround();
    
    // Debug output for first few updates
    static int updateCount = 0;
    updateCount++;
    if (updateCount <= 5) {
        printf("[DEBUG] Professional Controller Update %d:\n", updateCount);
        printf("        Character Center: (%.3f, %.3f, %.3f)\n", 
               currentCenter.x(), currentCenter.y(), currentCenter.z());
        printf("        Ground Position:  (%.3f, %.3f, %.3f)\n", 
               newPosition.x, newPosition.y, newPosition.z);
        printf("        On Ground: %s\n", onGround ? "YES" : "NO");
    }
    
    // Periodic debug output every 3 seconds
    static int debugCounter = 0;
    debugCounter++;
    if (debugCounter % 180 == 0) {
        printf("[DEBUG] Character Status - Ground Pos: (%.3f, %.3f, %.3f), Center: (%.3f, %.3f, %.3f), OnGround: %s\n",
               newPosition.x, newPosition.y, newPosition.z, 
               currentCenter.x(), currentCenter.y(), currentCenter.z(), 
               onGround ? "YES" : "NO");
    }
}

void AdvancedPlayerController::handleInput(float movementSpeed) {
    if (!characterController) return;
    
    // Handle movement input
    Vector2 moveAxis = InputSystem::getMovementAxis();
    
    // Calculate movement direction
    Vector3 moveDirection = {moveAxis.x, 0, -moveAxis.y}; // -y because forward is negative Z
    
    if (Vector3Length(moveDirection) > 0.01f) {
        moveDirection = Vector3Normalize(moveDirection);
        
        // Use character controller's setWalkDirection for proper movement
        btVector3 walkDirection(
            moveDirection.x * movementSpeed,
            0, // Don't override gravity with horizontal movement
            moveDirection.z * movementSpeed
        );
        characterController->setWalkDirection(walkDirection);
    } else {
        // Stop movement when no input
        characterController->setWalkDirection(btVector3(0, 0, 0));
    }
    
    // Handle jump input
    if (InputSystem::isJumpPressed() && onGround) {
        printf("[DEBUG] JUMP! OnGround: %s\n", onGround ? "YES" : "NO");
        characterController->jump(btVector3(0, 1, 0)); // Jump with normalized up vector
    }
}

void AdvancedPlayerController::updateGhostObjects() {
    // Not needed with btKinematicCharacterController - it handles this automatically
}

void AdvancedPlayerController::performGroundCheck() {
    // Not needed with btKinematicCharacterController - it has built-in ground detection
    // onGround status is updated automatically in the update() method
}

AdvancedPlayerController::BodyPartCollider AdvancedPlayerController::createBodyPartCollider(const Vector3& size, const Vector3& offset) {
    BodyPartCollider collider;
    collider.shape = new btCapsuleShape(size.x, size.y);
    collider.ghost = new btGhostObject();
    collider.ghost->setCollisionShape(collider.shape);
    collider.ghost->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    collider.localOffset = offset;
    world->addCollisionObject(collider.ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
    return collider;
}

bool AdvancedPlayerController::checkHeadCollision() const {
    if (!ghostObject) return false;
    
    // Get current position
    btTransform t = ghostObject->getWorldTransform();
    btVector3 center = t.getOrigin();
    
    // Cast a ray upward from the top of the capsule to check for ceiling/overhead obstacles
    btVector3 top = center + btVector3(0, ADV_PLAYER_CAPSULE_HALF_HEIGHT + 0.1f, 0);
    btVector3 rayEnd = top + btVector3(0, 0.3f, 0); // Check 30cm above head
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(top, rayEnd);
    world->rayTest(top, rayEnd, rayCallback);
    
    return rayCallback.hasHit();
}
bool AdvancedPlayerController::checkArmCollision(bool isLeft) const { return false; }
bool AdvancedPlayerController::checkLegCollision(bool isLeft) const { return false; }

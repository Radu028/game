// HumanoidCharacter.h - Solid humanoid character controller (no ragdoll)
#pragma once

#include "raylib.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "btBulletDynamicsCommon.h"
#include "settings/Physics.h"
#include <vector>
#include <memory>

class GameWorld;

// Character states for animation
enum HumanoidState {
    IDLE,
    WALKING,
    JUMPING
};

// Visual part for rendering only (no individual physics)
struct HumanoidVisualPart {
    BodyPart visual;
    Vector3 baseOffset;  // Offset from character center
    Vector3 currentOffset;  // Current animated offset
    
    HumanoidVisualPart(Vector3 size, Color color, Vector3 offset) 
        : visual({0,0,0}, size, color, true), baseOffset(offset), currentOffset(offset) {}
};

// Facial feature for eyes and expressions
struct FacialFeature {
    BodyPart visual;
    Vector3 baseOffset;
    Vector3 currentOffset;
    
    FacialFeature(Vector3 size, Color color, Vector3 offset)
        : visual({0,0,0}, size, color, true), baseOffset(offset), currentOffset(offset) {}
};

class HumanoidCharacter : public GameObject {
public:
    HumanoidCharacter(Vector3 position);
    ~HumanoidCharacter();
    
    void setupPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void update(float deltaTime) override;
    void draw() const override;
    void drawCollisionBoxes() const; // For debugging individual part collisions
    BoundingBox getBoundingBox() const override;
    
    // Movement and control
    void handleInput(float movementSpeed);
    void jump();
    
    // Ground detection
    bool isOnGround() const;
    
    // Position methods
    Vector3 getFeetPosition() const;
    Vector3 getTorsoPosition() const;
    
    // Collision detection for individual body parts
    bool checkPartCollision(const HumanoidVisualPart& part, const BoundingBox& obstacle) const;
    bool checkAnyPartCollision(const BoundingBox& obstacle) const;
    bool wouldCollideAfterMovement(Vector3 movement, float deltaTime) const;
    Vector3 getPartWorldPosition(const HumanoidVisualPart& part) const;
    BoundingBox getPartBoundingBox(const HumanoidVisualPart& part) const;
    
    void setWorld(GameWorld* w) { world = w; }

private:
    // Main physics body for movement
    btRigidBody* characterBody = nullptr;
    btCollisionShape* characterShape = nullptr;
    btDefaultMotionState* motionState = nullptr;
    
    // Physics world reference
    btDiscreteDynamicsWorld* physicsWorld = nullptr;
    
    // Body parts (visual only)
    HumanoidVisualPart head;
    HumanoidVisualPart torso;
    HumanoidVisualPart leftArm;
    HumanoidVisualPart rightArm;
    HumanoidVisualPart leftLeg;
    HumanoidVisualPart rightLeg;
    
    // Facial features (visual only)
    FacialFeature leftEye;
    FacialFeature rightEye;
    FacialFeature mouth;
    
    GameWorld* world = nullptr;
    
    // Character state and animation
    HumanoidState currentState = IDLE;
    float animationTime = 0.0f;
    bool isJumping = false;
    float jumpCooldown = 0.0f;
    
    // Movement state
    Vector3 targetMovementDirection = {0, 0, 0};
    float currentMovementSpeed = 0.0f;
    
    // Private methods
    void createSinglePhysicsBody();
    void updateVisualPositions();
    void animateCharacter(float deltaTime);
    void applyMovementForces(Vector3 movement, float speed);
    void updateCharacterState();
};

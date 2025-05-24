// SimpleRagdoll.h - Simple character with single physics body and visual parts
#pragma once

#include "raylib.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "btBulletDynamicsCommon.h"
#include <memory>

class GameWorld;

class SimpleRagdoll : public GameObject {
public:
    SimpleRagdoll(Vector3 position);
    ~SimpleRagdoll();
    
    void setupPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void update(float deltaTime) override;
    void draw() const override;
    BoundingBox getBoundingBox() const override;
    
    // Movement and control
    void handleInput(float movementSpeed);
    void jump();
    
    // Ground detection
    bool isOnGround() const;
    bool isOnGroundContact() const;  // Alternative ground detection method
    
    // Position methods
    Vector3 getFeetPosition() const;
    
    void setWorld(GameWorld* w) { world = w; }

private:
    // Visual components (for display only)
    BodyPart torso;
    BodyPart head;
    BodyPart leftArm;
    BodyPart rightArm;
    BodyPart leftLeg;
    BodyPart rightLeg;
    
    // Single physics body (capsule for character controller)
    btRigidBody* physicsBody = nullptr;
    btCollisionShape* physicsShape = nullptr;
    btDefaultMotionState* motionState = nullptr;
    
    GameWorld* world = nullptr;
    
    // Character dimensions
    static constexpr float CHARACTER_HEIGHT = 1.8f;
    static constexpr float CHARACTER_RADIUS = 0.3f;
    static constexpr float CHARACTER_MASS = 70.0f;
    
    // Movement settings - Increased for better responsiveness
    static constexpr float MOVEMENT_FORCE = 1500.0f;   // Increased from 500
    static constexpr float JUMP_IMPULSE = 50.0f;       // MASSIVELY increased for testing - was 18.0f
    static constexpr float MAX_SPEED = 8.0f;           // Increased from 5
    static constexpr float DAMPING_LINEAR = 0.001f;    // Very low damping to reduce Y velocity suppression  
    static constexpr float DAMPING_ANGULAR = 0.8f;
    
    // Animation
    float animationTime = 0.0f;
    bool wasMovingLastFrame = false;
    
    // Private methods
    void updateVisualFromPhysics();
    void animateLimbs(float deltaTime);
    Vector3 getPhysicsPosition() const;
};

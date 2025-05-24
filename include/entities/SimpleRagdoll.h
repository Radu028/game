// SimpleRagdoll.h - Simple character with single physics body and visual parts
#pragma once

#include "raylib.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "btBulletDynamicsCommon.h"
#include "settings/Physics.h"
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
    
    // Animation
    float animationTime = 0.0f;
    bool wasMovingLastFrame = false;
    
    // Private methods
    void updateVisualFromPhysics();
    void animateLimbs(float deltaTime);
    Vector3 getPhysicsPosition() const;
};

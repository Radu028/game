// RobloxStylePlayer.h - True multi-part character with separate collision bodies
#pragma once

#include "raylib.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "btBulletDynamicsCommon.h"
#include <memory>
#include <vector>

class GameWorld;

// Individual body part physics component
struct BodyPartPhysics {
    btRigidBody* rigidBody = nullptr;
    btCollisionShape* shape = nullptr;
    btDefaultMotionState* motionState = nullptr;
    
    ~BodyPartPhysics() {
        cleanup();
    }
    
    void cleanup() {
        if (rigidBody) {
            delete rigidBody;
            rigidBody = nullptr;
        }
        if (motionState) {
            delete motionState;
            motionState = nullptr;
        }
        if (shape) {
            delete shape;
            shape = nullptr;
        }
    }
};

// Constraint between body parts
struct BodyPartConstraint {
    btTypedConstraint* constraint = nullptr;
    
    ~BodyPartConstraint() {
        if (constraint) {
            delete constraint;
        }
    }
};

class RobloxStylePlayer : public GameObject {
public:
    RobloxStylePlayer(Vector3 position);
    ~RobloxStylePlayer();
    
    void setupPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void removeFromPhysics(btDiscreteDynamicsWorld* bulletWorld);
    void update(float deltaTime) override;
    void draw() const override;
    BoundingBox getBoundingBox() const override;
    
    // Movement and control
    void handleInput(float movementSpeed);
    void jump();
    
    // Ground detection - uses feet position
    bool isOnGround() const;
    
    // Individual body part collision detection
    bool checkHeadCollision() const;
    bool checkTorsoCollision() const;
    bool checkArmCollision(bool isLeft) const;
    bool checkLegCollision(bool isLeft) const;
    
    // Get current feet position (lowest point of character)
    Vector3 getFeetPosition() const;
    
    // Set position based on feet touching ground
    void setFeetOnGround(Vector3 groundPosition);
    
    void setWorld(GameWorld* w) { world = w; }

private:
    // Visual components (Roblox-style body parts)
    BodyPart torso;
    BodyPart head;
    BodyPart leftArm;
    BodyPart rightArm;
    BodyPart leftLeg;
    BodyPart rightLeg;
    
    // Physics components for each body part
    BodyPartPhysics torsoPhysics;
    BodyPartPhysics headPhysics;
    BodyPartPhysics leftArmPhysics;
    BodyPartPhysics rightArmPhysics;
    BodyPartPhysics leftLegPhysics;
    BodyPartPhysics rightLegPhysics;
    
    // Constraints to keep body parts together
    std::vector<std::unique_ptr<BodyPartConstraint>> constraints;
    
    GameWorld* world = nullptr;
    
    // Body part dimensions (customizable for different character sizes)
    struct BodyPartDimensions {
        Vector3 torsoSize = {0.8f, 1.2f, 0.4f};
        float headRadius = 0.4f;
        Vector3 armSize = {0.3f, 0.8f, 0.3f};
        Vector3 legSize = {0.3f, 0.8f, 0.3f};
    } dimensions;
    
    // Physics settings
    static constexpr float BODY_MASS = 2.0f;        // Per body part
    static constexpr float HEAD_MASS = 1.0f;
    static constexpr float ARM_MASS = 1.0f;
    static constexpr float LEG_MASS = 1.5f;
    static constexpr float TORSO_MASS = 3.0f;
    
    static constexpr float MOVEMENT_FORCE = 500.0f;  // Increased from 50.0f for proper movement
    static constexpr float JUMP_IMPULSE = 8.0f;
    static constexpr float DAMPING_LINEAR = 0.1f;   // Reduced from 0.3f for better responsiveness
    static constexpr float DAMPING_ANGULAR = 0.8f;
    
    // Private methods
    void createBodyPartPhysics(BodyPartPhysics& physics, const Vector3& size, 
                              const Vector3& position, float mass, bool isSphere = false);
    void createConstraints(btDiscreteDynamicsWorld* world);
    void updateVisualFromPhysics();
    void calculateFeetPosition();
    Vector3 getCurrentCenterOfMass() const;
    
    // Animation
    void animateLimbs(float deltaTime);
    float animationTime = 0.0f;
    bool wasMovingLastFrame = false;
};

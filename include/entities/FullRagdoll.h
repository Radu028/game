// FullRagdoll.h - Professional ragdoll with separate physics bodies for each limb
#pragma once

#include "raylib.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "btBulletDynamicsCommon.h"
#include "settings/Physics.h"
#include <vector>
#include <memory>

class GameWorld;

// Collision groups for filtering
enum CollisionGroups {
    COL_NOTHING = 0,
    COL_GROUND = 1,
    COL_OBJECTS = 2,
    COL_RAGDOLL_HEAD = 4,
    COL_RAGDOLL_TORSO = 8,
    COL_RAGDOLL_ARMS = 16,
    COL_RAGDOLL_LEGS = 32,
    COL_ALL = -1
};

// What each part can collide with (everything except other ragdoll parts)
const int HEAD_COLLIDES_WITH = COL_GROUND | COL_OBJECTS;
const int TORSO_COLLIDES_WITH = COL_GROUND | COL_OBJECTS;
const int ARMS_COLLIDE_WITH = COL_GROUND | COL_OBJECTS;
const int LEGS_COLLIDE_WITH = COL_GROUND | COL_OBJECTS;

struct RagdollPart {
    BodyPart visual;
    btRigidBody* body = nullptr;
    btCollisionShape* shape = nullptr;
    btDefaultMotionState* motionState = nullptr;
    
    RagdollPart(Vector3 size, Color color) : visual({0,0,0}, size, color, true) {}
};

class FullRagdoll : public GameObject {
public:
    FullRagdoll(Vector3 position);
    ~FullRagdoll();
    
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
    Vector3 getTorsoPosition() const;
    
    void setWorld(GameWorld* w) { world = w; }

private:
    // Ragdoll parts with individual physics bodies
    RagdollPart head;
    RagdollPart torso;
    RagdollPart leftArm;
    RagdollPart rightArm;
    RagdollPart leftLeg;
    RagdollPart rightLeg;
    
    // Constraints to keep body parts together
    std::vector<btTypedConstraint*> constraints;
    
    GameWorld* world = nullptr;
    btDiscreteDynamicsWorld* physicsWorld = nullptr;
    
    // Animation and control
    float animationTime = 0.0f;
    bool wasMovingLastFrame = false;
    
    // Private methods
    void createRagdollPart(RagdollPart& part, Vector3 position, Vector3 size, 
                          int collisionGroup, int collidesWith, float mass = 1.0f);
    void createConstraints();
    void updateVisualFromPhysics();
    void animateLimbs(float deltaTime);
    void applyMovementForces(Vector3 movement, float speed);
    void applyStabilizationForces(); // NEW: Keep character upright
    btRigidBody* getMainBody() const { return torso.body; } // Torso is the main control body
};

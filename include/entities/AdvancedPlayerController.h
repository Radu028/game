// Professional Character Controller using Bullet Physics btKinematicCharacterController
#pragma once

#include "entities/Player.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>

class AdvancedPlayerController {
private:
    // Professional character controller components
    btKinematicCharacterController* characterController;
    btPairCachingGhostObject* ghostObject;
    btConvexShape* convexShape;
    btDiscreteDynamicsWorld* world;
    
    // Collision detection for body parts (optional for advanced features)
    struct BodyPartCollider {
        btGhostObject* ghost;
        btConvexShape* shape;
        Vector3 localOffset;
    };
    
    BodyPartCollider headCollider;
    BodyPartCollider leftArmCollider;
    BodyPartCollider rightArmCollider;
    BodyPartCollider leftLegCollider;
    BodyPartCollider rightLegCollider;
    
    Vector3 position;
    Vector3 velocity;
    bool onGround;
    
public:
    AdvancedPlayerController(btDiscreteDynamicsWorld* dynamicsWorld);
    ~AdvancedPlayerController();

    void setPosition(const Vector3& pos);
    Vector3 getPosition() const { return position; }

    void setVelocity(const Vector3& vel) { velocity = vel; }
    Vector3 getVelocity() const { return velocity; }

    bool isOnGround() const { return onGround; }

    // Collision detection for individual body parts
    bool checkHeadCollision() const;
    bool checkArmCollision(bool isLeft) const;
    bool checkLegCollision(bool isLeft) const;

    void update(float deltaTime);
    void handleInput(float movementSpeed);

    // Synchronize visual player position with character controller (feet on ground)
    void syncPlayerVisual(Player& player);

private:
    void updateGhostObjects();
    void performGroundCheck();

    BodyPartCollider createBodyPartCollider(const Vector3& size, const Vector3& offset);
};

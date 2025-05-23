// Exemplu de implementare cu Character Controller avansat
#pragma once

#include "entities/Player.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

class AdvancedPlayerController {
private:
    btGhostObject* ghostObject;
    btConvexShape* convexShape;
    btDiscreteDynamicsWorld* world;
    
    // Coliziuni separate pentru fiecare membru
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

    // Detectează coliziuni pentru fiecare membru individual
    bool checkHeadCollision() const;
    bool checkArmCollision(bool isLeft) const;
    bool checkLegCollision(bool isLeft) const;

    void update(float deltaTime);
    void handleInput(float movementSpeed);

    // Sincronizează poziția vizuală a playerului cu ghostObject-ul (baza la sol)
    void syncPlayerVisual(Player& player);

private:
    void updateGhostObjects();
    void performGroundCheck();
    void resolveCollisions();

    BodyPartCollider createBodyPartCollider(const Vector3& size, const Vector3& offset);
};

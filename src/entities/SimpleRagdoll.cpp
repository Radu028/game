// SimpleRagdoll.cpp - Simple character with single physics body
#include "entities/SimpleRagdoll.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raymath.h"
#include <cmath>

SimpleRagdoll::SimpleRagdoll(Vector3 position)
    : GameObject(position, true, true, false),
      torso({0, 0, 0}, {0.6f, 1.0f, 0.3f}, BLUE, false),
      head({0, 0, 0}, {0.4f, 0.4f, 0.4f}, RED, false),
      leftArm({0, 0, 0}, {0.2f, 0.6f, 0.2f}, GREEN, false),
      rightArm({0, 0, 0}, {0.2f, 0.6f, 0.2f}, GREEN, false),
      leftLeg({0, 0, 0}, {0.25f, 0.8f, 0.25f}, YELLOW, false),
      rightLeg({0, 0, 0}, {0.25f, 0.8f, 0.25f}, YELLOW, false) {
}

SimpleRagdoll::~SimpleRagdoll() {
}

void SimpleRagdoll::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    physicsShape = new btCapsuleShape(CHARACTER_RADIUS, CHARACTER_HEIGHT - 2*CHARACTER_RADIUS);
    physicsShape->setMargin(0.1f);
    float halfHeight = CHARACTER_HEIGHT * 0.5f;
    Vector3 physicsPos = this->position;
    float calculatedY = 0.50f + halfHeight + 0.05f;
    physicsPos.y = calculatedY;
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(physicsPos.x, physicsPos.y, physicsPos.z));
    motionState = new btDefaultMotionState(transform);
    btVector3 inertia(0, 0, 0);
    physicsShape->calculateLocalInertia(CHARACTER_MASS, inertia);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(CHARACTER_MASS, motionState, physicsShape, inertia);
    rbInfo.m_friction = 0.1f;
    rbInfo.m_rollingFriction = 0.0f;
    rbInfo.m_restitution = 0.0f;
    physicsBody = new btRigidBody(rbInfo);
    physicsBody->setDamping(DAMPING_LINEAR, DAMPING_ANGULAR);
    physicsBody->setActivationState(DISABLE_DEACTIVATION);
    physicsBody->setAngularFactor(btVector3(0, 1, 0));
    physicsBody->setLinearFactor(btVector3(1, 1, 1));
    physicsBody->setContactProcessingThreshold(0.0f);
    bulletWorld->addRigidBody(physicsBody);
    btTransform finalTransform;
    finalTransform.setIdentity();
    finalTransform.setOrigin(btVector3(physicsPos.x, physicsPos.y, physicsPos.z));
    physicsBody->setWorldTransform(finalTransform);
    physicsBody->getMotionState()->setWorldTransform(finalTransform);
    physicsBody->activate(true);
    physicsBody->setLinearVelocity(btVector3(0, 0, 0));
    physicsBody->setAngularVelocity(btVector3(0, 0, 0));
    updateVisualFromPhysics();
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
    
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 physicsPos = transform.getOrigin();
    btQuaternion rotation = transform.getRotation();
    
    Vector3 centerPos = {physicsPos.x(), physicsPos.y(), physicsPos.z()};
    
    float quarterHeight = CHARACTER_HEIGHT * 0.25f;
    float halfHeight = CHARACTER_HEIGHT * 0.5f;
    
    Vector3 headPos = {
        centerPos.x,
        centerPos.y + halfHeight - 0.2f,
        centerPos.z
    };
    
    Vector3 torsoPos = centerPos;
    
    float shoulderY = centerPos.y + quarterHeight;
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
    
    float legY = centerPos.y - quarterHeight;
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
    
    head.setPosition(headPos);
    torso.setPosition(torsoPos);
    leftArm.setPosition(leftArmPos);
    rightArm.setPosition(rightArmPos);
    leftLeg.setPosition(leftLegPos);
    rightLeg.setPosition(rightLegPos);
    
    float yaw = 0.0f;
    rotation.getEulerZYX(yaw, yaw, yaw);
    Vector3 rotAxis = {0, 1, 0};
    
    head.setRotation(rotAxis, yaw * RAD2DEG);
    torso.setRotation(rotAxis, yaw * RAD2DEG);
    leftArm.setRotation(rotAxis, yaw * RAD2DEG);
    rightArm.setRotation(rotAxis, yaw * RAD2DEG);
    leftLeg.setRotation(rotAxis, yaw * RAD2DEG);
    rightLeg.setRotation(rotAxis, yaw * RAD2DEG);
    
    this->position = centerPos;
}

void SimpleRagdoll::animateLimbs(float deltaTime) {
    if (!physicsBody) return;
    
    btVector3 velocity = physicsBody->getLinearVelocity();
    float speedSquared = velocity.x() * velocity.x() + velocity.z() * velocity.z();
    bool isMoving = speedSquared > 0.25f;
    
    if (isMoving) {
        float speed = sqrtf(speedSquared);
        animationTime += deltaTime * speed * 2.0f;
        
        float normalizedTime = fmodf(animationTime, 2.0f * PI) / PI;
        float triangleWave;
        if (normalizedTime <= 1.0f) {
            triangleWave = 2.0f * normalizedTime - 1.0f;
        } else {
            triangleWave = 3.0f - 2.0f * normalizedTime;
        }
        
        float armSwing = triangleWave * 20.0f;
        float legSwing = triangleWave * 25.0f;
        
        leftArm.setRotation({1, 0, 0}, armSwing);
        rightArm.setRotation({1, 0, 0}, -armSwing);
        leftLeg.setRotation({1, 0, 0}, -legSwing);
        rightLeg.setRotation({1, 0, 0}, legSwing);
    } else {
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
    centerPos.y -= CHARACTER_HEIGHT * 0.5f;
    return centerPos;
}

bool SimpleRagdoll::isOnGround() const {
    if (!physicsBody || !world) return false;
    
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    
    float halfHeight = CHARACTER_HEIGHT * 0.5f;
    
    float rayStartY = pos.y() - halfHeight + 0.1f;
    float rayEndY = rayStartY - 0.2f;
    
    btVector3 rayStart(pos.x(), rayStartY, pos.z());
    btVector3 rayEnd(pos.x(), rayEndY, pos.z());
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayStart, rayEnd);
    
    rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
    rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
    
    world->getBulletWorld()->rayTest(rayStart, rayEnd, rayCallback);
    
    if (rayCallback.hasHit() && rayCallback.m_collisionObject != physicsBody) {
        float hitDistance = rayCallback.m_closestHitFraction * 0.2f;
        btVector3 velocity = physicsBody->getLinearVelocity();
        
        return hitDistance < 0.20f && velocity.y() < 2.0f;
    }
    
    return false;
}

void SimpleRagdoll::handleInput(float movementSpeed) {
    if (!physicsBody) return;

    Vector2 moveAxis = InputSystem::getMovementAxis();
    


    if (InputSystem::isJumpPressed()) {
        bool onGround = isOnGround();
               
        if (onGround) {
            jump();
        }
    }

    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f) {
        Vector3 movement = {moveAxis.x, 0, -moveAxis.y};
        float lengthSquared = movement.x * movement.x + movement.z * movement.z;
        if (lengthSquared > 1.0f) {
            float invLength = 1.0f / sqrtf(lengthSquared);
            movement.x *= invLength;
            movement.z *= invLength;
        }
        
        btVector3 velocity = physicsBody->getLinearVelocity();
        
        float preservedY = velocity.y();
        
        float targetSpeed = movementSpeed;
        velocity.setX(movement.x * targetSpeed);
        velocity.setZ(movement.z * targetSpeed);
        
        velocity.setY(preservedY);
        physicsBody->setLinearVelocity(velocity);
        

        

        
    } else if (isOnGround()) {
        btVector3 velocity = physicsBody->getLinearVelocity();
        velocity.setX(0);
        velocity.setZ(0);
        physicsBody->setLinearVelocity(velocity);
    }
}

void SimpleRagdoll::jump() {
    if (!physicsBody) return;
    
    btVector3 velocity = physicsBody->getLinearVelocity();
    velocity.setY(0);
    physicsBody->setLinearVelocity(velocity);
    
    btVector3 jumpVelocity = physicsBody->getLinearVelocity();
    jumpVelocity.setY(8.0f);
    physicsBody->setLinearVelocity(jumpVelocity);
    
    float jumpForce = JUMP_IMPULSE * 2.0f;
    btVector3 jumpImpulse(0, jumpForce, 0);
    physicsBody->applyCentralImpulse(jumpImpulse);
    
    physicsBody->activate(true);
    
    physicsBody->setContactProcessingThreshold(1.0f);
}

void SimpleRagdoll::update(float deltaTime) {
    updateVisualFromPhysics();
    
    animateLimbs(deltaTime);
}

void SimpleRagdoll::draw() const {
    torso.draw();
    head.draw();
    leftArm.draw();
    rightArm.draw();
    leftLeg.draw();
    rightLeg.draw();
}

BoundingBox SimpleRagdoll::getBoundingBox() const {
    Vector3 pos = getPhysicsPosition();
    float halfHeight = CHARACTER_HEIGHT * 0.5f;
    
    return {
        {pos.x - CHARACTER_RADIUS, pos.y - halfHeight, pos.z - CHARACTER_RADIUS},
        {pos.x + CHARACTER_RADIUS, pos.y + halfHeight, pos.z + CHARACTER_RADIUS}
    };
}

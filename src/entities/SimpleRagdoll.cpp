// SimpleRagdoll.cpp - Simple character with single physics body
#include "entities/SimpleRagdoll.h"
#include "GameWorld.h"
#include "systems/InputSystem.h"
#include "raymath.h"
#include <cmath>

SimpleRagdoll::SimpleRagdoll(Vector3 position)
    : GameObject(position, true, true, false),
      torso({0, 0, 0}, GameSettings::BodyParts::TORSO_SIZE, GameSettings::BodyParts::TORSO_COLOR, false),
      head({0, 0, 0}, GameSettings::BodyParts::HEAD_SIZE, GameSettings::BodyParts::HEAD_COLOR, false),
      leftArm({0, 0, 0}, GameSettings::BodyParts::ARM_SIZE, GameSettings::BodyParts::ARM_COLOR, false),
      rightArm({0, 0, 0}, GameSettings::BodyParts::ARM_SIZE, GameSettings::BodyParts::ARM_COLOR, false),
      leftLeg({0, 0, 0}, GameSettings::BodyParts::LEG_SIZE, GameSettings::BodyParts::LEG_COLOR, false),
      rightLeg({0, 0, 0}, GameSettings::BodyParts::LEG_SIZE, GameSettings::BodyParts::LEG_COLOR, false) {
}

SimpleRagdoll::~SimpleRagdoll() {
}

void SimpleRagdoll::setupPhysics(btDiscreteDynamicsWorld* bulletWorld) {
    physicsShape = new btCapsuleShape(GameSettings::Character::RADIUS, GameSettings::Character::HEIGHT - 2*GameSettings::Character::RADIUS);
    physicsShape->setMargin(GameSettings::Collision::SHAPE_MARGIN);
    
    // Calculate proper initial Y position based on actual leg dimensions
    float torsoHalfHeight = GameSettings::BodyParts::TORSO_SIZE.y * 0.5f;
    float legHeight = GameSettings::BodyParts::LEG_SIZE.y;
    
    Vector3 physicsPos = this->position;
    // Position capsule center so that the bottom aligns with feet on ground
    float capsuleHalfHeight = GameSettings::Character::HEIGHT * 0.5f;
    float groundToFeetHeight = legHeight; // Distance from ground to bottom of torso
    physicsPos.y = groundToFeetHeight + torsoHalfHeight; // Capsule center position
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(physicsPos.x, physicsPos.y, physicsPos.z));
    motionState = new btDefaultMotionState(transform);
    btVector3 inertia(0, 0, 0);
    physicsShape->calculateLocalInertia(GameSettings::Character::MASS, inertia);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(GameSettings::Character::MASS, motionState, physicsShape, inertia);
    rbInfo.m_friction = GameSettings::Collision::FRICTION;
    rbInfo.m_rollingFriction = GameSettings::Collision::ROLLING_FRICTION;
    rbInfo.m_restitution = GameSettings::Collision::RESTITUTION;
    physicsBody = new btRigidBody(rbInfo);
    physicsBody->setDamping(GameSettings::Character::DAMPING_LINEAR, GameSettings::Character::DAMPING_ANGULAR);
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
    
    // Calculate positions based on actual body part sizes
    float torsoHalfHeight = GameSettings::BodyParts::TORSO_SIZE.y * 0.5f;
    float headHalfHeight = GameSettings::BodyParts::HEAD_SIZE.y * 0.5f;
    float armHalfHeight = GameSettings::BodyParts::ARM_SIZE.y * 0.5f;
    float legHalfHeight = GameSettings::BodyParts::LEG_SIZE.y * 0.5f;
    
    // Head positioning - on top of torso
    Vector3 headPos = {
        centerPos.x,
        centerPos.y + torsoHalfHeight + headHalfHeight + GameSettings::BodyParts::HEAD_OFFSET_Y,
        centerPos.z
    };
    
    // Torso stays at center
    Vector3 torsoPos = centerPos;
    
    // Arms positioning - at shoulder level (top part of torso)
    float shoulderY = centerPos.y + torsoHalfHeight - armHalfHeight;
    Vector3 leftArmPos = {
        centerPos.x - GameSettings::BodyParts::ARM_OFFSET_X,
        shoulderY,
        centerPos.z
    };
    Vector3 rightArmPos = {
        centerPos.x + GameSettings::BodyParts::ARM_OFFSET_X,
        shoulderY,
        centerPos.z
    };
    
    // Legs positioning - connected to bottom of torso
    float legY = centerPos.y - torsoHalfHeight - legHalfHeight;
    Vector3 leftLegPos = {
        centerPos.x - GameSettings::BodyParts::LEG_OFFSET_X,
        legY,
        centerPos.z
    };
    Vector3 rightLegPos = {
        centerPos.x + GameSettings::BodyParts::LEG_OFFSET_X,
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
    bool isMoving = speedSquared > GameSettings::Animation::SPEED_THRESHOLD;
    
    if (isMoving) {
        float speed = sqrtf(speedSquared);
        animationTime += deltaTime * speed * GameSettings::Animation::SPEED_MULTIPLIER;
        
        float normalizedTime = fmodf(animationTime, 2.0f * PI) / PI;
        float triangleWave;
        if (normalizedTime <= 1.0f) {
            triangleWave = 2.0f * normalizedTime - 1.0f;
        } else {
            triangleWave = 3.0f - 2.0f * normalizedTime;
        }
        
        float armSwing = triangleWave * GameSettings::Animation::ARM_SWING_AMOUNT;
        float legSwing = triangleWave * GameSettings::Animation::LEG_SWING_AMOUNT;
        
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
    
    // Calculate actual feet position based on body part dimensions
    float torsoHalfHeight = GameSettings::BodyParts::TORSO_SIZE.y * 0.5f;
    float legHeight = GameSettings::BodyParts::LEG_SIZE.y;
    
    // Feet are at: center - torso half height - leg height
    centerPos.y = centerPos.y - torsoHalfHeight - legHeight;
    return centerPos;
}

bool SimpleRagdoll::isOnGround() const {
    if (!physicsBody || !world) return false;
    
    btTransform transform;
    physicsBody->getMotionState()->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin();
    
    // Calculate actual feet position based on body part dimensions
    float torsoHalfHeight = GameSettings::BodyParts::TORSO_SIZE.y * 0.5f;
    float legHeight = GameSettings::BodyParts::LEG_SIZE.y;
    
    // Ray starts slightly above feet position to avoid floating point precision issues
    float rayStartY = pos.y() - torsoHalfHeight - legHeight + 0.05f;
    float rayEndY = rayStartY - GameSettings::Collision::GROUND_CHECK_DISTANCE;
    
    btVector3 rayStart(pos.x(), rayStartY, pos.z());
    btVector3 rayEnd(pos.x(), rayEndY, pos.z());
    
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayStart, rayEnd);
    
    rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
    rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
    
    world->getBulletWorld()->rayTest(rayStart, rayEnd, rayCallback);
    
    if (rayCallback.hasHit() && rayCallback.m_collisionObject != physicsBody) {
        float hitDistance = rayCallback.m_closestHitFraction * GameSettings::Collision::GROUND_CHECK_DISTANCE;
        btVector3 velocity = physicsBody->getLinearVelocity();
        
        return hitDistance < GameSettings::Collision::GROUND_CHECK_TOLERANCE && velocity.y() < GameSettings::Collision::VELOCITY_Y_THRESHOLD;
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
    
    float jumpForce = GameSettings::Character::JUMP_IMPULSE * 2.0f;
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
    float halfHeight = GameSettings::Character::HEIGHT * 0.5f;
    
    return {
        {pos.x - GameSettings::Character::RADIUS, pos.y - halfHeight, pos.z - GameSettings::Character::RADIUS},
        {pos.x + GameSettings::Character::RADIUS, pos.y + halfHeight, pos.z + GameSettings::Character::RADIUS}
    };
}

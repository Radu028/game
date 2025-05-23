// Player.cpp - Professional, Bullet3-integrated player character
#include "entities/Player.h"
#include "GameWorld.h"
#include "raymath.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"
#include <cmath>

namespace {
constexpr float CAPSULE_RADIUS = 0.3f;
constexpr float CAPSULE_HEIGHT = 1.4f;
constexpr float PLAYER_MASS = 12.0f;
constexpr float JUMP_IMPULSE = 6.5f;
constexpr float ANIMATION_SPEED = 7.0f;
constexpr float SWING_ANGLE_DEGREES = 35.0f;
constexpr float RETURN_TO_NEUTRAL_SPEED = 8.0f;
}

Player::Player(Vector3 position)
    : GameObject(position, true, true, false),
      torso(position, {0.5f, 1.5f, 0.3f}, BLUE, false),        // No collision - handled by compound shape
      head({0, 0, 0}, {0.5f, 0.5f, 0.5f}, RED, false),         // No collision - handled by compound shape
      leftArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN, false),    // No collision - handled by compound shape
      rightArm({0, 0, 0}, {0.3f, 1.0f, 0.3f}, GREEN, false),   // No collision - handled by compound shape
      leftLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW, false),   // No collision - handled by compound shape
      rightLeg({0, 0, 0}, {0.3f, 1.0f, 0.3f}, YELLOW, false) {}// No collision - handled by compound shape

Player::~Player() {
    // Capsule body cleanup is handled by PhysicsSystem
}

void Player::setupCapsuleController(btDiscreteDynamicsWorld* bulletWorld) {
    if (capsuleBody) return;
    
    // Creează compound shape pentru toate componentele
    auto* compoundShape = new btCompoundShape();
    btTransform localTransform;
    localTransform.setIdentity();

    // Torso (capsule)
    auto* torsoShape = new btCapsuleShape(CAPSULE_RADIUS, CAPSULE_HEIGHT);
    localTransform.setOrigin(btVector3(0, 0, 0));
    compoundShape->addChildShape(localTransform, torsoShape);

    // Head (sphere)
    auto* headShape = new btSphereShape(0.25f);
    localTransform.setOrigin(btVector3(0, 1.0f, 0));
    compoundShape->addChildShape(localTransform, headShape);

    // Left Arm (capsule)
    auto* leftArmShape = new btCapsuleShape(0.15f, 0.8f);
    localTransform.setOrigin(btVector3(-0.45f, 0.5f, 0));
    compoundShape->addChildShape(localTransform, leftArmShape);

    // Right Arm (capsule)
    auto* rightArmShape = new btCapsuleShape(0.15f, 0.8f);
    localTransform.setOrigin(btVector3(0.45f, 0.5f, 0));
    compoundShape->addChildShape(localTransform, rightArmShape);

    // Left Leg (capsule) - positioned so feet touch the ground
    auto* leftLegShape = new btCapsuleShape(0.15f, 0.8f);
    localTransform.setOrigin(btVector3(-0.15f, -1.1f, 0)); // Y = -0.7f (torso bottom) - 0.4f (half leg height)
    compoundShape->addChildShape(localTransform, leftLegShape);

    // Right Leg (capsule) - positioned so feet touch the ground
    auto* rightLegShape = new btCapsuleShape(0.15f, 0.8f);
    localTransform.setOrigin(btVector3(0.15f, -1.1f, 0)); // Y = -0.7f (torso bottom) - 0.4f (half leg height)
    compoundShape->addChildShape(localTransform, rightLegShape);

    // Creează rigid body cu compound shape
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(position.x, position.y, position.z));

    btVector3 inertia(0, 0, 0);
    compoundShape->calculateLocalInertia(PLAYER_MASS, inertia);

    auto* motion = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(PLAYER_MASS, motion, compoundShape, inertia);
    capsuleBody = new btRigidBody(rbInfo);

    // Setări de control
    capsuleBody->setAngularFactor(btVector3(0, 0, 0)); // Previne rotația
    capsuleBody->setDamping(0.4f, 0.8f);
    capsuleBody->setActivationState(DISABLE_DEACTIVATION);
    capsuleBody->setFriction(0.8f);
    capsuleBody->setRollingFriction(0.1f);
    capsuleBody->setSpinningFriction(0.1f);

    bulletWorld->addRigidBody(capsuleBody);
}

void Player::removeCapsuleController(btDiscreteDynamicsWorld* bulletWorld) {
    if (capsuleBody) {
        bulletWorld->removeRigidBody(capsuleBody);
        delete capsuleBody->getMotionState();
        // Șterge toate shape-urile din compound
        btCompoundShape* compound = dynamic_cast<btCompoundShape*>(capsuleBody->getCollisionShape());
        if (compound) {
            for (int i = compound->getNumChildShapes() - 1; i >= 0; --i) {
                btCollisionShape* child = compound->getChildShape(i);
                delete child;
            }
        }
        delete capsuleBody->getCollisionShape();
        delete capsuleBody;
        capsuleBody = nullptr;
    }
}

bool Player::isOnGroundBullet() const {
    if (!capsuleBody || !world) return false;
    btTransform trans;
    capsuleBody->getMotionState()->getWorldTransform(trans);
    btVector3 start = trans.getOrigin();
    btVector3 end = start - btVector3(0, 0.8f, 0);
    btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    world->getBulletWorld()->rayTest(start, end, rayCallback);
    return rayCallback.hasHit();
}

bool Player::isOnGround() const {
    return isOnGroundBullet();
}

// Individual body part collision detection
// Individual body part collision detection using precise ray casting
bool Player::checkHeadCollision() const {
    if (!capsuleBody || !world) return false;
    
    btTransform trans;
    capsuleBody->getMotionState()->getWorldTransform(trans);
    btVector3 playerPos = trans.getOrigin();
    
    // Head position (offset from player center)
    btVector3 headCenter = playerPos + btVector3(0, 1.0f, 0);
    
    // Check collision in multiple directions around head (spherical detection)
    btVector3 directions[] = {
        btVector3(0.3f, 0, 0),    // Right
        btVector3(-0.3f, 0, 0),   // Left
        btVector3(0, 0.3f, 0),    // Up
        btVector3(0, -0.3f, 0),   // Down
        btVector3(0, 0, 0.3f),    // Forward
        btVector3(0, 0, -0.3f),   // Backward
        btVector3(0.2f, 0.2f, 0), // Diagonals
        btVector3(-0.2f, 0.2f, 0),
        btVector3(0.2f, -0.2f, 0),
        btVector3(-0.2f, -0.2f, 0)
    };
    
    for (const auto& dir : directions) {
        btVector3 start = headCenter;
        btVector3 end = headCenter + dir;
        btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
        rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
        rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
        
        world->getBulletWorld()->rayTest(start, end, rayCallback);
        if (rayCallback.hasHit()) {
            // Verifică dacă nu e coliziune cu propriul corp
            if (rayCallback.m_collisionObject != capsuleBody) {
                return true;
            }
        }
    }
    
    return false;
}

bool Player::checkArmCollision(bool isLeft) const {
    if (!capsuleBody || !world) return false;
    
    btTransform trans;
    capsuleBody->getMotionState()->getWorldTransform(trans);
    btVector3 playerPos = trans.getOrigin();
    
    // Arm position (offset from player center)
    float armOffset = isLeft ? -0.45f : 0.45f;
    btVector3 armPos = playerPos + btVector3(armOffset, 0.5f, 0);
    
    // Check collision in multiple directions around arm
    btVector3 directions[] = {
        btVector3(armOffset > 0 ? 0.2f : -0.2f, 0, 0),  // Outward
        btVector3(0, 0.2f, 0),   // Up
        btVector3(0, -0.2f, 0),  // Down
        btVector3(0, 0, 0.2f),   // Forward
        btVector3(0, 0, -0.2f)   // Backward
    };
    
    for (const auto& dir : directions) {
        btVector3 start = armPos;
        btVector3 end = armPos + dir;
        btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
        rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
        rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
        
        world->getBulletWorld()->rayTest(start, end, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != capsuleBody) {
            return true;
        }
    }
    
    return false;
}

bool Player::checkLegCollision(bool isLeft) const {
    if (!capsuleBody || !world) return false;
    
    btTransform trans;
    capsuleBody->getMotionState()->getWorldTransform(trans);
    btVector3 playerPos = trans.getOrigin();
    
    // Leg position (offset from player center)
    float legOffset = isLeft ? -0.15f : 0.15f;
    float capsuleHalfHeight = CAPSULE_HEIGHT / 2.0f;
    btVector3 legPos = playerPos + btVector3(legOffset, -capsuleHalfHeight - 0.4f, 0);
    
    // Check collision in multiple directions around leg
    btVector3 directions[] = {
        btVector3(legOffset > 0 ? 0.1f : -0.1f, 0, 0),  // Outward
        btVector3(0, -0.1f, 0),  // Down
        btVector3(0, 0, 0.1f),   // Forward
        btVector3(0, 0, -0.1f)   // Backward
    };
    
    for (const auto& dir : directions) {
        btVector3 start = legPos;
        btVector3 end = legPos + dir;
        btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
        rayCallback.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
        rayCallback.m_collisionFilterMask = btBroadphaseProxy::AllFilter;
        
        world->getBulletWorld()->rayTest(start, end, rayCallback);
        if (rayCallback.hasHit() && rayCallback.m_collisionObject != capsuleBody) {
            return true;
        }
    }
    
    return false;
}

bool Player::checkTorsoCollision() const {
    if (!capsuleBody || !world) return false;
    
    btTransform trans;
    capsuleBody->getMotionState()->getWorldTransform(trans);
    btVector3 playerPos = trans.getOrigin();
    
    // Check collision around torso center
    btVector3 directions[] = {
        btVector3(0.1f, 0, 0),   // Right
        btVector3(-0.1f, 0, 0),  // Left
        btVector3(0, 0, 0.1f),   // Forward
        btVector3(0, 0, -0.1f)   // Backward
    };
    
    for (const auto& dir : directions) {
        btVector3 start = playerPos;
        btVector3 end = playerPos + dir;
        btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
        world->getBulletWorld()->rayTest(start, end, rayCallback);
        if (rayCallback.hasHit()) return true;
    }
    
    return false;
}

void Player::handleInput(float movementSpeed) {
    if (!capsuleBody) return;
    Vector2 moveAxis = InputSystem::getMovementAxis();
    btVector3 forward(0, 0, -1), right(1, 0, 0);
    btVector3 move = moveAxis.x * right + moveAxis.y * forward;
    if (move.length2() > 0.01f) {
        move.normalize();
        move *= movementSpeed * 50.0f;
        btVector3 vel = capsuleBody->getLinearVelocity();
        move.setY(0);
        vel.setY(0);
        btVector3 desiredVel = move;
        btVector3 impulse = (desiredVel - vel) * capsuleBody->getMass();
        capsuleBody->applyCentralImpulse(impulse * 0.5f);
    }
    if (InputSystem::isJumpPressed()) {
        btVector3 vel = capsuleBody->getLinearVelocity();
        if (std::abs(vel.y()) < 0.1f && isOnGroundBullet()) {
            capsuleBody->applyCentralImpulse(btVector3(0, JUMP_IMPULSE, 0));
        }
    }
}

void Player::updateBodyPartPositions() {
    if (!capsuleBody) return;
    btTransform t;
    capsuleBody->getMotionState()->getWorldTransform(t);
    btVector3 p = t.getOrigin();
    // Torso (center of capsule)
    torso.setPosition({p.x(), p.y(), p.z()});
    torso.setRotation({0, 1, 0}, 0);
    // Head (above torso)
    Vector3 headOffset = {0, 1.0f, 0};
    head.setPosition({p.x() + headOffset.x, p.y() + headOffset.y, p.z() + headOffset.z});
    // Arms (sideways, mid torso)
    leftArm.setPosition({p.x() - 0.45f, p.y() + 0.5f, p.z()});
    rightArm.setPosition({p.x() + 0.45f, p.y() + 0.5f, p.z()});
    // Legs (below capsule)
    float capsuleHalfHeight = CAPSULE_HEIGHT / 2.0f;
    float legLength = 1.0f;
    float legY = p.y() - capsuleHalfHeight - (legLength / 2.0f) + 0.05f;
    leftLeg.setPosition({p.x() - 0.15f, legY, p.z()});
    rightLeg.setPosition({p.x() + 0.15f, legY, p.z()});
}

void Player::update(float deltaTime) {
    if (capsuleBody) {
        btTransform trans;
        capsuleBody->getMotionState()->getWorldTransform(trans);
        btVector3 pos = trans.getOrigin();
        this->position = {pos.x(), pos.y(), pos.z()};
        updateBodyPartPositions();
    }
    // Animate limbs (visual only)
    const Vector3 swingAxis = {1.0f, 0.0f, 0.0f};
    static float animTime = 0.0f;
    Vector2 moveInput = InputSystem::getMovementAxis();
    bool isMoving = (moveInput.x != 0.0f || moveInput.y != 0.0f);
    if (isMoving) {
        animTime += deltaTime * ANIMATION_SPEED;
        float angleDegrees = sin(animTime) * SWING_ANGLE_DEGREES;
        leftArm.setRotation(swingAxis, angleDegrees);
        rightArm.setRotation(swingAxis, -angleDegrees);
        leftLeg.setRotation(swingAxis, -angleDegrees);
        rightLeg.setRotation(swingAxis, angleDegrees);
    } else {
        const float NEUTRAL_ANGLE = 0.0f;
        const float ANGLE_THRESHOLD = 0.1f;
        auto returnLimbToNeutral = [&](BodyPart &limb) {
            float currentAngle = limb.getRotationAngle();
            if (std::abs(currentAngle - NEUTRAL_ANGLE) > ANGLE_THRESHOLD) {
                float newAngle = Lerp(currentAngle, NEUTRAL_ANGLE, RETURN_TO_NEUTRAL_SPEED * deltaTime);
                limb.setRotation(swingAxis, newAngle);
            } else {
                limb.setRotation(swingAxis, NEUTRAL_ANGLE);
            }
        };
        returnLimbToNeutral(leftArm);
        returnLimbToNeutral(rightArm);
        returnLimbToNeutral(leftLeg);
        returnLimbToNeutral(rightLeg);
    }
}

void Player::postPhysicsUpdate(float deltaTime) {
    updateBodyPartPositions();
    // Animate limbs (visual only)
    const Vector3 swingAxis = {1.0f, 0.0f, 0.0f};
    static float animTime = 0.0f;
    Vector2 moveInput = InputSystem::getMovementAxis();
    bool isMoving = (moveInput.x != 0.0f || moveInput.y != 0.0f);
    if (isMoving) {
        animTime += deltaTime * ANIMATION_SPEED;
        float angleDegrees = sin(animTime) * SWING_ANGLE_DEGREES;
        leftArm.setRotation(swingAxis, angleDegrees);
        rightArm.setRotation(swingAxis, -angleDegrees);
        leftLeg.setRotation(swingAxis, -angleDegrees);
        rightLeg.setRotation(swingAxis, angleDegrees);
    } else {
        const float NEUTRAL_ANGLE = 0.0f;
        const float ANGLE_THRESHOLD = 0.1f;
        auto returnLimbToNeutral = [&](BodyPart &limb) {
            float currentAngle = limb.getRotationAngle();
            if (std::abs(currentAngle - NEUTRAL_ANGLE) > ANGLE_THRESHOLD) {
                float newAngle = Lerp(currentAngle, NEUTRAL_ANGLE, RETURN_TO_NEUTRAL_SPEED * deltaTime);
                limb.setRotation(swingAxis, newAngle);
            } else {
                limb.setRotation(swingAxis, NEUTRAL_ANGLE);
            }
        };
        returnLimbToNeutral(leftArm);
        returnLimbToNeutral(rightArm);
        returnLimbToNeutral(leftLeg);
        returnLimbToNeutral(rightLeg);
    }
}

void Player::draw() const {
    torso.draw();
    head.draw();
    leftArm.draw();
    rightArm.draw();
    leftLeg.draw();
    rightLeg.draw();
}

BoundingBox Player::getBoundingBox() const {
    BoundingBox combinedBox = torso.getBoundingBox();
    BoundingBox headBox = head.getBoundingBox();
    BoundingBox leftArmBox = leftArm.getBoundingBox();
    BoundingBox rightArmBox = rightArm.getBoundingBox();
    BoundingBox leftLegBox = leftLeg.getBoundingBox();
    BoundingBox rightLegBox = rightLeg.getBoundingBox();
    combinedBox.min = Vector3Min(combinedBox.min, headBox.min);
    combinedBox.max = Vector3Max(combinedBox.max, headBox.max);
    combinedBox.min = Vector3Min(combinedBox.min, leftArmBox.min);
    combinedBox.max = Vector3Max(combinedBox.max, leftArmBox.max);
    combinedBox.min = Vector3Min(combinedBox.min, rightArmBox.min);
    combinedBox.max = Vector3Max(combinedBox.max, rightArmBox.max);
    combinedBox.min = Vector3Min(combinedBox.min, leftLegBox.min);
    combinedBox.max = Vector3Max(combinedBox.max, leftLegBox.max);
    combinedBox.min = Vector3Min(combinedBox.min, rightLegBox.min);
    combinedBox.max = Vector3Max(combinedBox.max, rightLegBox.max);
    return combinedBox;
}

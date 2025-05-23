// AdvancedPlayerController.cpp - Implementare pentru ca playerul să stea cu picioarele pe podea
#include "entities/AdvancedPlayerController.h"
#include <cmath>
#include "raymath.h"

namespace {
constexpr float ADV_PLAYER_CAPSULE_RADIUS = 0.3f;
constexpr float ADV_PLAYER_CAPSULE_HEIGHT = 1.4f;
constexpr float ADV_PLAYER_CAPSULE_HALF_HEIGHT = ADV_PLAYER_CAPSULE_HEIGHT / 2.0f;
}

AdvancedPlayerController::AdvancedPlayerController(btDiscreteDynamicsWorld* dynamicsWorld)
    : world(dynamicsWorld), position{0,0,0}, velocity{0,0,0}, onGround(false) {
    // Creează ghost object pentru capsulă (corpul principal)
    convexShape = new btCapsuleShape(ADV_PLAYER_CAPSULE_RADIUS, ADV_PLAYER_CAPSULE_HEIGHT);
    ghostObject = new btGhostObject();
    ghostObject->setCollisionShape(convexShape);
    ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
    world->addCollisionObject(ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
    // Inițializează ghost objects pentru membre dacă vrei detecție suplimentară
}

AdvancedPlayerController::~AdvancedPlayerController() {
    world->removeCollisionObject(ghostObject);
    delete ghostObject;
    delete convexShape;
    // Șterge și ghost-urile pentru membre dacă există
}

void AdvancedPlayerController::setPosition(const Vector3& pos) {
    position = pos;
    btTransform t;
    t.setIdentity();
    // Poziționează ghostObject astfel încât baza capsulei să fie la poziția dorită (solul)
    t.setOrigin(btVector3(pos.x, pos.y + ADV_PLAYER_CAPSULE_HALF_HEIGHT, pos.z));
    if (ghostObject) ghostObject->setWorldTransform(t);
    updateGhostObjects();
}

void AdvancedPlayerController::syncPlayerVisual(Player& player) {
    // Poziția vizuală a playerului trebuie să fie la baza capsulei (adică la sol)
    Vector3 visualPos = position;
    visualPos.y += ADV_PLAYER_CAPSULE_HALF_HEIGHT;
    player.setPosition(visualPos);
}

void AdvancedPlayerController::update(float deltaTime) {
    // Simulează mișcarea (simplificat)
    Vector3 scaledVel = Vector3Scale(velocity, deltaTime);
    position = Vector3Add(position, scaledVel);
    setPosition(position); // repoziționează ghostObject cu baza la sol
    performGroundCheck();
    updateGhostObjects();
}

void AdvancedPlayerController::handleInput(float movementSpeed) {
    // Exemplu simplu: WASD = modifică velocity
    // (integrează cu InputSystem dacă vrei)
}

void AdvancedPlayerController::updateGhostObjects() {
    // Repoziționează ghost-urile pentru membre relativ la ghostObject
    // Exemplu pentru picior stâng:
    // if (leftLegCollider.ghost) {
    //     btTransform t = ghostObject->getWorldTransform();
    //     t.setOrigin(t.getOrigin() + btVector3(leftLegCollider.localOffset.x, leftLegCollider.localOffset.y, leftLegCollider.localOffset.z));
    //     leftLegCollider.ghost->setWorldTransform(t);
    // }
}

void AdvancedPlayerController::performGroundCheck() {
    // Verifică dacă baza capsulei atinge podeaua
    btTransform t = ghostObject->getWorldTransform();
    btVector3 base = t.getOrigin() - btVector3(0, ADV_PLAYER_CAPSULE_HALF_HEIGHT, 0);
    btVector3 end = base - btVector3(0, 0.2f, 0);
    btCollisionWorld::ClosestRayResultCallback rayCallback(base, end);
    world->rayTest(base, end, rayCallback);
    onGround = rayCallback.hasHit();
}

AdvancedPlayerController::BodyPartCollider AdvancedPlayerController::createBodyPartCollider(const Vector3& size, const Vector3& offset) {
    BodyPartCollider collider;
    collider.shape = new btCapsuleShape(size.x, size.y);
    collider.ghost = new btGhostObject();
    collider.ghost->setCollisionShape(collider.shape);
    collider.ghost->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    collider.localOffset = offset;
    world->addCollisionObject(collider.ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
    return collider;
}

bool AdvancedPlayerController::checkHeadCollision() const {
    // Exemplu: verifică coliziunea ghost-ului capului
    // return headCollider.ghost && headCollider.ghost->getNumOverlappingObjects() > 0;
    return false;
}
bool AdvancedPlayerController::checkArmCollision(bool isLeft) const { return false; }
bool AdvancedPlayerController::checkLegCollision(bool isLeft) const { return false; }

#include "systems/PhysicsSystem.h"
#include <algorithm>
#include "GameWorld.h"
#include "entities/Player.h"
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/Wall.h"
#include "raymath.h"
#include "settings/Physics.h"
#include <BulletDynamics/Character/btKinematicCharacterController.h>

PhysicsSystem::PhysicsSystem(GameWorld* gameWorld) : world(gameWorld) {
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    dynamicsWorld->setGravity(btVector3(0, PhysicsSettings::GRAVITY_ACCELERATION, 0));
    // Register ghost pair callback for character controller collision
    dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
}

PhysicsSystem::~PhysicsSystem() {
    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
}

void PhysicsSystem::addObject(GameObject* obj) {
    if (auto* player = dynamic_cast<Player*>(obj)) {
        addPlayerParts(player);
        return;
    } else {
        btCollisionShape* shape = nullptr;
        // Pentru CubeObject
        if (auto* cube = dynamic_cast<CubeObject*>(obj)) {
            Vector3 sz = cube->getSize();
            shape = new btBoxShape(btVector3(sz.x/2.0f, sz.y/2.0f, sz.z/2.0f));
        }
        // Pentru Floor
        else if (auto* floor = dynamic_cast<Floor*>(obj)) {
            Vector3 sz = floor->getBoundingBox().max;
            Vector3 mn = floor->getBoundingBox().min;
            Vector3 dims = {sz.x - mn.x, sz.y - mn.y, sz.z - mn.z};
            shape = new btBoxShape(btVector3(dims.x/2.0f, dims.y/2.0f, dims.z/2.0f));
        }
        // Pentru Wall
        else if (auto* wall = dynamic_cast<Wall*>(obj)) {
            Vector3 sz = wall->getBoundingBox().max;
            Vector3 mn = wall->getBoundingBox().min;
            Vector3 dims = {sz.x - mn.x, sz.y - mn.y, sz.z - mn.z};
            shape = new btBoxShape(btVector3(dims.x/2.0f, dims.y/2.0f, dims.z/2.0f));
        }
        // fallback
        else {
            shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
        }
        btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(obj->getPosition().x, obj->getPosition().y, obj->getPosition().z)));
        btScalar mass = obj->getIsStatic() ? 0.0f : 1.0f;
        btVector3 inertia(0,0,0);
        if (mass != 0.0f) shape->calculateLocalInertia(mass, inertia);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setActivationState(DISABLE_DEACTIVATION);
        if (dynamic_cast<Floor*>(obj)) {
            body->setFriction(1.5f);
            body->setRollingFriction(1.5f);
            body->setSpinningFriction(1.5f);
            body->setDamping(0.0f, 0.0f);
            // Ensure Bullet treats this as a static object
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
        } else if (dynamic_cast<CubeObject*>(obj)) {
            body->setFriction(2.0f);
            body->setRollingFriction(2.0f);
            body->setSpinningFriction(2.0f);
            body->setDamping(0.8f, 0.8f);
        }
        dynamicsWorld->addRigidBody(body);
        // Debug: Print info for static objects (Floor)
        if (dynamic_cast<Floor*>(obj)) {
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            btVector3 pos = trans.getOrigin();
            printf("[DEBUG][Bullet] Floor rigid body Bullet position: (%.2f, %.2f, %.2f)\n", pos.x(), pos.y(), pos.z());
        }
        obj->setBulletBody(body);
        objectToBody[obj] = body;
        physicsObjects.push_back(obj);
    }
}

void PhysicsSystem::removeObject(GameObject* obj) {
    if (auto* player = dynamic_cast<Player*>(obj)) {
        removePlayerParts(player);
        return;
    }
    auto it = objectToBody.find(obj);
    if (it != objectToBody.end()) {
        dynamicsWorld->removeRigidBody(it->second);
        delete it->second->getMotionState();
        delete it->second->getCollisionShape();
        delete it->second;
        objectToBody.erase(it);
    }
    physicsObjects.erase(std::remove(physicsObjects.begin(), physicsObjects.end(), obj), physicsObjects.end());
}

void PhysicsSystem::update(float deltaTime) {
    dynamicsWorld->stepSimulation(deltaTime, 10);
    syncGameObjectsFromBullet();
}

void PhysicsSystem::syncGameObjectsFromBullet() {
    for (auto* obj : physicsObjects) {
        btRigidBody* body = obj->getBulletBody();
        if (body) {
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            btVector3 pos = trans.getOrigin();
            obj->setPosition({pos.x(), pos.y(), pos.z()});
        }
    }
}

void PhysicsSystem::addPlayerParts(Player* player) {
    // Only add physics body if player has collision enabled
    if (player->getHasCollision()) {
        player->setupCapsuleController(dynamicsWorld);
    }
}

void PhysicsSystem::removePlayerParts(Player* player) {
    player->removeCapsuleController(dynamicsWorld);
}
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

PhysicsSystem::PhysicsSystem(GameWorld* gameWorld) : world(gameWorld) {
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    dynamicsWorld->setGravity(btVector3(0, PhysicsSettings::GRAVITY_ACCELERATION, 0));
}

PhysicsSystem::~PhysicsSystem() {
    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
}

void PhysicsSystem::addObject(GameObject* obj) {
    // Dacă e Player, adaugă UN SINGUR rigid body pentru bounding box-ul total
    if (auto* player = dynamic_cast<Player*>(obj)) {
        BoundingBox box = player->getBoundingBox();
        Vector3 dims = {box.max.x - box.min.x, box.max.y - box.min.y, box.max.z - box.min.z};
        btCollisionShape* shape = new btBoxShape(btVector3(dims.x/2.0f, dims.y/2.0f, dims.z/2.0f));
        Vector3 pos = player->getPosition();
        btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(pos.x, pos.y, pos.z)));
        btScalar mass = 1.0f;
        btVector3 inertia(0,0,0);
        shape->calculateLocalInertia(mass, inertia);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setActivationState(DISABLE_DEACTIVATION);
        body->setFriction(0.8f); // fricțiune moderată
        body->setRollingFriction(0.8f);
        body->setSpinningFriction(0.8f);
        body->setDamping(0.2f, 0.3f); // damping mai mic pentru control
        dynamicsWorld->addRigidBody(body);
        player->setBulletBody(body);
        objectToBody[player] = body;
        physicsObjects.push_back(player);
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
            body->setFriction(1.0f);
            body->setRollingFriction(1.0f);
            body->setSpinningFriction(1.0f);
            body->setDamping(0.0f, 0.0f);
        } else if (dynamic_cast<CubeObject*>(obj)) {
            body->setFriction(0.8f);
            body->setRollingFriction(0.8f);
            body->setSpinningFriction(0.8f);
            body->setDamping(0.2f, 0.3f);
        }
        dynamicsWorld->addRigidBody(body);
        obj->setBulletBody(body);
        objectToBody[obj] = body;
        physicsObjects.push_back(obj);
    }
}

void PhysicsSystem::removeObject(GameObject* obj) {
    // Dacă e Player, elimină doar corpul rigid principal
    if (auto* player = dynamic_cast<Player*>(obj)) {
        auto it = objectToBody.find(player);
        if (it != objectToBody.end()) {
            dynamicsWorld->removeRigidBody(it->second);
            delete it->second->getMotionState();
            delete it->second->getCollisionShape();
            delete it->second;
            objectToBody.erase(it);
        }
        physicsObjects.erase(std::remove(physicsObjects.begin(), physicsObjects.end(), player), physicsObjects.end());
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
            // Pentru Player, repoziționează și BodyPart-urile vizual
            if (auto* player = dynamic_cast<Player*>(obj)) {
                player->updateBodyPartPositions();
            }
        }
    }
}
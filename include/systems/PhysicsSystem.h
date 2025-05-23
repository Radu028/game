#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H

#include <btBulletDynamicsCommon.h>
#include <unordered_map>
#include <vector>

#include "objects/GameObject.h"
#include "raylib.h"
#include "settings/Physics.h"

class GameWorld;

class PhysicsSystem {
 private:
  std::vector<GameObject*> physicsObjects;
  std::unordered_map<GameObject*, btRigidBody*> objectToBody;  // mapare GameObject -> btRigidBody
  GameWorld* world;

  // Bullet Physics
  btDefaultCollisionConfiguration* collisionConfig;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* broadphase;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;

 public:
  PhysicsSystem(GameWorld* gameWorld);
  ~PhysicsSystem();

  void addObject(GameObject* obj);
  void removeObject(GameObject* obj);
  void update(float deltaTime);

 private:
  void syncGameObjectsFromBullet();
};

#endif

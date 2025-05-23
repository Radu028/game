// Player.h - Professional, Bullet3-integrated player character
#ifndef PLAYER_H
#define PLAYER_H

#include <memory>
#include <string>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include "BodyPart.h"
#include "objects/GameObject.h"
#include "raylib.h"

class GameWorld;

/**
 * Player character, fully integrated with Bullet Physics (Bullet3).
 * Uses a single compound rigid body for all physics and collision.
 * Each body part has its own collision shape within the compound.
 * Visual body parts are synchronized with the compound body.
 */
class Player : public GameObject {
 private:
  // Visual body parts (no collision, only for rendering)
  BodyPart torso;
  BodyPart head;
  BodyPart leftArm, rightArm;
  BodyPart leftLeg, rightLeg;

  GameWorld* world = nullptr;

  // Bullet physics compound body (contains collision shapes for all body parts)
  btRigidBody* capsuleBody = nullptr;

  // --- Internal helpers ---
  void updateBodyPartPositions();
  bool isOnGroundBullet() const;

 public:
  // Construct player at given position
  explicit Player(Vector3 position = {0.0f, 0.0f, 0.0f});
  ~Player() override;

  // Bullet capsule setup/teardown
  void setupCapsuleController(btDiscreteDynamicsWorld* bulletWorld);
  void removeCapsuleController(btDiscreteDynamicsWorld* bulletWorld);

  // Set game world pointer
  void setWorld(GameWorld* gameWorld) { world = gameWorld; }

  // Input & update
  void handleInput(float movementSpeed);
  void update(float deltaTime) override;
  void postPhysicsUpdate(float deltaTime);
  void draw() const override;

  // Bounding box covers all body parts (for selection, not collision)
  BoundingBox getBoundingBox() const override;

  // Bullet-based ground check
  bool isOnGround() const;

  // Individual body part collision detection
  bool checkHeadCollision() const;
  bool checkArmCollision(bool isLeft) const;
  bool checkLegCollision(bool isLeft) const;
  bool checkTorsoCollision() const;

  // Access to body parts (for visuals)
  const BodyPart& getTorso() const { return torso; }
  const BodyPart& getHead() const { return head; }
  const BodyPart& getLeftArm() const { return leftArm; }
  const BodyPart& getRightArm() const { return rightArm; }
  const BodyPart& getLeftLeg() const { return leftLeg; }
  const BodyPart& getRightLeg() const { return rightLeg; }

  // Capsule body access (for physics system)
  btRigidBody* getCapsuleBody() const { return capsuleBody; }
};

#endif
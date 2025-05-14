#ifndef PLAYER_H
#define PLAYER_H

#include <memory>
#include <string>

#include "BodyPart.h"
#include "objects/GameObject.h"
#include "raylib.h"

enum Direction { FORWARD, BACKWARD, LEFT, RIGHT };

class Player : public GameObject {
 private:
  BodyPart torso;
  BodyPart head;
  BodyPart leftArm, rightArm;
  BodyPart leftLeg, rightLeg;

  GameWorld* world;

  void move(Direction direction, float byValue);
  void jump();
  bool checkCollisionWithWorldHorizontal() const;
  void moveWithSliding(float start, float end, float* positionComponent);

 public:
  Player(Vector3 position = (Vector3){0.0f, 0.0f, 0.0f});

  void updateBodyPartPositions();

  void setWorld(GameWorld* gameWorld) { this->world = gameWorld; }
  void handleInput(float movementSpeed);
  void update(float deltaTime) override;
  void draw() const override;

  float getVerticalCollisionContactTime(const Vector3& vericalMovementVector,
                                        const GameWorld* world,
                                        int maxIterations) const override;

  BoundingBox getBoundingBox() const override;

  void performDetailedGroundCheck();
};

#endif
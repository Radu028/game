#pragma once

#include "objects/GameObject.h"
#include "raylib.h"

class GameObject;

class BodyPart : public GameObject {
 private:
  Vector3 size;
  Color color;
  Vector3 rotationAxis;
  float rotationAngle;

  Model model;

 public:
  BodyPart(Vector3 position, Vector3 size, Color color,
           bool hasCollision = true);

  void setRotation(const Vector3 axis, float angle);
  void setPosition(Vector3 newPos);

  Vector3 getSize() const;
  Color getColor() const;

  float getRotationAngle() const;
  Vector3 getRotationAxis() const;

  void draw() const override;
  BoundingBox getBoundingBox() const override;

  void setBulletBody(btRigidBody* body) { GameObject::setBulletBody(body); }
  btRigidBody* getBulletBody() const { return GameObject::getBulletBody(); }
};

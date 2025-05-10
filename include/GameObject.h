#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include <memory>  // For std::shared_ptr

#include "raylib.h"

class GameObject {
 protected:
  Vector3 position;
  // Vector3 size;

  // Color color;
  // Model model;

  bool hasCollision;

 public:
  // GameObject(Vector3 position, Vector3 size, Color color, bool hasCollision =
  // true);
  GameObject(Vector3 position, bool hasCollision = true);
  virtual ~GameObject() = default;

  Vector3 getPosition() const { return position; }
  // Vector3 getSize() const { return size; }
  // Color getColor() const { return color; }
  bool getHasCollision() const { return hasCollision; }

  void setPosition(Vector3 newPosition) { this->position = newPosition; }

  // BoundingBox getBoundingBox() const;
  virtual void update(float deltaTime) {};
  virtual void draw() const = 0;

  // Virtual clone method for object copying
  virtual std::shared_ptr<GameObject> clone() const = 0;

  // Virtual interact method for object interactions
  virtual void interact() {};

  bool checkCollision(const GameObject& other) const;
  // virtual void handleCollision(GameObject& other);

  float getDistance(const GameObject& other) const;
};

#endif
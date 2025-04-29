#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include <memory>

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

  // Virtual clone constructor
  virtual std::shared_ptr<GameObject> clone() const = 0;

  Vector3 getPosition() const { return position; }
  // Vector3 getSize() const { return size; }
  // Color getColor() const { return color; }
  bool getHasCollision() const { return hasCollision; }

  void setPosition(Vector3 newPosition) { this->position = newPosition; }

  virtual void update(float deltaTime) {};
  virtual void draw() const = 0;
  virtual void interact() = 0;
  virtual BoundingBox getBoundingBox() const = 0;

  bool checkCollision(const GameObject& other) const;
  // virtual void handleCollision(GameObject& other);

  float getDistance(const GameObject& other) const;
};

#endif
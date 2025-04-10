#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H

#include "raylib.h"

class GameObject {
   protected:
    Vector3 position;
    Vector3 size;
    Color color;
    bool hasCollision;

   public:
    GameObject(Vector3 position, Vector3 size, Color color, bool hasCollision = true);
    virtual ~GameObject() = default;

    Vector3 getPosition() const { return position; }
    Vector3 getSize() const { return size; }
    Color getColor() const { return color; }
    bool getHasCollision() const { return hasCollision; }

    void setPosition(Vector3 newPosition) { this->position = newPosition; }

    BoundingBox getBoundingBox() const;
    virtual void update(float deltaTime) {};
    virtual void draw() const;

    bool checkCollision(const GameObject& other) const;
    // virtual void handleCollision(GameObject& other);

    float getDistance(const GameObject& other) const;
};

#endif
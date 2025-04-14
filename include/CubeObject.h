#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include "GameObject.h"
#include "raylib.h"

class GameObject;

class CubeObject : public GameObject {
   private:
    float width, height, length;
    Color color;

   public:
    CubeObject(Vector3 position, float width, float height, float length, Color color,
               bool hasCollision = true);

    BoundingBox getBoundingBox() const;
    void draw() const;
};

#endif
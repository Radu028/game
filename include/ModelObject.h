#ifndef MODELOBJECT_H
#define MODELOBJECT_H

#include "GameObject.h"
#include "raylib.h"

class GameObject;

class ModelObject : public GameObject {
   private:
    float scale;
    Model model;
    Color tint;

   public:
    ModelObject(Model model, Vector3 position, float scale, Color tint, bool hasCollision = true);

    void draw() const;
};

#endif
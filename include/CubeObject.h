#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include "GameObject.h"
#include "raylib.h"

class GameObject;

class CubeObject : public GameObject {
   protected:
    float width, height, length;
    Color color;

    Texture2D texture;
    bool hasTexture;
    Model model;

   public:
    CubeObject(Vector3 position, float width, float height, float length, Color color,
               bool hasCollision, const char* texturePath = nullptr);
    ~CubeObject() override;

    BoundingBox getBoundingBox() const;
    void draw() const override;

    bool checkCollision(const CubeObject& other) const;
};

#endif
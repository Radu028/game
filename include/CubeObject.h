#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include <memory>  // For std::shared_ptr

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
  CubeObject(Vector3 position, float width, float height, float length,
             Color color, bool hasCollision, const char* texturePath = nullptr);
  ~CubeObject() override;

  // Copy constructor
  CubeObject(const CubeObject& other);

  // Clone method
  std::shared_ptr<GameObject> clone() const override;

  BoundingBox getBoundingBox() const override;
  void draw() const override;

  bool checkCollision(const CubeObject& other) const;

  void interact() override;
};

#endif
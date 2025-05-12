#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include <memory>  // For std::shared_ptr
#include <string>

#include "objects/GameObject.h"
#include "raylib.h"

class GameObject;

class CubeObject : public GameObject {
 protected:
  Vector3 size;
  Color color;

  Texture2D texture;
  bool hasTexture;
  bool textureLoaded;
  Model model;

 public:
  CubeObject(Vector3 position, Vector3 size, Color color, bool hasCollision,
             const std::string& texturePath = "",
             bool affectedByGravity = false, bool isStatic = true);
  ~CubeObject() override;

  // Copy constructor
  CubeObject(const CubeObject& other);

  Vector3 getSize() const;

  BoundingBox getBoundingBox() const override;
  void draw() const override;

  bool checkCollision(const CubeObject& other) const;

  void interact() override;
};

#endif
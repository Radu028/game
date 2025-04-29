#ifndef CUBEOBJECT_H
#define CUBEOBJECT_H

#include <memory>

#include "GameObject.h"
#include "raylib.h"

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
  // Assignment operator
  CubeObject& operator=(CubeObject other);
  // Swap method for copy-and-swap idiom
  friend void swap(CubeObject& first, CubeObject& second) noexcept;

  std::shared_ptr<GameObject> clone() const override;
  BoundingBox getBoundingBox() const override;
  void draw() const override;
  void interact() override;

  bool checkCollision(const CubeObject& other) const;
};

#endif
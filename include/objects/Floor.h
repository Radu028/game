#pragma once

#include <string>

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Floor : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

  bool hasTexture;
  Model model;

  bool useShaders;

 public:
  Floor(Vector3 position, Vector3 dimensions, Color color,
        bool hasCollision = true, bool useShaders = true);
  Floor(Vector3 position, Vector3 dimensions, std::string texturePath,
        bool hasCollision = true);

  void draw() const override;
  BoundingBox getBoundingBox() const override;
  std::unique_ptr<GameObject> clone() const override;
};
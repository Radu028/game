#pragma once

#include <string>

#include "objects/StaticWorldObject.h"

class StaticWorldObject;

class Floor : public StaticWorldObject {
  Vector3 dimensions;
  Color color;

  bool hasTexture;
  Model model;

 public:
  Floor(Vector3 position, Vector3 dimensions, Color color,
        bool hasCollision = true);
  Floor(Vector3 position, Vector3 dimensions, std::string texturePath,
        bool hasCollision = true);

  void draw() const;
  BoundingBox getBoundingBox() const;
};
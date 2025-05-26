#pragma once

#include "objects/StaticWorldObject.h"

class Sphere : public StaticWorldObject {
 private:
  float radius;
  Color color;

  Model model;
  bool useShaders;

 public:
  Sphere(Vector3 position, float radius, Color color, bool hasCollisions = true,
         bool useShaders = true);

  void draw() const;
  BoundingBox getBoundingBox() const;
  float getRadius() const { return radius; }
};
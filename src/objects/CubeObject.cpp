#include "objects/CubeObject.h"

#include <iostream>
#include <memory>
#include <string>

CubeObject::CubeObject(Vector3 position, Vector3 size, Color color,
                       bool hasCollision, const std::string& texturePath,
                       bool affectedByGravity, bool isStatic)
    : GameObject(position, hasCollision),
      size(size),
      color(color),
      textureLoaded(false) {
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));

  if (!texturePath.empty()) {
    texture = LoadTexture(texturePath.c_str());
    if (texture.id > 0) {
      textureLoaded = true;
      hasTexture = true;
      model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    } else {
      TraceLog(LOG_WARNING, "Failed to load texture: %s", texturePath.c_str());
    }
  }
}

CubeObject::CubeObject(const CubeObject& other)
    : GameObject(other.position, other.hasCollision),
      size(other.size),
      color(other.color),
      hasTexture(other.hasTexture) {
  // Create new model instance
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));

  // Handle texture if present
  if (hasTexture) {
    // Since we can't directly copy textures in raylib, we'll use the same one
    // This assumes the texture is still available at runtime
    texture = other.texture;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  }
}

CubeObject::~CubeObject() {
  UnloadModel(model);

  if (hasTexture) {
    UnloadTexture(texture);
  }
}

Vector3 CubeObject::getSize() const { return size; }

BoundingBox CubeObject::getBoundingBox() const {
  return (BoundingBox){
      (Vector3){position.x - size.x / 2, position.y - size.y / 2,
                position.z - size.z / 2},
      (Vector3){position.x + size.x / 2, position.y + size.y / 2,
                position.z + size.z / 2},
  };
}

void CubeObject::draw() const {
  if (hasTexture) {
    DrawModel(model, position, 1.0f, WHITE);
  } else {
    DrawCube(position, size.x, size.y, size.z, color);
  }

  Color wireColor = RED;
  if (color.r == 255) wireColor = GREEN;

  BoundingBox box = getBoundingBox();
  DrawBoundingBox(box, wireColor);
}

bool CubeObject::checkCollision(const CubeObject& other) const {
  if (!hasCollision || !other.getHasCollision()) {
    return false;
  }

  BoundingBox box1 = this->getBoundingBox();
  BoundingBox box2 = other.getBoundingBox();

  bool result = CheckCollisionBoxes(box1, box2);
  return result;
}

void CubeObject::interact() { std::cout << "Test" << std::endl; }
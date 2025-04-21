#include "CubeObject.h"

CubeObject::CubeObject(Vector3 position, float width, float height,
                       float length, Color color, bool hasCollision,
                       const char* texturePath)
    : GameObject(position, hasCollision),
      width(width),
      height(height),
      length(length),
      color(color) {
  model = LoadModelFromMesh(GenMeshCube(width, height, length));

  if (texturePath != nullptr) {
    texture = LoadTexture(texturePath);
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    hasTexture = true;
  } else {
    hasTexture = false;
  }
}

CubeObject::~CubeObject() {
  UnloadModel(model);

  if (hasTexture) {
    UnloadTexture(texture);
  }
}

BoundingBox CubeObject::getBoundingBox() const {
  return (BoundingBox){
      (Vector3){position.x - width / 2, position.y - height / 2,
                position.z - length / 2},
      (Vector3){position.x + width / 2, position.y + height / 2,
                position.z + length / 2},
  };
}

void CubeObject::draw() const {
  if (hasTexture) {
    DrawModel(model, position, 1.0f, WHITE);
  } else {
    DrawCube(position, width, height, length, color);
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
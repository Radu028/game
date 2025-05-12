#include "objects/Floor.h"

#include <string>

Floor::Floor(Vector3 position, Vector3 dimensions, Color color,
             bool hasCollision)
    : StaticWorldObject(position, hasCollision),
      dimensions(dimensions),
      color(color),
      hasTexture(false) {}

Floor::Floor(Vector3 position, Vector3 dimensions, std::string texturePath,
             bool hasCollision)
    : StaticWorldObject(position, true), dimensions(dimensions), color(WHITE) {
  model =
      LoadModelFromMesh(GenMeshCube(dimensions.x, dimensions.y, dimensions.z));

  Texture2D texture = LoadTexture(texturePath.c_str());
  if (texture.id > 0) {
    hasTexture = true;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
  } else {
    TraceLog(LOG_WARNING, "Failed to load texture: %s", texturePath.c_str());
  }
}

void Floor::draw() const {
  if (hasTexture) {
    DrawModel(model, position, 1.0f, WHITE);
  } else {
    DrawCube(position, dimensions.x, dimensions.y, dimensions.z, color);
  }
}

BoundingBox Floor::getBoundingBox() const {
  return (BoundingBox){
      (Vector3){position.x - dimensions.x / 2, position.y - dimensions.y / 2,
                position.z - dimensions.z / 2},
      (Vector3){position.x + dimensions.x / 2, position.y + dimensions.y / 2,
                position.z + dimensions.z / 2},
  };
}
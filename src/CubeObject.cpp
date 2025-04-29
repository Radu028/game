#include "CubeObject.h"

#include <algorithm>

#include "exceptions/ResourceException.h"

CubeObject::CubeObject(Vector3 position, float width, float height,
                       float length, Color color, bool hasCollision,
                       const char* texturePath)
    : GameObject(position, hasCollision),
      width(width),
      height(height),
      length(length),
      color(color),
      hasTexture(false) {
  if (texturePath != nullptr) {
    texture = LoadTexture(texturePath);
    if (texture.id == 0) {
      throw ResourceException("Failed to load texture: " +
                              std::string(texturePath));
    }
    hasTexture = true;
  }
}

CubeObject::~CubeObject() {
  if (hasTexture) {
    UnloadTexture(texture);
  }
}

// Copy constructor
CubeObject::CubeObject(const CubeObject& other)
    : GameObject(other.position, other.hasCollision),
      width(other.width),
      height(other.height),
      length(other.length),
      color(other.color),
      hasTexture(false) {
  if (other.hasTexture) {
    // We need to reload the texture since we can't copy it directly
    Image img = LoadImageFromTexture(other.texture);
    texture = LoadTextureFromImage(img);
    UnloadImage(img);
    hasTexture = true;
  }
}

// Assignment operator using copy-and-swap idiom
CubeObject& CubeObject::operator=(CubeObject other) {
  swap(*this, other);
  return *this;
}

// Swap function for copy-and-swap idiom
void swap(CubeObject& first, CubeObject& second) noexcept {
  using std::swap;

  // Swap base class members
  swap(first.position, second.position);
  swap(first.hasCollision, second.hasCollision);

  // Swap CubeObject members
  swap(first.width, second.width);
  swap(first.height, second.height);
  swap(first.length, second.length);
  swap(first.color, second.color);
  swap(first.hasTexture, second.hasTexture);
  swap(first.texture, second.texture);
}

std::shared_ptr<GameObject> CubeObject::clone() const {
  return std::make_shared<CubeObject>(*this);
}

BoundingBox CubeObject::getBoundingBox() const {
  return (BoundingBox){
      (Vector3){position.x - width / 2, position.y - height / 2,
                position.z - length / 2},
      (Vector3){position.x + width / 2, position.y + height / 2,
                position.z + length / 2}};
}

void CubeObject::draw() const {
  if (hasTexture) {
    // DrawCubeTexture doesn't exist in raylib, using DrawCube instead
    DrawCube(position, width, height, length, color);
    // You could add some additional drawing here if needed
  } else {
    DrawCube(position, width, height, length, color);
  }

  // Uncomment if you want to see the bounding box
  // DrawBoundingBox(getBoundingBox(), LIME);
}

void CubeObject::interact() {
  // Basic interaction - change color
  color = (Color){static_cast<unsigned char>(GetRandomValue(0, 255)),
                  static_cast<unsigned char>(GetRandomValue(0, 255)),
                  static_cast<unsigned char>(GetRandomValue(0, 255)), 255};
}

bool CubeObject::checkCollision(const CubeObject& other) const {
  if (!hasCollision || !other.hasCollision) {
    return false;
  }

  BoundingBox box1 = getBoundingBox();
  BoundingBox box2 = other.getBoundingBox();

  return CheckCollisionBoxes(box1, box2);
}
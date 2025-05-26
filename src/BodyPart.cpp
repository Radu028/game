#include "BodyPart.h"

#include <algorithm>
#include <iostream>

#include "raylib.h"
#include "systems/ShaderSystem.h"

BodyPart::BodyPart(Vector3 position, Vector3 size, Color color,
                   bool hasCollision, bool useShaders)
    : GameObject(position, hasCollision, false, false),
      size(size),
      color(color),
      rotationAxis({0.0f, 1.0f, 0.0f}),
      rotationAngle(0.0f),
      useShaders(useShaders),
      name("Body Part"),
      health(100.0f),
      maxHealth(100.0f),
      isInjured(false),
      injuryType("") {
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));

  // If using shaders, assign the lighting shader to the model material
  if (useShaders) {
    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    if (shaderSystem->isInitialized()) {
      model.materials[0].shader = shaderSystem->getShader();
    }
  }
}

// Copy constructor implementation
BodyPart::BodyPart(const BodyPart& other)
    : GameObject(other),
      size(other.size),
      color(other.color),
      rotationAxis(other.rotationAxis),
      rotationAngle(other.rotationAngle),
      useShaders(other.useShaders),
      name(other.name),
      health(other.health),
      maxHealth(other.maxHealth),
      isInjured(other.isInjured),
      injuryType(other.injuryType) {
  std::cout << "BodyPart copy constructor called for: " << name << std::endl;
  // Reload the model for the copy
  model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));
  if (useShaders) {
    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    if (shaderSystem->isInitialized()) {
      model.materials[0].shader = shaderSystem->getShader();
    }
  }
}

// Copy assignment operator using copy-and-swap idiom
BodyPart& BodyPart::operator=(const BodyPart& other) {
  std::cout << "BodyPart copy assignment operator called" << std::endl;
  if (this != &other) {
    BodyPart temp(other);  // Create temporary copy
    swap(*this, temp);     // Swap with temporary
  }
  return *this;
}

// Move constructor
BodyPart::BodyPart(BodyPart&& other) noexcept
    : GameObject(std::move(other)),
      size(other.size),
      color(other.color),
      rotationAxis(other.rotationAxis),
      rotationAngle(other.rotationAngle),
      useShaders(other.useShaders),
      name(std::move(other.name)),
      health(other.health),
      maxHealth(other.maxHealth),
      isInjured(other.isInjured),
      injuryType(std::move(other.injuryType)) {
  std::cout << "BodyPart move constructor called" << std::endl;

  // Move the model
  model = other.model;
  other.model = {};  // Reset moved-from object's model

  // Reset moved-from object to valid state
  other.health = 0.0f;
  other.maxHealth = 0.0f;
  other.isInjured = false;
}

// Move assignment operator
BodyPart& BodyPart::operator=(BodyPart&& other) noexcept {
  std::cout << "BodyPart move assignment operator called" << std::endl;
  if (this != &other) {
    // Move individual members (GameObject base class handling omitted for
    // simplicity)
    size = other.size;
    color = other.color;
    rotationAxis = other.rotationAxis;
    rotationAngle = other.rotationAngle;
    useShaders = other.useShaders;
    name = std::move(other.name);
    health = other.health;
    maxHealth = other.maxHealth;
    isInjured = other.isInjured;
    injuryType = std::move(other.injuryType);

    // Move the model
    if (model.meshCount > 0) UnloadModel(model);
    model = other.model;
    other.model = {};

    // Reset moved-from object
    other.health = 0.0f;
    other.maxHealth = 0.0f;
    other.isInjured = false;
  }
  return *this;
}

// Destructor implementation
BodyPart::~BodyPart() {
  if (model.meshCount > 0) {
    UnloadModel(model);
  }
}

// Friend swap function for copy-and-swap idiom
void swap(BodyPart& first, BodyPart& second) noexcept {
  using std::swap;
  // Note: GameObject base class doesn't have swap, so we swap individual
  // members
  swap(first.size, second.size);
  swap(first.color, second.color);
  swap(first.rotationAxis, second.rotationAxis);
  swap(first.rotationAngle, second.rotationAngle);
  swap(first.useShaders, second.useShaders);
  swap(first.model, second.model);
  swap(first.name, second.name);
  swap(first.health, second.health);
  swap(first.maxHealth, second.maxHealth);
  swap(first.isInjured, second.isInjured);
  swap(first.injuryType, second.injuryType);
}

void BodyPart::setRotation(const Vector3 axis, float angle) {
  rotationAxis = axis;
  rotationAngle = angle;
}

void BodyPart::setPosition(Vector3 newPos) { GameObject::setPosition(newPos); }

Vector3 BodyPart::getSize() const { return size; }
Color BodyPart::getColor() const { return color; }

float BodyPart::getRotationAngle() const { return rotationAngle; }
Vector3 BodyPart::getRotationAxis() const { return rotationAxis; }

void BodyPart::draw() const {
  if (useShaders) {
    // With shaders, the material already has the shader assigned
    DrawModelEx(model, position, rotationAxis, rotationAngle,
                {1.0f, 1.0f, 1.0f}, color);
  } else {
    // Fallback to normal rendering
    DrawModelEx(model, position, rotationAxis, rotationAngle,
                {1.0f, 1.0f, 1.0f}, color);
  }
}

BoundingBox BodyPart::getBoundingBox() const {
  Vector3 currentPosition = getPosition();
  float halfX = size.x * 0.5f;
  float halfY = size.y * 0.5f;
  float halfZ = size.z * 0.5f;

  return (BoundingBox){
      (Vector3){currentPosition.x - halfX, currentPosition.y - halfY,
                currentPosition.z - halfZ},
      (Vector3){currentPosition.x + halfX, currentPosition.y + halfY,
                currentPosition.z + halfZ},
  };
}

std::unique_ptr<GameObject> BodyPart::clone() const {
  return std::make_unique<BodyPart>(*this);
}

// Health-related methods implementation
void BodyPart::takeDamage(float damage) {
  health = std::max(0.0f, health - damage);
  if (health <= maxHealth * 0.3f) {
    isInjured = true;
    injuryType = "Severe damage";
  }
}

void BodyPart::heal(float amount) {
  health = std::min(maxHealth, health + amount);
  if (health > maxHealth * 0.7f) {
    isInjured = false;
    injuryType = "";
  }
}

float BodyPart::getHealthPercentage() const {
  return maxHealth > 0 ? (health / maxHealth) * 100.0f : 0.0f;
}

void BodyPart::displayStatus() const {
  std::cout << "BodyPart: " << name << " | Health: " << health << "/"
            << maxHealth << " (" << getHealthPercentage() << "%)"
            << " | Injured: " << (isInjured ? "Yes" : "No");
  if (isInjured && !injuryType.empty()) {
    std::cout << " (" << injuryType << ")";
  }
  std::cout << std::endl;
}

// Demonstration function for copy operations
void demonstrateBodyPartCopyOperations() {
  std::cout << "\n=== BodyPart Copy Operations Demonstration ===" << std::endl;

  BodyPart original({0, 0, 0}, {1, 1, 1}, RED, false, false);
  original.setName("Left Arm");
  original.setHealth(80.0f, 100.0f);
  original.takeDamage(30.0f);
  std::cout << "Original: ";
  original.displayStatus();

  // Test copy constructor
  BodyPart copied(original);
  std::cout << "Copied: ";
  copied.displayStatus();

  // Test copy assignment
  BodyPart assigned({1, 1, 1}, {1, 1, 1}, BLUE, false, false);
  assigned.setName("Right Leg");
  assigned.setHealth(100.0f, 100.0f);
  assigned = original;
  std::cout << "Assigned: ";
  assigned.displayStatus();

  std::cout << "=============================================" << std::endl;
}
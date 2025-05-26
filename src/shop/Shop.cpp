#include "shop/Shop.h"

#include <random>

#include "GameWorld.h"
#include "objects/Wall.h"
#include "raymath.h"

Shop::Shop(Vector3 position, Vector3 size)
    : CubeObject(position, size, BEIGE, false, "", false, false,
                 false),  // Make shop cube invisible
      entranceSize({2.0f, 3.0f, 0.5f}) {
  // Calculate entrance position (front center of shop, but closer for easier
  // access)
  entrancePosition = {
      position.x,  // Center X of the shop
      position.y,  // Same Y as shop center
      position.z + size.z / 2.0f +
          2.0f  // Moved further from shop front for easier access
  };

  interiorBounds = calculateInteriorBounds();

  buildWalls();
  createShelves();
  stockShelves();
}

void Shop::buildWalls() {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  float wallThickness = 0.3f;
  float wallHeight = shopSize.y;

  // Calculate entrance dimensions
  float entranceWidth = entranceSize.x;
  float frontWallWidth = (shopSize.x - entranceWidth) / 2.0f;

  // Front wall (left side of entrance)
  addWall({shopPos.x - shopSize.x / 2.0f + frontWallWidth / 2.0f, shopPos.y,
           shopPos.z + shopSize.z / 2.0f},
          {frontWallWidth, wallHeight, wallThickness});

  // Front wall (right side of entrance)
  addWall({shopPos.x + shopSize.x / 2.0f - frontWallWidth / 2.0f, shopPos.y,
           shopPos.z + shopSize.z / 2.0f},
          {frontWallWidth, wallHeight, wallThickness});

  // Back wall
  addWall({shopPos.x, shopPos.y, shopPos.z - shopSize.z / 2.0f},
          {shopSize.x, wallHeight, wallThickness});

  // Left wall
  addWall({shopPos.x - shopSize.x / 2.0f, shopPos.y, shopPos.z},
          {wallThickness, wallHeight, shopSize.z});

  // Right wall
  addWall({shopPos.x + shopSize.x / 2.0f, shopPos.y, shopPos.z},
          {wallThickness, wallHeight, shopSize.z});

  // Add roof to complete the shop structure
  addWall({shopPos.x, shopPos.y + shopSize.y / 2.0f, shopPos.z},
          {shopSize.x, wallThickness, shopSize.z});

  // Add all walls to the game world
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    for (auto& wall : walls) {
      world->addObject(wall);
    }
  }
}

void Shop::createShelves() {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  // Create shelves along the walls
  float shelfHeight = 1.0f;

  // Left side shelves
  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {shopPos.x - shopSize.x / 2.0f + 1.0f,
                        shopPos.y - shopSize.y / 2.0f + shelfHeight,
                        shopPos.z - shopSize.z / 4.0f + i * shopSize.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  // Right side shelves
  for (int i = 0; i < 2; ++i) {
    Vector3 shelfPos = {shopPos.x + shopSize.x / 2.0f - 1.0f,
                        shopPos.y - shopSize.y / 2.0f + shelfHeight,
                        shopPos.z - shopSize.z / 4.0f + i * shopSize.z / 2.0f};
    auto shelf = std::make_shared<Shelf>(shelfPos);
    shelves.push_back(shelf);
  }

  // Center shelf
  Vector3 centerShelfPos = {
      shopPos.x, shopPos.y - shopSize.y / 2.0f + shelfHeight, shopPos.z - 1.0f};
  auto centerShelf = std::make_shared<Shelf>(centerShelfPos);
  shelves.push_back(centerShelf);

  // Add shelves to game world
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    for (auto& shelf : shelves) {
      world->addObject(shelf);
    }
  }
}

void Shop::stockShelves() {
  for (auto& shelf : shelves) {
    shelf->restockAllFruits();
  }
}

bool Shop::isInsideShop(Vector3 position) const {
  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  return (position.x >= shopPos.x - shopSize.x / 2.0f &&
          position.x <= shopPos.x + shopSize.x / 2.0f &&
          position.z >= shopPos.z - shopSize.z / 2.0f &&
          position.z <= shopPos.z + shopSize.z / 2.0f &&
          position.y >= shopPos.y - shopSize.y / 2.0f &&
          position.y <= shopPos.y + shopSize.y / 2.0f);
}

bool Shop::isNearEntrance(Vector3 position, float threshold) const {
  return Vector3Distance(position, entrancePosition) <= threshold;
}

Vector3 Shop::getRandomInteriorPosition() const {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  Vector3 shopPos = getPosition();
  Vector3 shopSize = getSize();

  // Create safer bounds to avoid walls and shelves
  float safeMargin = 2.0f;  // Increased margin to avoid walls

  std::uniform_real_distribution<float> xDist(
      shopPos.x - shopSize.x / 2.0f + safeMargin,
      shopPos.x + shopSize.x / 2.0f - safeMargin);
  std::uniform_real_distribution<float> zDist(
      shopPos.z - shopSize.z / 2.0f + safeMargin,
      shopPos.z + shopSize.z / 2.0f - safeMargin);

  return {xDist(gen), shopPos.y - shopSize.y / 2.0f + 1.0f, zDist(gen)};
}

void Shop::restockAllShelves() {
  for (auto& shelf : shelves) {
    shelf->restockAllFruits();
  }
}

int Shop::getTotalFruitCount() const {
  int total = 0;
  for (const auto& shelf : shelves) {
    total += shelf->getFruitCount();
  }
  return total;
}

bool Shop::isEmpty() const { return getTotalFruitCount() == 0; }

std::shared_ptr<Fruit> Shop::findNearestFruit(Vector3 position) {
  std::shared_ptr<Fruit> nearest = nullptr;
  float minDistance = std::numeric_limits<float>::max();

  for (auto& shelf : shelves) {
    auto fruit = shelf->getNearestFruit(position);
    if (fruit) {
      float distance = Vector3Distance(position, fruit->getPosition());
      if (distance < minDistance) {
        minDistance = distance;
        nearest = fruit;
      }
    }
  }

  return nearest;
}

void Shop::interact() {
  // Shop interaction could show status or restock
  restockAllShelves();
}

void Shop::update(float deltaTime) {
  CubeObject::update(deltaTime);

  // Update all shelves
  for (auto& shelf : shelves) {
    shelf->update(deltaTime);
  }
}

std::unique_ptr<GameObject> Shop::clone() const {
  return std::make_unique<Shop>(*this);
}

void Shop::addWall(Vector3 position, Vector3 size, Color color) {
  auto wall = std::make_shared<CubeObject>(position, size, color, true, "",
                                           false, true, true);
  walls.push_back(wall);
}

Vector3 Shop::calculateInteriorBounds() const {
  Vector3 shopSize = getSize();
  return {
      shopSize.x - 2.0f,  // Leave space for walls
      shopSize.y - 1.0f,  // Leave space for floor/ceiling
      shopSize.z - 2.0f   // Leave space for walls
  };
}

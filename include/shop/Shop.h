#pragma once

#include <memory>
#include <vector>

#include "objects/CubeObject.h"
#include "raylib.h"
#include "shop/Shelf.h"

// Shop building with walls, entrance, and interior shelves
class Shop : public CubeObject {
 private:
  std::vector<std::shared_ptr<CubeObject>> walls;
  std::vector<std::shared_ptr<Shelf>> shelves;
  Vector3 entrancePosition;
  Vector3 entranceSize;
  Vector3 interiorBounds;

 public:
  Shop(Vector3 position, Vector3 size = {12.0f, 4.0f, 8.0f});
  ~Shop() override = default;

  // Shop setup
  void buildWalls();
  void createShelves();
  void stockShelves();

  // Access methods
  const std::vector<std::shared_ptr<Shelf>>& getShelves() const {
    return shelves;
  }
  const std::vector<std::shared_ptr<CubeObject>>& getWalls() const {
    return walls;
  }

  // Position checking
  bool isInsideShop(Vector3 position) const;
  bool isNearEntrance(Vector3 position, float threshold = 1.0f) const;
  Vector3 getEntrancePosition() const { return entrancePosition; }
  Vector3 getRandomInteriorPosition() const;

  // Shop management
  void restockAllShelves();
  int getTotalFruitCount() const;
  bool isEmpty() const;
  std::shared_ptr<Fruit> findNearestFruit(Vector3 position);

  // Override methods
  void interact() override;
  void update(float deltaTime) override;
  std::unique_ptr<GameObject> clone() const override;

 private:
  void addWall(Vector3 position, Vector3 size, Color color = DARKBROWN);
  Vector3 calculateInteriorBounds() const;
};

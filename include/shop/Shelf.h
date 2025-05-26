#pragma once

#include <memory>
#include <vector>

#include "objects/CubeObject.h"
#include "raylib.h"
#include "shop/Fruit.h"

// Shelf class for displaying and storing fruits
class Shelf : public CubeObject {
 private:
  std::vector<std::shared_ptr<Fruit>> fruits;
  Vector3 shelfPosition;
  int maxFruits;
  float fruitSpacing;

 public:
  Shelf(Vector3 position, Vector3 size = {2.0f, 0.2f, 0.8f},
        Color color = BROWN);
  ~Shelf() override = default;

  // Fruit management
  void addFruit(std::shared_ptr<Fruit> fruit);
  void removeFruit(std::shared_ptr<Fruit> fruit);
  bool hasFruit(FruitType type) const;
  std::shared_ptr<Fruit> getFruit(FruitType type);
  std::shared_ptr<Fruit> getNearestFruit(Vector3 position);

  // Stock management
  void restockShelf(FruitType type, int quantity);
  void restockAllFruits();
  int getFruitCount() const;
  int getFruitCount(FruitType type) const;
  bool isEmpty() const;

  // Getters
  const std::vector<std::shared_ptr<Fruit>>& getFruits() const {
    return fruits;
  }

  // Shelf interaction
  void interact() override;

  // Update method for shelf-specific logic
  void update(float deltaTime) override;

  // Clone for polymorphism
  std::unique_ptr<GameObject> clone() const override;

 private:
  Vector3 calculateFruitPosition(int index) const;
  void arrangeFruits();
};

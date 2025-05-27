#include "utils/Storage.h"

#include <iostream>

#include "shop/Fruit.h"

template <>
void Storage<Fruit>::displaySpecialInfo() const {
  std::cout << "=== Fruit Storage Special Information ===" << std::endl;
  std::cout << "Total fruits stored: " << items.size() << std::endl;

  int pickedCount = 0;
  int availableCount = 0;
  float totalValue = 0.0f;

  for (const auto& fruit : items) {
    if (fruit->getIsPicked()) {
      pickedCount++;
    } else {
      availableCount++;
    }
    totalValue += fruit->getPrice();
  }

  std::cout << "Available fruits: " << availableCount << std::endl;
  std::cout << "Picked fruits: " << pickedCount << std::endl;
  std::cout << "Total inventory value: $" << totalValue << std::endl;
  std::cout << "Average fruit price: $"
            << (items.empty() ? 0.0f : totalValue / items.size()) << std::endl;
  std::cout << "=========================================" << std::endl;
}

template <>
double Storage<Fruit>::calculateTotalNutrition() const {
  // For the existing Fruit class, we'll calculate based on type and
  // availability
  double totalNutrition = 0.0;
  for (const auto& fruit : items) {
    if (!fruit->getIsPicked()) {  // Only available fruits provide nutrition
      // Simple nutrition calculation based on fruit type
      switch (fruit->getType()) {
        case FruitType::APPLE:
          totalNutrition += 4.0;
          break;
        case FruitType::FRUIT_ORANGE:
          totalNutrition += 6.0;
          break;
        case FruitType::BANANA:
          totalNutrition += 5.0;
          break;
        case FruitType::GRAPE:
          totalNutrition += 3.0;
          break;
        case FruitType::STRAWBERRY:
          totalNutrition += 2.5;
          break;
      }
    }
  }
  return totalNutrition;
}

// Explicit template instantiation for GameObject-derived types only
template class Storage<Fruit>;

void demonstrateStorage() {
  std::cout << "Storage template demonstration:" << std::endl;

  Storage<Fruit> fruitStorage({0, 0, 0}, "Fruit Storage");

  auto apple = std::make_shared<Fruit>(Vector3{0, 0, 0}, FruitType::APPLE);
  auto banana = std::make_shared<Fruit>(Vector3{1, 1, 1}, FruitType::BANANA);

  fruitStorage.addItem(apple);
  fruitStorage.addItem(banana);
  fruitStorage.displaySpecialInfo();

  std::cout << "Fruit storage capacity: " << fruitStorage.getMaxCapacity()
            << std::endl;
  std::cout << "Total nutrition value: "
            << fruitStorage.calculateTotalNutrition() << std::endl;
}

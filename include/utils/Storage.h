#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

#include "exceptions/GameExceptions.h"
#include "objects/CubeObject.h"
#include "raylib.h"
#include "shop/Fruit.h"

// Forward declaration
class GameObject;

// Template Storage class for displaying and storing various types of items
template <typename ItemType>
class Storage : public CubeObject {
  static_assert(std::is_base_of_v<GameObject, ItemType>,
                "ItemType must be derived from GameObject");

 private:
  std::vector<std::shared_ptr<ItemType>> items;
  Vector3 storagePosition;
  int maxItems;
  float itemSpacing;
  std::string storageType;

  // Static member to track total storage units created
  static int totalStorageUnits;

 public:
  // Default constructor
  Storage() : Storage({0, 0, 0}, "Default") {}

  Storage(Vector3 position, const std::string& type = "Generic",
          Vector3 size = {2.0f, 0.2f, 0.8f}, Color color = BROWN,
          int capacity = 10)
      : CubeObject(position, size, color, true),
        storagePosition(position),
        maxItems(capacity),
        itemSpacing(0.3f),
        storageType(type) {
    items.reserve(maxItems);
    ++totalStorageUnits;
  }

  ~Storage() override = default;

  // Copy constructor with deep copying
  Storage(const Storage& other)
      : CubeObject(other),
        storagePosition(other.storagePosition),
        maxItems(other.maxItems),
        itemSpacing(other.itemSpacing),
        storageType(other.storageType + "_copy") {
    items.reserve(other.items.size());
    for (const auto& item : other.items) {
      if (item) {
        // For template safety, create a copy using the shared_ptr copy
        // constructor Note: This creates a shallow copy of the pointed-to
        // object For true deep copy, the ItemType class should implement a
        // proper copy constructor
        items.push_back(std::make_shared<ItemType>(*item));
      }
    }
    ++totalStorageUnits;
  }

  // Assignment operator using copy-and-swap idiom
  Storage& operator=(Storage other) {
    swap(*this, other);
    return *this;
  }

  // Move constructor
  Storage(Storage&& other) noexcept
      : CubeObject(std::move(other)),
        items(std::move(other.items)),
        storagePosition(other.storagePosition),
        maxItems(other.maxItems),
        itemSpacing(other.itemSpacing),
        storageType(std::move(other.storageType)) {
    other.maxItems = 0;
    other.itemSpacing = 0.0f;
  }

  // Friend function for swap (part of copy-and-swap idiom)
  friend void swap(Storage& first, Storage& second) noexcept {
    using std::swap;
    // Swap base class
    swap(static_cast<CubeObject&>(first), static_cast<CubeObject&>(second));
    // Swap derived class members
    swap(first.items, second.items);
    swap(first.storagePosition, second.storagePosition);
    swap(first.maxItems, second.maxItems);
    swap(first.itemSpacing, second.itemSpacing);
    swap(first.storageType, second.storageType);
  }

  // Template-dependent item management
  void addItem(std::shared_ptr<ItemType> item) {
    if (!item) {
      throw GameException("Cannot add null item to " + storageType +
                          " storage");
    }

    if (static_cast<int>(items.size()) >= maxItems) {
      throw GameException(storageType + " storage is full (capacity: " +
                          std::to_string(maxItems) + ")");
    }

    items.push_back(item);
    arrangeItems();
  }

  bool removeItem(std::shared_ptr<ItemType> item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
      items.erase(it);
      arrangeItems();
      return true;
    }
    return false;
  }

  // Template method to find items by predicate
  template <typename Predicate>
  std::vector<std::shared_ptr<ItemType>> findItems(Predicate pred) const {
    std::vector<std::shared_ptr<ItemType>> result;
    std::copy_if(items.begin(), items.end(), std::back_inserter(result), pred);
    return result;
  }

  // Template method to find nearest item to a position
  std::shared_ptr<ItemType> getNearestItem(Vector3 position) const {
    if (items.empty()) {
      return nullptr;
    }

    std::shared_ptr<ItemType> nearest = nullptr;
    float minDistance = std::numeric_limits<float>::max();

    for (const auto& item : items) {
      if (!item) continue;

      Vector3 itemPos = item->getPosition();
      float dx = position.x - itemPos.x;
      float dy = position.y - itemPos.y;
      float dz = position.z - itemPos.z;
      float distance = dx * dx + dy * dy + dz * dz;

      if (distance < minDistance) {
        minDistance = distance;
        nearest = item;
      }
    }

    return nearest;
  }

  // Template method for type-safe access to specific derived types
  template <typename DerivedType>
  std::vector<std::shared_ptr<DerivedType>> getItemsOfType() const {
    static_assert(std::is_base_of_v<ItemType, DerivedType>,
                  "DerivedType must be derived from ItemType");

    std::vector<std::shared_ptr<DerivedType>> result;
    for (const auto& item : items) {
      if (auto derived = std::dynamic_pointer_cast<DerivedType>(item)) {
        result.push_back(derived);
      }
    }
    return result;
  }

  // Template method for applying functions to all items
  template <typename Function>
  void forEachItem(Function func) {
    std::for_each(items.begin(), items.end(), func);
  }

  // Stock management with template specialization support
  template <typename SpecificType = ItemType>
  void restockItems(int quantity) {
    for (int i = 0; i < quantity && static_cast<int>(items.size()) < maxItems;
         ++i) {
      try {
        // This would need to be specialized for specific types
        auto newItem = createDefaultItem<SpecificType>();
        if (newItem) {
          addItem(newItem);
        }
      } catch (const GameException& e) {
        // Log error but continue with other items
        break;
      }
    }
  }

  // Virtual functions that depend on template parameter
  void update(float deltaTime) override {
    // Update all stored items
    for (auto& item : items) {
      if (item) {
        item->update(deltaTime);
      }
    }
  }

  void interact() override {
    if (!items.empty() && items.back()) {
      // Interact with the last item (most recently added)
      items.back()->interact();
    }
  }

  std::unique_ptr<GameObject> clone() const override {
    return std::make_unique<Storage<ItemType>>(*this);
  }

  // Getters
  const std::vector<std::shared_ptr<ItemType>>& getItems() const {
    return items;
  }
  int getItemCount() const { return static_cast<int>(items.size()); }
  int getMaxCapacity() const { return maxItems; }
  bool isEmpty() const { return items.empty(); }
  bool isFull() const { return static_cast<int>(items.size()) >= maxItems; }
  const std::string& getStorageType() const { return storageType; }
  float getItemSpacing() const { return itemSpacing; }

  // Methods for specialized functionality (can be specialized for specific
  // types)
  virtual void displaySpecialInfo() const {
    std::cout << "Generic storage containing " << items.size() << " items"
              << std::endl;
  }

  virtual double calculateTotalNutrition() const {
    return 0.0;  // Default implementation
  }

  // Static method
  static int getTotalStorageUnits() { return totalStorageUnits; }

  // Setters
  void setItemSpacing(float spacing) {
    itemSpacing = spacing;
    arrangeItems();
  }
  void setMaxItems(int capacity) {
    if (capacity < 0) {
      throw GameException("Storage capacity cannot be negative");
    }
    maxItems = capacity;
  }

 private:
  Vector3 calculateItemPosition(int index) const {
    // Arrange items in a grid pattern on the storage surface
    int itemsPerRow =
        std::max(1, static_cast<int>((getSize().x - 0.2f) / itemSpacing));
    int row = index / itemsPerRow;
    int col = index % itemsPerRow;

    Vector3 basePos = getPosition();
    float startX = basePos.x - (getSize().x * 0.4f);
    float startZ = basePos.z - (getSize().z * 0.4f);

    return {startX + col * itemSpacing,
            basePos.y + getSize().y * 0.5f +
                0.1f,  // Slightly above storage surface
            startZ + row * itemSpacing};
  }

  void arrangeItems() {
    for (size_t i = 0; i < items.size(); ++i) {
      if (items[i]) {
        Vector3 newPos = calculateItemPosition(static_cast<int>(i));
        items[i]->setPosition(newPos);
      }
    }
  }

  // Template helper method for creating default items (to be specialized)
  template <typename T = ItemType>
  std::shared_ptr<T> createDefaultItem() {
    // This should be specialized for specific types
    // Default implementation throws exception
    throw GameException("Cannot create default item for type " + storageType +
                        ". Template specialization required.");
  }
};

// Initialize static member for template class
template <typename ItemType>
int Storage<ItemType>::totalStorageUnits = 0;

// Type alias for backward compatibility with existing Fruit-based shelf
class Fruit;  // Forward declaration
using FruitShelf = Storage<Fruit>;

// Template specialization for creating default fruits
template <>
template <>
inline std::shared_ptr<Fruit> Storage<Fruit>::createDefaultItem<Fruit>() {
  // This would create a default fruit - implementation would depend on Fruit
  // constructor
  Vector3 defaultPos = calculateItemPosition(static_cast<int>(items.size()));
  // Note: This is a placeholder - actual implementation would need proper Fruit
  // constructor
  return nullptr;  // Would return std::make_shared<Fruit>(defaultPos,
                   // FruitType::APPLE);
}

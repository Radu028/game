#ifndef GAMEWORLD_H
#define GAMEWORLD_H

#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "exceptions/GameExceptions.h"
#include "objects/GameObject.h"

class PhysicsSystem;
class GameObject;
class Player;
class btDiscreteDynamicsWorld;

class GameWorld {
 private:
  static GameWorld* instance;
  static size_t totalObjectsCreated;
  static size_t totalObjectsDestroyed;
  static std::unordered_map<std::string, size_t> objectTypeCount;
  static std::string creationTimestamp;

  std::vector<std::shared_ptr<GameObject>> objects;
  GameObject* player;
  std::unique_ptr<PhysicsSystem> physicsSystem;
  std::string worldName;

  GameWorld(GameObject* player, const std::string& name = "DefaultWorld");

  // Delete copy constructor and assignment operator for singleton
  GameWorld(const GameWorld&) = delete;
  GameWorld& operator=(const GameWorld&) = delete;
  ~GameWorld();

 public:
  // Singleton access
  static GameWorld* getInstance(GameObject* player = nullptr);
  static void destroyInstance();

  // Static utility functions
  static size_t getTotalObjectsCreated() { return totalObjectsCreated; }
  static size_t getTotalObjectsDestroyed() { return totalObjectsDestroyed; }
  static size_t getActiveObjectCount() {
    return totalObjectsCreated - totalObjectsDestroyed;
  }
  static size_t getTotalObjectCount() { return totalObjectsCreated; }
  static const std::unordered_map<std::string, size_t>& getObjectTypeCount() {
    return objectTypeCount;
  }
  static const std::string& getCreationTimestamp() { return creationTimestamp; }
  static void resetStatistics();

  // Template method for type-safe object retrieval with dynamic casting
  template <typename T>
  std::vector<std::shared_ptr<T>> findObjectsOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    std::vector<std::shared_ptr<T>> result;
    for (const auto& obj : objects) {
      if (obj) {
        if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
          result.push_back(castedObj);
        }
      }
    }
    return result;
  }

  // Template method for finding first object of specific type
  template <typename T>
  std::shared_ptr<T> findFirstObjectOfType() const {
    static_assert(std::is_base_of_v<GameObject, T>,
                  "T must be derived from GameObject");

    for (const auto& obj : objects) {
      if (obj) {
        if (auto castedObj = std::dynamic_pointer_cast<T>(obj)) {
          return castedObj;
        }
      }
    }
    return nullptr;
  }

  // Object management
  void addObject(std::shared_ptr<GameObject> object);
  void removeObject(std::shared_ptr<GameObject> object);
  void removeObjectsOfType(const std::string& typeName);
  void clearAllObjects();

  // World operations
  void update(float deltaTime);
  void draw() const;

  // Physics access
  btDiscreteDynamicsWorld* getBulletWorld() const;
  btDiscreteDynamicsWorld* getDynamicsWorld() const { return getBulletWorld(); }

  // Getters
  const std::vector<std::shared_ptr<GameObject>>& getObjects() const {
    return objects;
  }
  size_t getObjectCount() const { return objects.size(); }
  const std::string& getWorldName() const { return worldName; }
  GameObject* getPlayer() const { return player; }

  // Setters
  void setWorldName(const std::string& name) { worldName = name; }

 private:
  static void incrementObjectCount(const std::string& typeName);
  static void decrementObjectCount(const std::string& typeName);
  std::string getObjectTypeName(const GameObject* obj) const;
};

#endif
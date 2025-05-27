#pragma once

#include <memory>
#include <random>
#include <vector>

#include "ai/NPC.h"
#include "raylib.h"
#include "shop/Shop.h"

// Forward declaration
class NPCChatSystem;

// Singleton pattern for managing NPC spawning and lifecycle
class NPCManager : public NPCObserver {
 private:
  static NPCManager* instance;

  std::vector<std::shared_ptr<NPC>> activeNPCs;
  std::shared_ptr<Shop> shop;
  NPCChatSystem* chatSystem;

  // Spawning parameters
  float spawnTimer;
  float spawnInterval;
  float minSpawnInterval;
  float maxSpawnInterval;
  int maxActiveNPCs;

  // Statistics
  int totalNPCsSpawned;
  int totalFruitsPicked;
  int totalNPCsExited;

  // Random number generation
  std::random_device rd;
  mutable std::mt19937 gen;

  NPCManager();

 public:
  ~NPCManager();

  // Singleton access
  static NPCManager* getInstance();
  static void destroyInstance();

  // Setup
  void setShop(std::shared_ptr<Shop> targetShop);
  void setChatSystem(NPCChatSystem* chat) { chatSystem = chat; }

  // Main update loop
  void update(float deltaTime);

  // NPC management
  void spawnNPC();
  void removeInactiveNPCs();
  std::shared_ptr<NPC> createNPC();

  void onNPCFruitPicked(NPC* npc, std::shared_ptr<Fruit> fruit) override;
  void onNPCExited(NPC* npc) override;
  void onNPCEnteredShop(NPC* npc) override;

  void setSpawnInterval(float min, float max);
  void setMaxActiveNPCs(int max) { maxActiveNPCs = max; }

  const std::vector<std::shared_ptr<NPC>>& getActiveNPCs() const {
    return activeNPCs;
  }
  int getActiveNPCCount() const { return activeNPCs.size(); }
  int getTotalNPCsSpawned() const { return totalNPCsSpawned; }
  int getTotalFruitsPicked() const { return totalFruitsPicked; }
  int getTotalNPCsExited() const { return totalNPCsExited; }
  Vector3 getRandomSpawnPosition() const;
  bool shouldSpawnNPC() const;

 private:
  void updateSpawnTimer(float deltaTime);
  void cleanupNPCs();
};

#include "ai/NPCManager.h"

#include <algorithm>
#include <iostream>

#include "GameWorld.h"

NPCManager* NPCManager::instance = nullptr;

NPCManager::NPCManager()
    : spawnTimer(0.0f),
      spawnInterval(5.0f),
      minSpawnInterval(3.0f),
      maxSpawnInterval(8.0f),
      maxActiveNPCs(1),  // Only spawn 1 NPC for debugging
      chatSystem(nullptr),
      totalNPCsSpawned(0),
      totalFruitsPicked(0),
      totalNPCsExited(0),
      gen(rd()) {}

NPCManager::~NPCManager() {
  // Clean up any remaining NPCs
  for (auto& npc : activeNPCs) {
    npc->removeObserver(this);
  }
  activeNPCs.clear();
}

NPCManager* NPCManager::getInstance() {
  if (instance == nullptr) {
    instance = new NPCManager();
  }
  return instance;
}

void NPCManager::destroyInstance() {
  delete instance;
  instance = nullptr;
}

void NPCManager::setShop(std::shared_ptr<Shop> targetShop) {
  shop = targetShop;
}

void NPCManager::update(float deltaTime) {
  updateSpawnTimer(deltaTime);

  // Update all active NPCs
  for (auto& npc : activeNPCs) {
    npc->update(deltaTime);
  }

  // Remove inactive NPCs
  removeInactiveNPCs();

  // Spawn new NPCs if needed
  if (shouldSpawnNPC()) {
    spawnNPC();
  }
}

void NPCManager::spawnNPC() {
  if (!shop || activeNPCs.size() >= maxActiveNPCs) {
    return;
  }

  auto newNPC = createNPC();
  if (newNPC) {
    // Set the chat system for the NPC
    if (chatSystem) {
      newNPC->setChatSystem(chatSystem);
    }

    activeNPCs.push_back(newNPC);
    newNPC->addObserver(this);

    // Add to game world
    if (GameWorld* world = GameWorld::getInstance(nullptr)) {
      world->addObject(newNPC);

      // Setup physics for the NPC
      newNPC->setupPhysics(world->getDynamicsWorld());
    }

    totalNPCsSpawned++;

    // Debug output to confirm NPC spawning
    Vector3 spawnPos = newNPC->getPosition();
    std::cout << "🧑 NPC #" << totalNPCsSpawned << " spawned at (" << spawnPos.x
              << ", " << spawnPos.y << ", " << spawnPos.z
              << ") - Total active: " << activeNPCs.size() << std::endl;

    // Reset spawn timer with random interval
    std::uniform_real_distribution<float> intervalDist(minSpawnInterval,
                                                       maxSpawnInterval);
    spawnInterval = intervalDist(gen);
    spawnTimer = 0.0f;
  }
}

void NPCManager::removeInactiveNPCs() {
  auto it = std::remove_if(
      activeNPCs.begin(), activeNPCs.end(),
      [this](const std::shared_ptr<NPC>& npc) {
        if (!npc->getIsActive()) {
          npc->removeObserver(this);

          // Remove from game world
          if (GameWorld* world = GameWorld::getInstance(nullptr)) {
            // Remove physics first
            npc->removeFromPhysics(world->getDynamicsWorld());
            world->removeObject(npc);
          }

          return true;
        }
        return false;
      });

  activeNPCs.erase(it, activeNPCs.end());
}

std::shared_ptr<NPC> NPCManager::createNPC() {
  if (!shop) {
    return nullptr;
  }

  Vector3 spawnPos = getRandomSpawnPosition();
  auto npc = std::make_shared<NPC>(spawnPos, shop);

  return npc;
}

void NPCManager::onNPCFruitPicked(NPC* npc, std::shared_ptr<Fruit> fruit) {
  totalFruitsPicked++;

  // Optional: Log or handle fruit picking event
  if (fruit) {
    std::cout << "NPC picked " << fruit->getName()
              << " (Total picked: " << totalFruitsPicked << ")" << std::endl;
  }
}

void NPCManager::onNPCExited(NPC* npc) {
  totalNPCsExited++;

  // Optional: Log NPC exit
  std::cout << "NPC exited shop (Total exited: " << totalNPCsExited << ")"
            << std::endl;
}

void NPCManager::onNPCEnteredShop(NPC* npc) {
  // Optional: Log NPC entry
  std::cout << "NPC entered shop" << std::endl;
}

void NPCManager::setSpawnInterval(float min, float max) {
  minSpawnInterval = min;
  maxSpawnInterval = max;

  // Update current interval if needed
  std::uniform_real_distribution<float> intervalDist(minSpawnInterval,
                                                     maxSpawnInterval);
  spawnInterval = intervalDist(gen);
}

Vector3 NPCManager::getRandomSpawnPosition() const {
  // Spawn NPC closer to the shop for debugging purposes
  std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * PI);
  std::uniform_real_distribution<float> distanceDist(
      8.0f, 12.0f);  // Much closer for debugging

  float angle = angleDist(gen);
  float distance = distanceDist(gen);

  Vector3 shopPos = shop ? shop->getPosition() : Vector3{0, 0, 0};

  // For debugging, prefer spawning in front of the shop where pathfinding is
  // easier
  if (shop) {
    // Always spawn in front of shop for easier debugging
    angle =
        PI + (angleDist(gen) - PI) * 0.2f;  // Very narrow angle range in front
  }

  return {shopPos.x + cos(angle) * distance,
          1.0f,  // Spawn at ground level where feet should be
          shopPos.z + sin(angle) * distance};
}

bool NPCManager::shouldSpawnNPC() const {
  return spawnTimer >= spawnInterval && activeNPCs.size() < maxActiveNPCs &&
         shop != nullptr;
}

void NPCManager::updateSpawnTimer(float deltaTime) { spawnTimer += deltaTime; }

void NPCManager::cleanupNPCs() {
  for (auto& npc : activeNPCs) {
    npc->removeObserver(this);
    npc->setActive(false);
  }
  activeNPCs.clear();
}

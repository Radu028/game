#include "game/FruitShopGame.h"

#include <iostream>

#include "GameWorld.h"
#include "ai/NPCManager.h"

FruitShopGame* FruitShopGame::instance = nullptr;

FruitShopGame::FruitShopGame()
    : currentState(GameState::STARTING),
      npcManager(nullptr),
      chatSystem(std::make_unique<NPCChatSystem>()),
      initialStockPerFruit(10),
      totalFruitsInStock(0),
      gameStarted(false),
      gameTimer(0.0f),
      fruitsPickedByNPCs(0),
      npcsSoFar(0) {}

FruitShopGame::~FruitShopGame() { shutdown(); }

FruitShopGame* FruitShopGame::getInstance() {
  if (instance == nullptr) {
    instance = new FruitShopGame();
  }
  return instance;
}

void FruitShopGame::destroyInstance() {
  delete instance;
  instance = nullptr;
}

void FruitShopGame::initialize() {
  initializeShop();

  if (GameWorld* world = GameWorld::getInstance()) {
    world->initializeNavMesh();
    world->finalizeObstacles();
  }

  npcManager = NPCManager::getInstance();
  if (npcManager && shop) {
    npcManager->setShop(shop);
    npcManager->setChatSystem(chatSystem.get());
    npcManager->setSpawnInterval(2.0f, 4.0f);
    npcManager->setMaxActiveNPCs(5);
  }

  currentState = GameState::RUNNING;
  gameStarted = true;
}

void FruitShopGame::update(float deltaTime) {
  if (currentState != GameState::RUNNING) {
    return;
  }

  gameTimer += deltaTime;

  if (npcManager) {
    npcManager->update(deltaTime);
  }

  if (chatSystem) {
    chatSystem->update(deltaTime);
  }

  if (shop) {
    shop->update(deltaTime);
  }

  updateGameLogic(deltaTime);
  checkGameEndConditions();
}

void FruitShopGame::render(Camera3D camera) {
  if (currentState != GameState::RUNNING) {
    return;
  }

  if (chatSystem) {
    chatSystem->drawAllMessages(camera);
  }

  if (GameWorld* world = GameWorld::getInstance()) {
    if (auto navMesh = world->getNavMesh()) {
      navMesh->debugDraw();
      if (shop) {
        navMesh->debugDrawEntranceNodes(shop->getEntrancePosition(),
                                        shop->getEntranceSize());
      }
    }
  }
}

void FruitShopGame::shutdown() {
  if (npcManager) {
    NPCManager::destroyInstance();
    npcManager = nullptr;
  }

  shop.reset();
  currentState = GameState::GAME_OVER;
}

void FruitShopGame::startGame() {
  if (currentState == GameState::STARTING) {
    initialize();
  }
}

void FruitShopGame::endGame() { currentState = GameState::ENDING; }

void FruitShopGame::resetGame() {
  shutdown();

  // Reset statistics
  fruitsPickedByNPCs = 0;
  npcsSoFar = 0;
  gameTimer = 0.0f;
  gameStarted = false;

  // Reinitialize
  currentState = GameState::STARTING;
  initialize();
}

bool FruitShopGame::isGameOver() const {
  return currentState == GameState::GAME_OVER ||
         currentState == GameState::ENDING || getRemainingFruits() <= 0;
}

int FruitShopGame::getRemainingFruits() const {
  if (!shop) {
    return 0;
  }

  return shop->getTotalFruitCount();
}

int FruitShopGame::getNPCCount() const {
  if (!npcManager) {
    return 0;
  }

  return npcManager->getActiveNPCCount();
}

void FruitShopGame::onFruitPicked() { fruitsPickedByNPCs++; }

void FruitShopGame::onNPCSpawned() { npcsSoFar++; }

void FruitShopGame::setShop(std::shared_ptr<Shop> newShop) {
  shop = newShop;

  if (npcManager && shop) {
    npcManager->setShop(shop);
  }
}

void FruitShopGame::drawGameInfo() const {
  // Draw game status in top-left corner
  DrawText("🍎 FRUIT SHOP GAME", 10, 10, 24, DARKGREEN);

  const char* stateText = "Unknown";
  Color stateColor = WHITE;

  switch (currentState) {
    case GameState::STARTING:
      stateText = "Starting...";
      stateColor = YELLOW;
      break;
    case GameState::RUNNING:
      stateText = "Running";
      stateColor = GREEN;
      break;
    case GameState::ENDING:
      stateText = "Ending";
      stateColor = ORANGE;
      break;
    case GameState::GAME_OVER:
      stateText = "Game Over";
      stateColor = RED;
      break;
  }

  DrawText(TextFormat("Status: %s", stateText), 10, 40, 16, stateColor);
  DrawText(TextFormat("Time: %.1fs", gameTimer), 10, 60, 16, WHITE);
  DrawText(TextFormat("Remaining Fruits: %d", getRemainingFruits()), 10, 80, 16,
           WHITE);
  DrawText(TextFormat("Active NPCs: %d", getNPCCount()), 10, 100, 16, WHITE);

  // Game over message
  if (isGameOver() && getRemainingFruits() <= 0) {
    DrawText("🎉 ALL FRUITS SOLD! 🎉", 10, 130, 20, GOLD);
    DrawText("Press R to restart", 10, 155, 16, YELLOW);
  }
}

void FruitShopGame::drawGameStats() const {
  // Draw detailed statistics in bottom-right corner
  int screenWidth = GetScreenWidth();
  int screenHeight = GetScreenHeight();

  int statsX = screenWidth - 250;
  int statsY = screenHeight - 150;

  DrawText("📊 STATISTICS", statsX, statsY, 16, DARKBLUE);
  DrawText(TextFormat("Fruits Picked: %d", fruitsPickedByNPCs), statsX,
           statsY + 25, 14, WHITE);
  DrawText(TextFormat("NPCs Spawned: %d", npcsSoFar), statsX, statsY + 45, 14,
           WHITE);

  if (npcManager) {
    DrawText(TextFormat("Total Picked: %d", npcManager->getTotalFruitsPicked()),
             statsX, statsY + 65, 14, WHITE);
    DrawText(TextFormat("NPCs Exited: %d", npcManager->getTotalNPCsExited()),
             statsX, statsY + 85, 14, WHITE);
  }
}

void FruitShopGame::updateGameLogic(float deltaTime) {
  // Update game-specific logic here

  // Sync statistics with NPC manager
  if (npcManager) {
    fruitsPickedByNPCs = npcManager->getTotalFruitsPicked();
    npcsSoFar = npcManager->getTotalNPCsSpawned();
  }
}

void FruitShopGame::checkGameEndConditions() {
  // Check if all fruits have been picked
  if (getRemainingFruits() <= 0 && currentState == GameState::RUNNING) {
    endGame();
  }
}

void FruitShopGame::initializeShop() {
  // Create shop at the center of the world
  Vector3 shopPosition = {0.0f, 2.0f, -10.0f};
  shop = std::make_shared<Shop>(shopPosition);

  // Add shop to game world
  if (GameWorld* world = GameWorld::getInstance(nullptr)) {
    world->addObject(shop);
  }

  // Count initial fruits
  totalFruitsInStock = shop->getTotalFruitCount();
}

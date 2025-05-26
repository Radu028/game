#pragma once

#include <memory>

#include "ai/NPCChatSystem.h"
#include "ai/NPCManager.h"
#include "raylib.h"
#include "shop/Shop.h"

// Enum for different game states
enum class GameState { STARTING, RUNNING, ENDING, GAME_OVER };

// Game management class using Singleton pattern
class FruitShopGame {
 private:
  static FruitShopGame* instance;

  GameState currentState;
  std::shared_ptr<Shop> shop;
  NPCManager* npcManager;
  std::unique_ptr<NPCChatSystem> chatSystem;

  // Game parameters
  int initialStockPerFruit;
  int totalFruitsInStock;
  bool gameStarted;
  float gameTimer;

  // Statistics
  int fruitsPickedByNPCs;
  int npcsSoFar;

  FruitShopGame();

 public:
  ~FruitShopGame();

  // Singleton access
  static FruitShopGame* getInstance();
  static void destroyInstance();

  // Game lifecycle
  void initialize();
  void update(float deltaTime);
  void render(Camera3D camera);
  void shutdown();

  // Game state management
  void startGame();
  void endGame();
  void resetGame();
  GameState getCurrentState() const { return currentState; }

  // Shop access
  std::shared_ptr<Shop> getShop() const { return shop; }
  void setShop(std::shared_ptr<Shop> newShop);

  // Game status
  bool isGameRunning() const { return currentState == GameState::RUNNING; }
  bool isGameOver() const;
  int getRemainingFruits() const;

  // Statistics
  int getFruitsPickedByNPCs() const { return fruitsPickedByNPCs; }
  int getNPCCount() const;
  float getGameTime() const { return gameTimer; }

  // Game events
  void onFruitPicked();
  void onNPCSpawned();

  // Chat system access
  NPCChatSystem* getChatSystem() const { return chatSystem.get(); }

  // UI/Debug info
  void drawGameInfo() const;
  void drawGameStats() const;

 private:
  void updateGameLogic(float deltaTime);
  void checkGameEndConditions();
  void initializeShop();
};

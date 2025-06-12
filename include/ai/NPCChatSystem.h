#pragma once

#include <random>
#include <string>
#include <vector>

#include "raylib.h"
#include "objects/GameObject.h"

// NPC Chat System for Romanian comments without diacritics
class NPCChatSystem {
 private:
  struct ChatMessage {
    std::string text;
    float displayTime;
    float maxDisplayTime;
    Vector3 worldPosition;
    const GameObject* follow;
    bool isActive;

    ChatMessage()
        : text(""),
          displayTime(0.0f),
          maxDisplayTime(3.0f),
          worldPosition({0, 0, 0}),
          follow(nullptr),
          isActive(false) {}
  };

  std::vector<std::string> greetingMessages;
  std::vector<std::string> shoppingMessages;
  std::vector<std::string> fruitComments;
  std::vector<std::string> leavingMessages;
  std::vector<std::string> generalComments;

  std::vector<ChatMessage> activeMessages;
  std::mt19937 rng;

  float chatCooldown;
  const float maxChatCooldown = 5.0f;

 public:
  NPCChatSystem();
  ~NPCChatSystem() = default;

  void addMessage(const std::string& message, Vector3 position,
                  float duration = 3.0f,
                  const GameObject* follow = nullptr);
  void update(float deltaTime);
  void drawAllMessages(Camera3D camera) const;

  std::string getRandomGreeting();
  std::string getRandomShoppingComment();
  std::string getRandomFruitComment();
  std::string getRandomLeavingComment();
  std::string getRandomGeneralComment();

  bool canSpeak() const { return chatCooldown <= 0.0f; }
  void resetChatCooldown() { chatCooldown = maxChatCooldown; }

 private:
  void initializeMessages();
  Vector2 worldToScreen(Vector3 worldPos, Camera3D camera) const;
  void drawChatBubble(const std::string& text, Vector2 screenPos) const;
};

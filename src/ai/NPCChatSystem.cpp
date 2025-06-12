#include "ai/NPCChatSystem.h"

#include <algorithm>
#include <chrono>

NPCChatSystem::NPCChatSystem()
    : rng(std::chrono::steady_clock::now().time_since_epoch().count()),
      chatCooldown(0.0f) {
  initializeMessages();
}

void NPCChatSystem::initializeMessages() {
  // Romanian greetings without diacritics
  greetingMessages = {"Buna ziua!",
                      "Salut!",
                      "Ce mai faceti?",
                      "Buna dimineata!",
                      "Noroc!",
                      "Hai sa cumpar ceva...",
                      "Oh, un magazin frumos!"};

  // Shopping comments
  shoppingMessages = {"Hmm, ce sa aleg?",
                      "Merele arata bine...",
                      "Ce preturi sunt aici?",
                      "Sa vad ce fructe aveti...",
                      "Banane fresh, perfect!",
                      "Cam scump dar...",
                      "Asta pare bun",
                      "Ce sa cumpar oare?",
                      "Am nevoie de ceva sanatos",
                      "Pentru copii acasa...",
                      "Poate ceva pentru deserturi",
                      "Sa verific calitatea..."};

  // Fruit-specific comments
  fruitComments = {"Mere frumoase!",
                   "Banane perfecte!",
                   "Portocale dulci!",
                   "Capsuni aromate!",
                   "Perfect pentru mic dejun!",
                   "Asta e exact ce cautam!",
                   "Calitate buna!",
                   "Fresh si sanatos!",
                   "Pentru smoothie...",
                   "Copiii vor adora!",
                   "Mmmm, arata delicios!",
                   "Exact ce imi trebuie!"};

  // Leaving comments
  leavingMessages = {"Multumesc!",
                     "Buna ziua!",
                     "Pe curand!",
                     "Am terminat cumparaturile",
                     "Ma duc acasa",
                     "La revedere!",
                     "O zi buna!",
                     "Sunt multumit!",
                     "Perfect, am gasit tot!",
                     "Mergem acasa..."};

  // General comments
  generalComments = {"Uite ce magazin fain!",   "Sunt multe de ales...",
                     "Se pare ca e plin aici",  "Magazin cu produse bune",
                     "Mi-e foame...",           "Sa ma grabesc putin",
                     "Trebuie sa ajung acasa",  "Copiii m-au trimis...",
                     "Pentru cina de azi",      "Cumparaturi rapide",
                     "Sper sa nu fie coada...", "Preturi rezonabile"};
}

void NPCChatSystem::addMessage(const std::string& message, Vector3 position,
                               float duration, const GameObject* follow) {
  ChatMessage newMessage;
  newMessage.text = message;
  newMessage.worldPosition = position;
  newMessage.follow = follow;
  newMessage.maxDisplayTime = duration;
  newMessage.displayTime = 0.0f;
  newMessage.isActive = true;

  activeMessages.push_back(newMessage);
  resetChatCooldown();
}

void NPCChatSystem::update(float deltaTime) {
  if (chatCooldown > 0.0f) {
    chatCooldown -= deltaTime;
  }

  for (auto& message : activeMessages) {
    if (message.isActive) {
      if (message.follow) {
        message.worldPosition = message.follow->getPosition();
      }
      message.displayTime += deltaTime;
      if (message.displayTime >= message.maxDisplayTime) {
        message.isActive = false;
      }
    }
  }

  activeMessages.erase(
      std::remove_if(activeMessages.begin(), activeMessages.end(),
                     [](const ChatMessage& msg) { return !msg.isActive; }),
      activeMessages.end());
}

void NPCChatSystem::drawAllMessages(Camera3D camera) const {
  for (const auto& message : activeMessages) {
    if (message.isActive) {
      Vector2 screenPos = worldToScreen(message.worldPosition, camera);

      // Only draw if position is on screen
      if (screenPos.x >= -100 && screenPos.x <= GetScreenWidth() + 100 &&
          screenPos.y >= -100 && screenPos.y <= GetScreenHeight() + 100) {
        drawChatBubble(message.text, screenPos);
      }
    }
  }
}

Vector2 NPCChatSystem::worldToScreen(Vector3 worldPos, Camera3D camera) const {
  // Add some height offset for the chat bubble
  Vector3 chatPos = {worldPos.x, worldPos.y + 2.5f, worldPos.z};
  return GetWorldToScreen(chatPos, camera);
}

void NPCChatSystem::drawChatBubble(const std::string& text,
                                   Vector2 screenPos) const {
  if (text.empty()) return;

  int fontSize = 16;
  Vector2 textSize = MeasureTextEx(GetFontDefault(), text.c_str(), fontSize, 1);

  float padding = 8.0f;
  float bubbleWidth = textSize.x + padding * 2;
  float bubbleHeight = textSize.y + padding * 2;

  Vector2 bubblePos = {screenPos.x - bubbleWidth / 2,
                       screenPos.y - bubbleHeight - 10};

  // Draw bubble background with rounded corners effect
  DrawRectangleRounded({bubblePos.x, bubblePos.y, bubbleWidth, bubbleHeight},
                       0.3f, 8, {255, 255, 255, 220});

  // Draw bubble border
  DrawRectangleRoundedLines(
      {bubblePos.x, bubblePos.y, bubbleWidth, bubbleHeight}, 0.3f, 8,
      {100, 100, 100, 180});

  // Draw text
  Vector2 textPos = {bubblePos.x + padding, bubblePos.y + padding};
  DrawTextEx(GetFontDefault(), text.c_str(), textPos, fontSize, 1,
             {50, 50, 50, 255});

  // Draw small arrow pointing to NPC
  Vector2 arrowTip = {screenPos.x, screenPos.y - 5};
  Vector2 arrowLeft = {screenPos.x - 6, bubblePos.y + bubbleHeight};
  Vector2 arrowRight = {screenPos.x + 6, bubblePos.y + bubbleHeight};

  DrawTriangle(arrowTip, arrowLeft, arrowRight, {255, 255, 255, 220});
  DrawTriangleLines(arrowTip, arrowLeft, arrowRight, {100, 100, 100, 180});
}

std::string NPCChatSystem::getRandomGreeting() {
  if (greetingMessages.empty()) return "";
  std::uniform_int_distribution<size_t> dist(0, greetingMessages.size() - 1);
  return greetingMessages[dist(rng)];
}

std::string NPCChatSystem::getRandomShoppingComment() {
  if (shoppingMessages.empty()) return "";
  std::uniform_int_distribution<size_t> dist(0, shoppingMessages.size() - 1);
  return shoppingMessages[dist(rng)];
}

std::string NPCChatSystem::getRandomFruitComment() {
  if (fruitComments.empty()) return "";
  std::uniform_int_distribution<size_t> dist(0, fruitComments.size() - 1);
  return fruitComments[dist(rng)];
}

std::string NPCChatSystem::getRandomLeavingComment() {
  if (leavingMessages.empty()) return "";
  std::uniform_int_distribution<size_t> dist(0, leavingMessages.size() - 1);
  return leavingMessages[dist(rng)];
}

std::string NPCChatSystem::getRandomGeneralComment() {
  if (generalComments.empty()) return "";
  std::uniform_int_distribution<size_t> dist(0, generalComments.size() - 1);
  return generalComments[dist(rng)];
}

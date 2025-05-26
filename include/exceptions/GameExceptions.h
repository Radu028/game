#pragma once

#include <exception>
#include <string>

// Base exception class derived from std::exception
class GameException : public std::exception {
 private:
  std::string message;

 public:
  explicit GameException(const std::string& msg) : message(msg) {}
  virtual ~GameException() = default;

  const char* what() const noexcept override { return message.c_str(); }

  virtual const std::string& getType() const {
    static const std::string type = "GameException";
    return type;
  }
};

// Initialization specific exceptions
class GameInitException : public GameException {
 public:
  explicit GameInitException(const std::string& msg)
      : GameException("Game Initialization Error: " + msg) {}

  const std::string& getType() const override {
    static const std::string type = "GameInitException";
    return type;
  }
};

// Resource loading exceptions
class ResourceException : public GameException {
 private:
  std::string resourcePath;

 public:
  ResourceException(const std::string& msg, const std::string& path)
      : GameException("Resource Error: " + msg + " (Path: " + path + ")"),
        resourcePath(path) {}

  const std::string& getResourcePath() const { return resourcePath; }

  const std::string& getType() const override {
    static const std::string type = "ResourceException";
    return type;
  }
};

// Physics and collision exceptions
class PhysicsException : public GameException {
 private:
  std::string objectName;

 public:
  PhysicsException(const std::string& msg, const std::string& objName = "")
      : GameException("Physics Error: " + msg +
                      (objName.empty() ? "" : " (Object: " + objName + ")")),
        objectName(objName) {}

  const std::string& getObjectName() const { return objectName; }

  const std::string& getType() const override {
    static const std::string type = "PhysicsException";
    return type;
  }
};

// AI and NPC specific exceptions
class AIException : public GameException {
 private:
  std::string npcId;
  std::string currentState;

 public:
  AIException(const std::string& msg, const std::string& id = "",
              const std::string& state = "")
      : GameException("AI Error: " + msg +
                      (id.empty() ? "" : " (NPC: " + id + ")") +
                      (state.empty() ? "" : " (State: " + state + ")")),
        npcId(id),
        currentState(state) {}

  const std::string& getNPCId() const { return npcId; }
  const std::string& getCurrentState() const { return currentState; }

  const std::string& getType() const override {
    static const std::string type = "AIException";
    return type;
  }
};

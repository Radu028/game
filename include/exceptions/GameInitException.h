#ifndef GAME_INIT_EXCEPTION_H
#define GAME_INIT_EXCEPTION_H

#include "GameException.h"

class GameInitException : public GameException {
 public:
  GameInitException(const std::string& msg)
      : GameException("Initialization error: " + msg) {}
};

#endif  // GAME_INIT_EXCEPTION_H
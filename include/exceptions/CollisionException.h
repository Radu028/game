#ifndef COLLISION_EXCEPTION_H
#define COLLISION_EXCEPTION_H

#include "GameException.h"

class CollisionException : public GameException {
 public:
  CollisionException(const std::string& msg)
      : GameException("Collision error: " + msg) {}
};

#endif  // COLLISION_EXCEPTION_H
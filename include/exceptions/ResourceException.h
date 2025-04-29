#ifndef RESOURCE_EXCEPTION_H
#define RESOURCE_EXCEPTION_H

#include "GameException.h"

class ResourceException : public GameException {
 public:
  ResourceException(const std::string& msg)
      : GameException("Resource error: " + msg) {}
};

#endif  // RESOURCE_EXCEPTION_H
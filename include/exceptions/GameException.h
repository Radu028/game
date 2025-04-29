#ifndef GAME_EXCEPTION_H
#define GAME_EXCEPTION_H

#include <exception>
#include <string>

class GameException : public std::exception {
 private:
  std::string message;

 public:
  GameException(const std::string& msg) : message(msg) {}

  const char* what() const noexcept override { return message.c_str(); }
};

#endif  // GAME_EXCEPTION_H
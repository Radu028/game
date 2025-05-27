#include "utils/GameObjectManager.h"

#include <iostream>

#include "objects/GameObject.h"

template class GameObjectManager<GameObject>;

void demonstrateGameObjectManager() {
  GameObjectManager<GameObject> manager("Test Manager");

  std::cout << "GameObjectManager demonstration:" << std::endl;
  std::cout << "Total objects managed: " << manager.size() << std::endl;
  std::cout << "Manager instance count: "
            << GameObjectManager<GameObject>::getTotalManagersCreated()
            << std::endl;
}

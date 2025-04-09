#include "GameWorld.h"

GameWorld::GameWorld(Player* player) : player(player) {}

void GameWorld::addObject(std::shared_ptr<GameObject> object) { objects.push_back(object); }

void GameWorld::update(float deltaTime) {
    player->update(deltaTime);

    for (auto& obj : objects) {
        obj->update(deltaTime);
    }

    checkCollisions();
}

void GameWorld::draw() const {
    for (const auto& obj : objects) {
        obj->draw();
    }
}

void GameWorld::checkCollisions() {
    Vector3 prevPosition = player->getPosition();

    for (const auto& obj : objects) {
        if (player->checkCollision(*obj)) {
            player->setPosition(prevPosition);
            break;
        }
    }
}
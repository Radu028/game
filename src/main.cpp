#include <cmath>
#include <iostream>

#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "exceptions/GameExceptions.h"
#include "game/FruitShopGame.h"
#include "objects/Floor.h"
#include "raylib.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"
#include "systems/ShaderSystem.h"

int main() {
  try {
    InitWindow(1280, 720, "3D Game");
    SetTargetFPS(120);

    EnableCursor();

    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    if (!shaderSystem->initialize()) {
      throw GameInitException("Failed to initialize shader system");
    }

    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 25.0f, 5.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float cameraDistance = 25.0f;
    float cameraAngleX = 0.0f;
    float cameraAngleY = -30.0f;
    const float mouseSensitivity = 0.5f;
    const float minCameraDistance = 5.0f;
    const float maxCameraDistance = 30.0f;
    const float maxVerticalAngle = 89.0f;
    const float minVerticalAngle = -89.0f;

    bool debugMode = false;

    auto player1 =
        std::make_shared<HumanoidCharacter>((Vector3){0.0f, 1.0f, 15.0f});

    GameWorld* world = GameWorld::getInstance(player1.get());
    player1->setWorld(world);
    player1->setupPhysics(world->getDynamicsWorld());

    world->addObject(std::make_shared<Floor>((Vector3){0.0f, 0.0f, 0.0f},
                                             (Vector3){50.0f, 1.0f, 50.0f},
                                             DARKGREEN, true, true));

    FruitShopGame* fruitShopGame = FruitShopGame::getInstance();
    fruitShopGame->startGame();

    bool dynamicSun = false;

    while (!WindowShouldClose()) {
      float deltaTime = GetFrameTime();

      if (IsKeyPressed(KEY_F1)) {
        debugMode = !debugMode;
      }

      if (IsKeyPressed(KEY_F2)) {
        dynamicSun = !dynamicSun;
      }

      if (!dynamicSun) {
        if (IsKeyDown(KEY_LEFT)) {
          Vector3 sunDir = shaderSystem->getSunDirection();
          sunDir.x += deltaTime;
          shaderSystem->setSunDirection(sunDir);
        }
        if (IsKeyDown(KEY_RIGHT)) {
          Vector3 sunDir = shaderSystem->getSunDirection();
          sunDir.x -= deltaTime;
          shaderSystem->setSunDirection(sunDir);
        }
        if (IsKeyDown(KEY_UP)) {
          Vector3 sunDir = shaderSystem->getSunDirection();
          sunDir.y += deltaTime;
          shaderSystem->setSunDirection(sunDir);
        }
        if (IsKeyDown(KEY_DOWN)) {
          Vector3 sunDir = shaderSystem->getSunDirection();
          sunDir.y -= deltaTime;
          shaderSystem->setSunDirection(sunDir);
        }
      }

      if (IsKeyPressed(KEY_R)) {
        fruitShopGame->resetGame();
      }

      InputSystem::updateMouseCamera();

      if (InputSystem::isRightMouseDown()) {
        Vector2 mouseDelta = InputSystem::getMouseDelta();

        cameraAngleX -= mouseDelta.x * mouseSensitivity;
        cameraAngleY += mouseDelta.y * mouseSensitivity;

        if (cameraAngleY > maxVerticalAngle) cameraAngleY = maxVerticalAngle;
        if (cameraAngleY < minVerticalAngle) cameraAngleY = minVerticalAngle;
      }

      float wheelMove = GetMouseWheelMove();
      if (wheelMove != 0) {
        cameraDistance -= wheelMove * 2.0f;
        if (cameraDistance < minCameraDistance)
          cameraDistance = minCameraDistance;
        if (cameraDistance > maxCameraDistance)
          cameraDistance = maxCameraDistance;
      }

      shaderSystem->updateTime(deltaTime);

      if (dynamicSun) {
        shaderSystem->enableDynamicSun(true);
      }

      player1->handleInput(GameSettings::Character::MOVEMENT_SPEED,
                           cameraAngleX);
      player1->update(deltaTime);

      fruitShopGame->update(deltaTime);
      world->update(deltaTime);

      Vector3 playerPos = player1->getFeetPosition();

      float angleXRad = cameraAngleX * DEG2RAD;
      float angleYRad = cameraAngleY * DEG2RAD;

      camera.target = (Vector3){
          playerPos.x, playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET,
          playerPos.z};

      camera.position = (Vector3){
          playerPos.x + cameraDistance * cosf(angleYRad) * sinf(angleXRad),
          playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET +
              cameraDistance * sinf(angleYRad),
          playerPos.z + cameraDistance * cosf(angleYRad) * cosf(angleXRad)};

      shaderSystem->updateUniforms(camera);

      BeginDrawing();
      ClearBackground(RAYWHITE);

      BeginMode3D(camera);
      player1->draw();
      world->draw();
      fruitShopGame->render(camera);

      if (debugMode) {
        player1->drawCollisionBoxes();
      }
      EndMode3D();

      EndDrawing();
    }

    CloseWindow();
    return 0;

  } catch (const GameInitException& e) {
    std::cerr << "Game Initialization Error: " << e.what() << std::endl;
    CloseWindow();
    return -1;
  } catch (const ResourceException& e) {
    std::cerr << "Resource Error: " << e.what() << std::endl;
    CloseWindow();
    return -2;
  } catch (const PhysicsException& e) {
    std::cerr << "Physics Error: " << e.what() << std::endl;
    CloseWindow();
    return -3;
  } catch (const AIException& e) {
    std::cerr << "AI Error: " << e.what() << std::endl;
    CloseWindow();
    return -4;
  } catch (const GameException& e) {
    std::cerr << "Game Error: " << e.what() << std::endl;
    CloseWindow();
    return -5;
  } catch (const std::exception& e) {
    std::cerr << "Standard Library Error: " << e.what() << std::endl;
    CloseWindow();
    return -6;
  } catch (...) {
    std::cerr << "Unknown error occurred!" << std::endl;
    CloseWindow();
    return -7;
  }
}

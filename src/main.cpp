#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "systems/InputSystem.h"
#include "raylib.h"
#include "settings/Physics.h"

int main() {
  InitWindow(1280, 720, "Joc 3D");
  SetTargetFPS(120);

  DisableCursor();

  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 5.0f, 10.0f};
  camera.target = (Vector3){0.0f, 1.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  auto player1 = std::make_shared<HumanoidCharacter>((Vector3){0.0f, 0.0f, 0.0f});
  
  GameWorld* world = GameWorld::getInstance(player1.get());
  player1->setWorld(world);

  player1->setupPhysics(world->getDynamicsWorld());

  world->addObject(std::make_shared<CubeObject>((Vector3){2.0f, 0.5f, 0.0f},
                                                (Vector3){1.0f, 2.0f, 1.0f},
                                                PINK, true, "", true, false));

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){-2.0f, 0.5f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, SKYBLUE, true,
      "", true, false));

  world->addObject(std::make_shared<CubeObject>((Vector3){0.0f, 0.5f, 2.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                ORANGE, true, "", true, false));

  world->addObject(std::make_shared<Floor>(
      (Vector3){0.0f, 0.0f, 0.0f}, (Vector3){10.0f, 1.0f, 10.0f},
      "../resources/forrest_ground_01_diff_4k.jpg", true));

  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime();

    player1->handleInput(GameSettings::Character::MOVEMENT_SPEED);
    player1->update(deltaTime);

    world->update(deltaTime);

    Vector3 playerPos = player1->getFeetPosition();
    
    camera.target = (Vector3){playerPos.x, playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET, playerPos.z};
    camera.position = (Vector3){playerPos.x + GameSettings::Camera::OFFSET.x, playerPos.y + GameSettings::Camera::OFFSET.y, playerPos.z + GameSettings::Camera::OFFSET.z};

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);
    player1->draw();
    world->draw();
    EndMode3D();

    DrawFPS(10, 40);

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
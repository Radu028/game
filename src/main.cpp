#include "GameWorld.h"
#include "entities/Player.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/GameObject.h"
#include "raylib.h"
#include "settings/Physics.h"

#define PLAYER_MOVEMENT_SPEED 0.1f

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

  auto player1 = std::make_shared<Player>((Vector3){0.0f, 5.0f, 0.0f});
  GameWorld* world = GameWorld::getInstance(player1.get());
  player1->setWorld(world);

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

  // Debug: Print floor bounding box after adding
  auto floorObj = world->getObjects().back();
  if (auto floor = std::dynamic_pointer_cast<Floor>(floorObj)) {
    BoundingBox box = floor->getBoundingBox();
    printf("[DEBUG] Floor pos: (%.2f, %.2f, %.2f), size: (%.2f, %.2f, %.2f)\n",
           floor->getPosition().x, floor->getPosition().y, floor->getPosition().z,
           10.0f, 1.0f, 10.0f);
    printf("[DEBUG] Floor bounding box min: (%.2f, %.2f, %.2f), max: (%.2f, %.2f, %.2f)\n",
           box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z);
  }

  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime();

    // Debug: Print Raylib input state
    printf("[DEBUG] IsKeyDown(KEY_W): %d\n", IsKeyDown(KEY_W));

    player1->handleInput(PLAYER_MOVEMENT_SPEED);
    world->update(deltaTime);

    // if (IsKeyPressed(KEY_E)) {
    //   for (const auto& obj : world->getObjects()) {
    //     if (obj) {
    //       obj->interact();
    //     }
    //   }
    // }

    Vector3 playerPos = player1->getPosition();
    camera.target = playerPos;
    camera.position =
        (Vector3){playerPos.x, playerPos.y + 5.0f, playerPos.z + 10.0f};

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
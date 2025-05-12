
#include "GameWorld.h"
#include "entities/Player.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/GameObject.h"
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

  Player player1((Vector3){0.0f, 2.0f, 0.0f});
  GameWorld* world = GameWorld::getInstance(&player1);
  player1.setWorld(world);

  world->addObject(std::make_shared<CubeObject>((Vector3){2.0f, 0.5f, 0.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                PINK, true, "", true, false));

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){-2.0f, 0.5f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, SKYBLUE, true,
      "", false, true));

  world->addObject(std::make_shared<CubeObject>((Vector3){0.0f, 0.5f, 2.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                ORANGE, true, "", false, true));

  world->addObject(std::make_shared<Floor>(
      (Vector3){0.0f, -5.05f, 0.0f}, (Vector3){50.0f, 0.1f, 50.0f},
      "../resources/forrest_ground_01_diff_4k.jpg", true));
  // "../resources/forrest_ground_01_diff_4k.jpg"

  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime();

    // player1.handleInput(0.1f);

    world->update(GetFrameTime());

    if (IsKeyPressed(KEY_E)) {
      for (const auto& obj : world->getObjects()) {
        if (obj) {
          obj->interact();
        }
      }
    }

    Vector3 playerPos = player1.getPosition();
    camera.target = playerPos;
    camera.position =
        (Vector3){playerPos.x, playerPos.y + 5.0f, playerPos.z + 10.0f};

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);
    player1.draw();
    world->draw();
    EndMode3D();

    DrawFPS(10, 40);

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
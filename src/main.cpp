#include "GameObject.h"
#include "GameWorld.h"
#include "Physics.h"
#include "entities/Player.h"
#include "exceptions/GameException.h"
#include "exceptions/GameInitException.h"
#include "exceptions/ResourceException.h"
#include "objects/MovingCubeObject.h"
#include "objects/RotatingCubeObject.h"
#include "objects/ScalingCubeObject.h"
#include "raylib.h"

int main() {
  try {
    // Initialize the window
    InitWindow(1280, 720, "Joc 3D");
    SetTargetFPS(120);
    DisableCursor();

    // Setup camera
    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 5.0f, 10.0f};
    camera.target = (Vector3){0.0f, 1.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create player
    Player player1((Vector3){0.0f, 2.0f, 0.0f});

    // Create game world
    GameWorld* world = GameWorld::getInstance(&player1);
    player1.setWorld(world);

    // Add regular cube
    world->addObject(std::make_shared<CubeObject>(
        (Vector3){2.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, PINK, true));

    // Add moving cube
    world->addObject(std::make_shared<MovingCubeObject>(
        (Vector3){-2.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, SKYBLUE, true,
        (Vector3){0.0f, 0.0f, 1.0f},    // velocity
        (Vector3){-3.0f, 0.0f, -5.0f},  // min bounds
        (Vector3){-1.0f, 3.0f, 5.0f}    // max bounds
        ));

    // Add rotating cube
    world->addObject(std::make_shared<RotatingCubeObject>(
        (Vector3){0.0f, 0.5f, 2.0f}, 1.0f, 1.0f, 1.0f, ORANGE, true, 30.0f,
        45.0f, 60.0f  // rotation speeds
        ));

    // Add scaling cube
    world->addObject(std::make_shared<ScalingCubeObject>(
        (Vector3){0.0f, 0.5f, -2.0f}, 1.0f, 1.0f, 1.0f, PURPLE, true, 0.5f,
        1.5f, 0.5f  // min scale, max scale, scale speed
        ));

    // Add ground
    try {
      world->addObject(std::make_shared<CubeObject>(
          (Vector3){0.0f, -0.05f, 0.0f}, 50.0f, 0.1f, 50.0f, GREEN, true,
          "../resources/forrest_ground_01_diff_4k.jpg"));
    } catch (const ResourceException& e) {
      TraceLog(LOG_WARNING,
               "Could not load texture: %s. Using plain color instead.",
               e.what());
      world->addObject(std::make_shared<CubeObject>(
          (Vector3){0.0f, -0.05f, 0.0f}, 50.0f, 0.1f, 50.0f, GREEN, true));
    }

    // Game loop
    while (!WindowShouldClose()) {
      // Process input
      player1.handleInput(0.1f);

      if (IsKeyPressed(KEY_E)) {
        // Interact with nearest object when 'E' is pressed
        world->interactWithNearestObject();
      }

      // Update game state
      world->update(GetFrameTime());

      // Update camera position
      Vector3 playerPos = player1.getPosition();
      camera.target = playerPos;
      camera.position =
          (Vector3){playerPos.x, playerPos.y + 5.0f, playerPos.z + 10.0f};

      // Render
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
  } catch (const GameInitException& e) {
    TraceLog(LOG_ERROR, "Game initialization error: %s", e.what());
  } catch (const ResourceException& e) {
    TraceLog(LOG_ERROR, "Resource error: %s", e.what());
  } catch (const GameException& e) {
    TraceLog(LOG_ERROR, "Game error: %s", e.what());
  } catch (const std::exception& e) {
    TraceLog(LOG_ERROR, "Standard exception: %s", e.what());
  } catch (...) {
    TraceLog(LOG_ERROR, "Unknown error occurred");
  }

  CloseWindow();
  return 1;
}
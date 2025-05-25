#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "systems/InputSystem.h"
#include "raylib.h"
#include "settings/Physics.h"

int main() {
  InitWindow(1280, 720, "3D Game");
  SetTargetFPS(120);

  DisableCursor();

  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 5.0f, 10.0f};
  camera.target = (Vector3){0.0f, 1.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  // Professional debug mode toggle
  bool debugMode = false;

  auto player1 = std::make_shared<HumanoidCharacter>((Vector3){0.0f, 0.5f, 0.0f});
  
  GameWorld* world = GameWorld::getInstance(player1.get());
  player1->setWorld(world);

  player1->setupPhysics(world->getDynamicsWorld());

  world->addObject(std::make_shared<CubeObject>((Vector3){2.0f, 1.0f, 0.0f},
                                                (Vector3){1.0f, 2.0f, 1.0f},
                                                PINK, true, "", false, true)); // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){-2.0f, 1.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, SKYBLUE, true,
      "", false, true)); // Static cube

  world->addObject(std::make_shared<CubeObject>((Vector3){0.0f, 1.0f, 2.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                ORANGE, true, "", false, true)); // Static cube

  // Add a tall platform for jump testing
  world->addObject(std::make_shared<CubeObject>((Vector3){4.0f, 1.5f, 0.0f},
                                                (Vector3){1.5f, 3.0f, 1.5f},
                                                GREEN, true, "", false, true)); // Static cube

  // Add more cubes for climbing and jumping practice
  world->addObject(std::make_shared<CubeObject>((Vector3){-4.0f, 0.5f, 2.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                RED, true, "", false, true)); // Static cube

  world->addObject(std::make_shared<CubeObject>((Vector3){6.0f, 1.0f, -2.0f},
                                                (Vector3){2.0f, 2.0f, 1.0f},
                                                PURPLE, true, "", false, true)); // Static cube

  // Create a stepping stone pattern for jump testing
  world->addObject(std::make_shared<CubeObject>((Vector3){1.0f, 0.5f, -4.0f},
                                                (Vector3){1.0f, 1.0f, 1.0f},
                                                YELLOW, true, "", false, true)); // Static cube

  world->addObject(std::make_shared<CubeObject>((Vector3){2.5f, 1.0f, -4.0f},
                                                (Vector3){1.0f, 2.0f, 1.0f},
                                                MAROON, true, "", false, true)); // Static cube

  world->addObject(std::make_shared<CubeObject>((Vector3){4.0f, 1.5f, -4.0f},
                                                (Vector3){1.0f, 3.0f, 1.0f},
                                                LIME, true, "", false, true)); // Static cube

  world->addObject(std::make_shared<Floor>(
      (Vector3){0.0f, 0.0f, 0.0f}, (Vector3){10.0f, 1.0f, 10.0f},
      "../resources/forrest_ground_01_diff_4k.jpg", true));

  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime();

    // Toggle debug mode with F1 key
    if (IsKeyPressed(KEY_F1)) {
      debugMode = !debugMode;
    }

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
    
    // Professional multi-body collision visualization
    if (debugMode) {
      player1->drawCollisionBoxes();
    }
    EndMode3D();

    DrawFPS(10, 40);
    
    // Debug info display
    if (debugMode) {
      DrawText("DEBUG MODE: F1 to toggle", 10, 70, 20, GREEN);
      DrawText("Multi-body collision system active", 10, 95, 20, GREEN);
      DrawText("Purple=Head, Blue=Torso, Green=Arms, Orange=Legs", 10, 120, 16, WHITE);
    }

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
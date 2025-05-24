#include "GameWorld.h"
#include "entities/Player.h"
#include "entities/AdvancedPlayerController.h"
#include "entities/SimpleRagdoll.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/GameObject.h"
#include "systems/InputSystem.h"
#include "raylib.h"
#include "settings/Physics.h"

#define PLAYER_MOVEMENT_SPEED 5.0f

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

  auto player1 = std::make_shared<SimpleRagdoll>((Vector3){0.0f, 0.0f, 0.0f}); // Ground level
  
  GameWorld* world = GameWorld::getInstance(player1.get());
  player1->setWorld(world);

  // Setup physics for the simple ragdoll
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

    // Use SimpleRagdoll for input and physics
    player1->handleInput(PLAYER_MOVEMENT_SPEED);
    player1->update(deltaTime);

    world->update(deltaTime);

    // if (IsKeyPressed(KEY_E)) {
    //   for (const auto& obj : world->getObjects()) {
    //     if (obj) {
    //       obj->interact();
    //     }
    //   }
    // }

    Vector3 playerPos = player1->getFeetPosition(); // Get feet position for proper camera tracking
    // Improved camera positioning - make sure character is always visible
    Vector3 cameraOffset = {0.0f, 3.0f, 8.0f};  // Closer and lower for better view
    camera.target = (Vector3){playerPos.x, playerPos.y + 1.5f, playerPos.z}; // Target at character center
    camera.position = (Vector3){playerPos.x + cameraOffset.x, playerPos.y + cameraOffset.y, playerPos.z + cameraOffset.z};
    
    // Debug: Print camera and player positions
    static int cameraDebugCounter = 0;
    if (++cameraDebugCounter % 120 == 0) { // Every 2 seconds
        printf("[DEBUG] Camera pos: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)\n",
               camera.position.x, camera.position.y, camera.position.z,
               camera.target.x, camera.target.y, camera.target.z);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);
    player1->draw();
    world->draw();
    EndMode3D();

    // Draw debug info and controls
    DrawFPS(10, 40);
    DrawText(TextFormat("Player Feet Pos: (%.2f, %.2f, %.2f)", playerPos.x, playerPos.y, playerPos.z), 10, 70, 20, WHITE);
    
    // Enhanced ground detection display
    bool raycastGround = player1->isOnGround();
    bool contactGround = player1->isOnGroundContact();
    bool canJump = raycastGround || contactGround;
    
    DrawText(TextFormat("Ground (Raycast): %s", raycastGround ? "Yes" : "No"), 10, 100, 20, raycastGround ? GREEN : RED);
    DrawText(TextFormat("Ground (Contact): %s", contactGround ? "Yes" : "No"), 10, 120, 20, contactGround ? GREEN : RED);
    DrawText(TextFormat("Can Jump: %s", canJump ? "YES" : "NO"), 10, 140, 20, canJump ? GREEN : RED);
    
    DrawText("Simple Ragdoll Character", 10, 170, 20, WHITE);
    
    // Control instructions
    DrawText("CONTROLS:", 10, 200, 20, YELLOW);
    DrawText("WASD - Move", 10, 220, 16, WHITE);
    DrawText("SPACE - Jump", 10, 240, 16, WHITE);
    DrawText("Mouse - Look around", 10, 260, 16, WHITE);
    DrawText("ESC - Close game", 10, 280, 16, WHITE);
    
    // Movement status indicators
    Vector2 moveAxis = InputSystem::getMovementAxis();
    if (moveAxis.x != 0.0f || moveAxis.y != 0.0f) {
        DrawText(TextFormat("MOVING: (%.1f, %.1f)", moveAxis.x, moveAxis.y), 10, 310, 18, GREEN);
    }
    if (InputSystem::isJumpPressed()) {
        DrawText("JUMPING!", 10, 330, 18, RED);
    }

    EndDrawing();
  }

  // Cleanup is handled by smart pointers and destructors
  CloseWindow();
  return 0;
}
#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/Sphere.h"
#include "raylib.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"
#include "systems/ShaderSystem.h"

int main() {
  InitWindow(1280, 720, "3D Game");
  SetTargetFPS(120);

  DisableCursor();

  // Initialize shader system
  ShaderSystem* shaderSystem = ShaderSystem::getInstance();
  if (!shaderSystem->initialize()) {
    TraceLog(LOG_WARNING,
             "Failed to initialize shader system, using default rendering");
  } else {
    TraceLog(LOG_INFO, "Shader system initialized successfully");
  }

  Camera3D camera = {0};
  camera.position = (Vector3){0.0f, 5.0f, 10.0f};
  camera.target = (Vector3){0.0f, 1.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  // Professional debug mode toggle
  bool debugMode = false;

  auto player1 =
      std::make_shared<HumanoidCharacter>((Vector3){0.0f, 0.5f, 0.0f});

  GameWorld* world = GameWorld::getInstance(player1.get());
  player1->setWorld(world);

  player1->setupPhysics(world->getDynamicsWorld());

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){2.0f, 1.0f, 0.0f}, (Vector3){1.0f, 2.0f, 1.0f}, PINK, true, "",
      false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){-2.0f, 1.0f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, SKYBLUE, true,
      "", false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){0.0f, 1.0f, 2.0f}, (Vector3){1.0f, 1.0f, 1.0f}, ORANGE, true,
      "", false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){4.0f, 1.5f, 0.0f}, (Vector3){1.5f, 3.0f, 1.5f}, GREEN, true, "",
      false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){-4.0f, 0.5f, 2.0f}, (Vector3){1.0f, 1.0f, 1.0f}, RED, true, "",
      false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){6.0f, 1.0f, -2.0f}, (Vector3){2.0f, 2.0f, 1.0f}, PURPLE, true,
      "", false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){1.0f, 0.5f, -4.0f}, (Vector3){1.0f, 1.0f, 1.0f}, YELLOW, true,
      "", false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){2.5f, 1.0f, -4.0f}, (Vector3){1.0f, 2.0f, 1.0f}, MAROON, true,
      "", false, true));  // Static cube

  world->addObject(std::make_shared<CubeObject>(
      (Vector3){4.0f, 1.5f, -4.0f}, (Vector3){1.0f, 3.0f, 1.0f}, LIME, true, "",
      false, true));  // Static cube

  world->addObject(std::make_shared<Floor>((Vector3){0.0f, 0.0f, 0.0f},
                                           (Vector3){10.0f, 1.0f, 10.0f},
                                           DARKGREEN, true, true));
  // "../resources/forrest_ground_01_diff_4k.jpg"

  world->addObject(std::make_shared<Sphere>((Vector3){-1.0f, 2.0f, 1.0f}, 2.0f,
                                            BLUE, true, true));

  // Dynamic sun state
  bool dynamicSun = false;

  while (!WindowShouldClose()) {
    float deltaTime = GetFrameTime();

    // Toggle debug mode with F1 key
    if (IsKeyPressed(KEY_F1)) {
      debugMode = !debugMode;
    }

    // Toggle dynamic sun with F2 key
    if (IsKeyPressed(KEY_F2)) {
      dynamicSun = !dynamicSun;
    }

    // Control sun direction with arrow keys for manual testing (only when
    // dynamic sun is off)
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

    // Update shader system time
    shaderSystem->updateTime(deltaTime);

    // Apply dynamic sun if enabled
    if (dynamicSun) {
      shaderSystem->enableDynamicSun(true);
    }

    player1->handleInput(GameSettings::Character::MOVEMENT_SPEED);
    player1->update(deltaTime);

    world->update(deltaTime);

    Vector3 playerPos = player1->getFeetPosition();

    camera.target = (Vector3){
        playerPos.x, playerPos.y + GameSettings::Camera::TARGET_Y_OFFSET,
        playerPos.z};
    camera.position = (Vector3){playerPos.x + GameSettings::Camera::OFFSET.x,
                                playerPos.y + GameSettings::Camera::OFFSET.y,
                                playerPos.z + GameSettings::Camera::OFFSET.z};

    // Update shader uniforms with camera position
    shaderSystem->updateUniforms(camera);

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
      DrawText("Purple=Head, Blue=Torso, Green=Arms, Orange=Legs", 10, 120, 16,
               WHITE);
      DrawText("F2: Toggle dynamic sun | Arrow keys: Manual sun control", 10,
               145, 16, YELLOW);

      // Display sun direction and lighting info
      Vector3 sunDir = shaderSystem->getSunDirection();
      DrawText(TextFormat("Sun Dir: (%.2f, %.2f, %.2f)", sunDir.x, sunDir.y,
                          sunDir.z),
               10, 170, 16, ORANGE);
      DrawText(TextFormat("Dynamic Sun: %s", dynamicSun ? "ON" : "OFF"), 10,
               195, 16, dynamicSun ? GREEN : RED);
    }

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
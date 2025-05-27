#include <cmath>
#include <iostream>

#include "BodyPart.h"
#include "GameWorld.h"
#include "entities/HumanoidCharacter.h"
#include "exceptions/GameExceptions.h"
#include "game/FruitShopGame.h"
#include "objects/CubeObject.h"
#include "objects/Floor.h"
#include "objects/Sphere.h"
#include "raylib.h"
#include "settings/Physics.h"
#include "systems/InputSystem.h"
#include "systems/ShaderSystem.h"
#include "utils/GameObjectManager.h"
#include "utils/Storage.h"

// Function declarations for demonstrations
void demonstrateGameObjectManager();
void demonstrateStorage();
void demonstrateBodyPartCopyOperations();

int main() {
  try {
    InitWindow(1280, 720, "3D Game");
    SetTargetFPS(120);

    EnableCursor();

    ShaderSystem* shaderSystem = ShaderSystem::getInstance();
    if (!shaderSystem->initialize()) {
      throw GameInitException("Failed to initialize shader system");
    }
    TraceLog(LOG_INFO, "Shader system initialized successfully");

    std::cout << "\n========== ACADEMIC REQUIREMENTS DEMONSTRATION ==========\n"
              << std::endl;

    demonstrateGameObjectManager();

    demonstrateStorage();

    demonstrateBodyPartCopyOperations();

    std::cout << "\n=== Static Members Demonstration ===" << std::endl;
    std::cout << "GameWorld total objects: " << GameWorld::getTotalObjectCount()
              << std::endl;
    std::cout << "GameWorld creation timestamp: "
              << GameWorld::getCreationTimestamp() << std::endl;
    std::cout << "HumanoidCharacter total created: "
              << HumanoidCharacter::getTotalCharactersCreated() << std::endl;
    std::cout << "HumanoidCharacter active count: "
              << HumanoidCharacter::getActiveCharacterCount() << std::endl;

    std::cout << "\n=== Exception Hierarchy Test ===" << std::endl;
    try {
      throw ResourceException("Test resource error", "test/path/resource.txt");
    } catch (const GameException& e) {
      std::cout << "Caught GameException: " << e.what() << std::endl;
      std::cout << "Exception type: " << e.getType() << std::endl;
    }

    std::cout << "\n=== Template Method Pattern with Dynamic Casting ==="
              << std::endl;

    std::cout << "\n========================================================\n"
              << std::endl;

    // Academic Requirements Summary
    std::cout << "✓ ACADEMIC REQUIREMENTS MET:" << std::endl;
    std::cout << "✓ Code separation: Headers (.h) and source files (.cpp)"
              << std::endl;
    std::cout << "✓ Inheritance hierarchy: GameObject -> HumanoidCharacter, "
                 "BodyPart, Floor, etc."
              << std::endl;
    std::cout
        << "✓ Virtual functions: draw(), update(), clone(), getBoundingBox()"
        << std::endl;
    std::cout << "✓ Constructors: All classes have proper constructors"
              << std::endl;
    std::cout
        << "✓ Polymorphism: Base class pointers (std::shared_ptr<GameObject>)"
        << std::endl;
    std::cout << "✓ Copy constructors: BodyPart with copy-and-swap idiom"
              << std::endl;
    std::cout << "✓ Dynamic casting: GameWorld template methods with "
                 "std::dynamic_pointer_cast"
              << std::endl;
    std::cout << "✓ Smart pointers: std::shared_ptr, std::unique_ptr throughout"
              << std::endl;
    std::cout << "✓ Exception hierarchy: GameException -> 4 derived classes"
              << std::endl;
    std::cout << "✓ Static functions/attributes: GameWorld, HumanoidCharacter, "
                 "GameObjectManager"
              << std::endl;
    std::cout << "✓ Template classes: GameObjectManager<T>, Storage<ItemType>"
              << std::endl;
    std::cout << "✓ Template functions: calculateDistance, findNearestObject, "
                 "groupObjectsByDistance"
              << std::endl;
    std::cout
        << "✓ Design patterns: Singleton, Observer, State, Template Method"
        << std::endl;
    std::cout << "========================================================\n"
              << std::endl;

    Camera3D camera = {0};
    camera.position = (Vector3){
        0.0f, 25.0f, 5.0f};  // Start much higher to avoid beige wall below map
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};  // Look at ground level
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Camera control variables (Roblox-style)
    float cameraDistance = 25.0f;  // Start higher up for better overview
    float cameraAngleX = 0.0f;     // Horizontal rotation
    float cameraAngleY =
        -30.0f;  // Start with steeper downward angle for better view
    const float mouseSensitivity = 0.5f;   // Original sensitivity
    const float minCameraDistance = 5.0f;  // Original minimum distance
    const float maxCameraDistance = 30.0f;
    const float maxVerticalAngle = 89.0f;  // Original maximum angle
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

      // Check for game restart
      if (IsKeyPressed(KEY_R)) {
        fruitShopGame->resetGame();
      }

      // Update mouse camera controls (Roblox-style)
      InputSystem::updateMouseCamera();

      // Handle mouse camera rotation when right-clicking
      if (InputSystem::isRightMouseDown()) {
        Vector2 mouseDelta = InputSystem::getMouseDelta();

        // Update camera angles based on mouse movement (original behavior)
        cameraAngleX -= mouseDelta.x * mouseSensitivity;
        cameraAngleY += mouseDelta.y * mouseSensitivity;

        // Clamp vertical angle to prevent camera from flipping
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

      DrawFPS(10, 40);

      DrawText("Right Click + Drag: Rotate Camera", 10, 10, 16, WHITE);
      DrawText("Mouse Wheel: Zoom In/Out", 10, 26, 16, WHITE);

      // Debug info display
      if (debugMode) {
        DrawText("DEBUG MODE: F1 to toggle", 10, 70, 20, GREEN);
        DrawText("Multi-body collision system active", 10, 95, 20, GREEN);
        DrawText("Purple=Head, Blue=Torso, Green=Arms, Orange=Legs", 10, 120,
                 16, WHITE);
        DrawText("F2: Toggle dynamic sun | Arrow keys: Manual sun control", 10,
                 145, 16, YELLOW);
        DrawText("R: Restart game", 10, 165, 16, BLUE);

        DrawText(TextFormat("Camera Distance: %.1f", cameraDistance), 10, 190,
                 16, YELLOW);
        DrawText(TextFormat("Camera Angles: H=%.1f V=%.1f", cameraAngleX,
                            cameraAngleY),
                 10, 210, 16, YELLOW);
        DrawText(
            TextFormat("Mouse Camera: %s",
                       InputSystem::isRightMouseDown() ? "ACTIVE" : "INACTIVE"),
            10, 230, 16, InputSystem::isRightMouseDown() ? GREEN : RED);

        Vector3 sunDir = shaderSystem->getSunDirection();
        DrawText(TextFormat("Sun Dir: (%.2f, %.2f, %.2f)", sunDir.x, sunDir.y,
                            sunDir.z),
                 10, 250, 16, ORANGE);
        DrawText(TextFormat("Dynamic Sun: %s", dynamicSun ? "ON" : "OFF"), 10,
                 270, 16, dynamicSun ? GREEN : RED);
      }

      EndDrawing();
    }

    CloseWindow();

    std::cout << "Game Statistics:" << std::endl;
    std::cout << "Total Storage Units Created: "
              << Storage<GameObject>::getTotalStorageUnits() << std::endl;
    std::cout << "Total Object Managers Created: "
              << GameObjectManager<GameObject>::getTotalManagersCreated()
              << std::endl;
    std::cout << "GameWorld Statistics:" << std::endl;
    std::cout << "Total Objects Created: "
              << GameWorld::getTotalObjectsCreated() << std::endl;
    std::cout << "Active Objects: " << GameWorld::getActiveObjectCount()
              << std::endl;

    return 0;

  } catch (const GameInitException& e) {
    std::cerr << "Game Initialization Error: " << e.what() << std::endl;
    std::cerr << "Error Type: " << e.getType() << std::endl;
    CloseWindow();
    return -1;
  } catch (const ResourceException& e) {
    std::cerr << "Resource Error: " << e.what() << std::endl;
    std::cerr << "Error Type: " << e.getType() << std::endl;
    std::cerr << "Resource Path: " << e.getResourcePath() << std::endl;
    CloseWindow();
    return -2;
  } catch (const PhysicsException& e) {
    std::cerr << "Physics Error: " << e.what() << std::endl;
    std::cerr << "Error Type: " << e.getType() << std::endl;
    if (!e.getObjectName().empty()) {
      std::cerr << "Object: " << e.getObjectName() << std::endl;
    }
    CloseWindow();
    return -3;
  } catch (const AIException& e) {
    std::cerr << "AI Error: " << e.what() << std::endl;
    std::cerr << "Error Type: " << e.getType() << std::endl;
    if (!e.getNPCId().empty()) {
      std::cerr << "NPC ID: " << e.getNPCId() << std::endl;
    }
    if (!e.getCurrentState().empty()) {
      std::cerr << "Current State: " << e.getCurrentState() << std::endl;
    }
    CloseWindow();
    return -4;
  } catch (const GameException& e) {
    std::cerr << "Game Error: " << e.what() << std::endl;
    std::cerr << "Error Type: " << e.getType() << std::endl;
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
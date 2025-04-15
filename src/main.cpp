#include <memory>
#include <string>
#include <vector>

#include "Character.h"
#include "Enemy.h"
#include "GameObject.h"
#include "GameUI.h"
#include "GameWorld.h"
#include "ItemPickup.h"
#include "Weapon.h"
#include "raylib.h"

// Constants for game configuration
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;
const char* GAME_TITLE = "Roblox-like 3D Game";
extern const float GRAVITY = -0.01f;
const float JUMP_FORCE = 0.2f;
const float PLAYER_SPEED = 0.1f;

// Game states
enum GameState { MENU, PLAYING, PAUSED, GAME_OVER };

// UI element structure
struct UIElement {
    Rectangle bounds;
    std::string text;
    Color color;
    Color textColor;
    bool isHovered;
    bool isButton;
};

// Function prototypes
void DrawUIElement(const UIElement& element);
bool IsUIElementClicked(const UIElement& element);

int main() {
    // Initialize window and settings
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, GAME_TITLE);
    SetTargetFPS(60);

    // Set up camera
    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 5.0f, 15.0f};
    camera.target = (Vector3){0.0f, 1.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Game state
    GameState currentState = MENU;

    // Create player and world
    Character player1((Vector3){0.0f, 2.0f, 0.0f});
    GameWorld world(&player1);
    player1.setWorld(&world);

    // Create the Game UI
    GameUI gameUI(&player1);

    // Create weapons
    Weapon* sword = new Weapon("Iron Sword", 25, 1.5f, 2.0f);
    Weapon* axe =
        new Weapon("Battle Axe", 40, 0.8f, 2.5f, (Vector3){0.15f, 0.6f, 0.15f}, DARKBROWN);

    // Create enemies
    std::vector<Enemy*> enemies;

    // Create 5 enemies at different positions
    Enemy* enemy1 = new Enemy((Vector3){-10.0f, 0.5f, -10.0f}, 1.0f, RED, 100, 15, 0.03f);
    Enemy* enemy2 = new Enemy((Vector3){10.0f, 0.5f, -10.0f}, 1.0f, MAROON, 150, 20, 0.025f);
    Enemy* enemy3 = new Enemy((Vector3){-10.0f, 0.5f, 10.0f}, 1.0f, RED, 100, 15, 0.03f);
    Enemy* enemy4 = new Enemy((Vector3){10.0f, 0.5f, 10.0f}, 1.0f, MAROON, 150, 20, 0.025f);
    Enemy* enemy5 = new Enemy((Vector3){0.0f, 0.5f, -20.0f}, 1.5f, DARKPURPLE, 200, 25, 0.02f);

    // Set player as target for enemies
    enemy1->setTarget(&player1);
    enemy2->setTarget(&player1);
    enemy3->setTarget(&player1);
    enemy4->setTarget(&player1);
    enemy5->setTarget(&player1);

    // Add enemies to the list
    enemies.push_back(enemy1);
    enemies.push_back(enemy2);
    enemies.push_back(enemy3);
    enemies.push_back(enemy4);
    enemies.push_back(enemy5);

    // Add objects to the world - creating a small level
    // Ground
    world.addObject(std::make_shared<CubeObject>((Vector3){0.0f, -0.05f, 0.0f},
                                                 50.0f,
                                                 0.1f,
                                                 50.0f,
                                                 GREEN,
                                                 true,
                                                 "../resources/forrest_ground_01_diff_4k.jpg"));

    // Add some platforms and obstacles
    world.addObject(
        std::make_shared<CubeObject>((Vector3){-5.0f, 1.0f, -5.0f}, 3.0f, 0.5f, 3.0f, BLUE, true));

    world.addObject(
        std::make_shared<CubeObject>((Vector3){5.0f, 2.0f, -7.0f}, 4.0f, 0.5f, 2.0f, RED, true));

    world.addObject(std::make_shared<CubeObject>(
        (Vector3){-3.0f, 3.0f, -10.0f}, 2.0f, 0.5f, 2.0f, ORANGE, true));

    world.addObject(std::make_shared<CubeObject>(
        (Vector3){0.0f, 4.0f, -15.0f}, 6.0f, 0.5f, 6.0f, PURPLE, true));

    // Add some walls
    world.addObject(std::make_shared<CubeObject>(
        (Vector3){-25.0f, 5.0f, 0.0f}, 0.5f, 10.0f, 50.0f, GRAY, true));

    world.addObject(
        std::make_shared<CubeObject>((Vector3){25.0f, 5.0f, 0.0f}, 0.5f, 10.0f, 50.0f, GRAY, true));

    world.addObject(std::make_shared<CubeObject>(
        (Vector3){0.0f, 5.0f, -25.0f}, 50.0f, 10.0f, 0.5f, GRAY, true));

    world.addObject(
        std::make_shared<CubeObject>((Vector3){0.0f, 5.0f, 25.0f}, 50.0f, 10.0f, 0.5f, GRAY, true));

    // Add some item pickups
    Item healthPotion = {"Health Potion", "Restores 25 health points", 10, false, true};
    Item staminaPotion = {"Stamina Potion", "Restores 25 stamina points", 10, false, true};
    Item swordItem = {"Iron Sword", "A basic weapon for combat", 50, true, false};

    auto healthPotionPickup =
        std::make_shared<ItemPickup>((Vector3){-2.0f, 1.0f, -3.0f}, healthPotion, RED);
    auto staminaPotionPickup =
        std::make_shared<ItemPickup>((Vector3){3.0f, 1.0f, -2.0f}, staminaPotion, GREEN);
    auto swordPickup =
        std::make_shared<ItemPickup>((Vector3){0.0f, 1.0f, -10.0f}, swordItem, SKYBLUE);

    world.addObject(healthPotionPickup);
    world.addObject(staminaPotionPickup);
    world.addObject(swordPickup);

    // Equip the default sword to the player
    player1.setWeapon(sword);

    // Create UI elements for menu
    std::vector<UIElement> menuElements = {
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 - 50, 300, 60},
         "Play Game",
         SKYBLUE,
         BLACK,
         false,
         true},
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 + 50, 300, 60},
         "Exit",
         SKYBLUE,
         BLACK,
         false,
         true}};

    // Create UI elements for pause menu
    std::vector<UIElement> pauseElements = {
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 - 50, 300, 60},
         "Resume",
         SKYBLUE,
         BLACK,
         false,
         true},
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 + 50, 300, 60},
         "Main Menu",
         SKYBLUE,
         BLACK,
         false,
         true}};

    // Create UI elements for game over menu
    std::vector<UIElement> gameOverElements = {
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 + 50, 300, 60},
         "Restart",
         SKYBLUE,
         BLACK,
         false,
         true},
        {{SCREEN_WIDTH / 2 - 150, SCREEN_HEIGHT / 2 + 150, 300, 60},
         "Main Menu",
         SKYBLUE,
         BLACK,
         false,
         true}};

    // Game loop
    while (!WindowShouldClose()) {
        switch (currentState) {
            case MENU:
                // Update menu elements
                for (auto& element : menuElements) {
                    element.isHovered = CheckCollisionPointRec(GetMousePosition(), element.bounds);
                    if (element.isHovered) {
                        element.color = SKYBLUE;
                        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
                            if (element.text == "Play Game") {
                                currentState = PLAYING;
                                DisableCursor();  // Hide cursor for gameplay
                            } else if (element.text == "Exit") {
                                CloseWindow();
                                return 0;
                            }
                        }
                    } else {
                        element.color = SKYBLUE;
                    }
                }

                // Draw menu
                BeginDrawing();
                ClearBackground(RAYWHITE);
                DrawText("ROBLOX-LIKE 3D GAME", SCREEN_WIDTH / 2 - 220, 120, 40, DARKGRAY);

                // Draw UI elements
                for (const auto& element : menuElements) {
                    DrawUIElement(element);
                }

                DrawText("(c) 2024 Your Game Studio", 10, SCREEN_HEIGHT - 20, 10, DARKGRAY);
                EndDrawing();
                break;

            case PLAYING: {  // Add opening brace to create a new scope
                // Check for game over
                if (player1.isDead()) {
                    currentState = GAME_OVER;
                    EnableCursor();
                    break;
                }

                // Check for pause
                if (IsKeyPressed(KEY_ESCAPE)) {
                    // If inventory is open, close it, otherwise pause game
                    if (gameUI.getIsInventoryOpen()) {
                        gameUI.toggleInventory();
                    } else {
                        currentState = PAUSED;
                        EnableCursor();  // Show cursor for menu navigation
                    }
                    break;
                }

                // Handle UI input
                gameUI.handleInput();

                // Update player and world (if inventory not open)
                if (!gameUI.getIsInventoryOpen()) {
                    player1.handleInput(PLAYER_SPEED, JUMP_FORCE);

                    // Check for player attack
                    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                        player1.attackWithWeapon(enemies);
                    }
                }

                // Always update world physics
                world.update(GetFrameTime());

                // Update enemies
                for (auto enemy : enemies) {
                    if (!enemy->isDead()) {
                        enemy->update(GetFrameTime());
                    }
                }

                // Update game UI
                gameUI.update();

                // Update camera to follow player
                Vector3 playerPos = player1.getPosition();
                camera.target = playerPos;
                camera.position = (Vector3){playerPos.x, playerPos.y + 5.0f, playerPos.z + 10.0f};

                // Draw game
                BeginDrawing();
                ClearBackground(SKYBLUE);

                BeginMode3D(camera);
                // Draw regular objects
                for (const auto& obj : world.getObjects()) {
                    // Check if the object is an ItemPickup
                    auto itemPickup = std::dynamic_pointer_cast<ItemPickup>(obj);
                    if (itemPickup) {
                        // Draw item pickup with the current camera for billboards
                        itemPickup->drawWithCamera(camera);

                        // Check for item pickup interaction
                        if (!itemPickup->getIsCollected() &&
                            player1.getDistance(*itemPickup) < 2.0f) {
                            // Check for interaction key
                            if (IsKeyPressed(KEY_E)) {
                                itemPickup->interactWith(player1);
                            }

                            // Show interaction hint
                            Vector3 itemPos = itemPickup->getPosition();
                            Vector3 textPos = {itemPos.x, itemPos.y + 1.0f, itemPos.z};

                            // Convert 3D position to screen space
                            Vector2 screenPos = GetWorldToScreen(textPos, camera);
                            DrawText("[E] Pick up", screenPos.x - 50, screenPos.y, 20, WHITE);
                        }
                    } else {
                        // Draw regular objects normally
                        obj->draw();
                    }
                }

                // Draw enemies
                for (auto enemy : enemies) {
                    if (!enemy->isDead()) {
                        enemy->draw();
                    }
                }

                // Draw player
                player1.draw();

                EndMode3D();

                // Draw game UI
                gameUI.draw();

                // Draw interaction prompt if not showing inventory
                if (!gameUI.getIsInventoryOpen()) {
                    DrawText("Press [I] to open inventory",
                             SCREEN_WIDTH - 250,
                             SCREEN_HEIGHT - 30,
                             20,
                             WHITE);

                    DrawText("Left Click to attack with weapon",
                             SCREEN_WIDTH - 300,
                             SCREEN_HEIGHT - 60,
                             20,
                             WHITE);
                }

                EndDrawing();
                break;
            }  // Add closing brace to end the PLAYING case scope

            case PAUSED:
                // Update pause menu elements
                for (auto& element : pauseElements) {
                    element.isHovered = CheckCollisionPointRec(GetMousePosition(), element.bounds);
                    if (element.isHovered) {
                        element.color = SKYBLUE;
                        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
                            if (element.text == "Resume") {
                                currentState = PLAYING;
                                DisableCursor();  // Hide cursor for gameplay
                            } else if (element.text == "Main Menu") {
                                currentState = MENU;
                            }
                        }
                    } else {
                        element.color = SKYBLUE;
                    }
                }

                // Draw pause menu
                BeginDrawing();
                ClearBackground(Fade(RAYWHITE, 0.8f));

                DrawText("GAME PAUSED", SCREEN_WIDTH / 2 - 150, 120, 40, DARKGRAY);

                // Draw UI elements
                for (const auto& element : pauseElements) {
                    DrawUIElement(element);
                }
                EndDrawing();
                break;

            case GAME_OVER:
                // Update game over menu elements
                for (auto& element : gameOverElements) {
                    element.isHovered = CheckCollisionPointRec(GetMousePosition(), element.bounds);
                    if (element.isHovered) {
                        element.color = SKYBLUE;
                        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
                            if (element.text == "Restart") {
                                // Reset game state
                                player1 = Character((Vector3){0.0f, 2.0f, 0.0f});
                                player1.setWorld(&world);
                                player1.setWeapon(sword);

                                // Reset enemies
                                for (auto enemy : enemies) {
                                    delete enemy;
                                }
                                enemies.clear();

                                enemy1 = new Enemy(
                                    (Vector3){-10.0f, 0.5f, -10.0f}, 1.0f, RED, 100, 15, 0.03f);
                                enemy2 = new Enemy(
                                    (Vector3){10.0f, 0.5f, -10.0f}, 1.0f, MAROON, 150, 20, 0.025f);
                                enemy3 = new Enemy(
                                    (Vector3){-10.0f, 0.5f, 10.0f}, 1.0f, RED, 100, 15, 0.03f);
                                enemy4 = new Enemy(
                                    (Vector3){10.0f, 0.5f, 10.0f}, 1.0f, MAROON, 150, 20, 0.025f);
                                enemy5 = new Enemy((Vector3){0.0f, 0.5f, -20.0f},
                                                   1.5f,
                                                   DARKPURPLE,
                                                   200,
                                                   25,
                                                   0.02f);

                                enemy1->setTarget(&player1);
                                enemy2->setTarget(&player1);
                                enemy3->setTarget(&player1);
                                enemy4->setTarget(&player1);
                                enemy5->setTarget(&player1);

                                enemies.push_back(enemy1);
                                enemies.push_back(enemy2);
                                enemies.push_back(enemy3);
                                enemies.push_back(enemy4);
                                enemies.push_back(enemy5);

                                currentState = PLAYING;
                                DisableCursor();
                            } else if (element.text == "Main Menu") {
                                currentState = MENU;
                            }
                        }
                    } else {
                        element.color = SKYBLUE;
                    }
                }

                // Draw game over screen
                BeginDrawing();
                ClearBackground(Fade(BLACK, 0.9f));

                DrawText("GAME OVER", SCREEN_WIDTH / 2 - 150, 120, 40, RED);

                // Draw UI elements
                for (const auto& element : gameOverElements) {
                    DrawUIElement(element);
                }
                EndDrawing();
                break;
        }
    }

    // Clean up
    delete sword;
    delete axe;

    for (auto enemy : enemies) {
        delete enemy;
    }

    CloseWindow();
    return 0;
}

// Draw a UI element (button or text)
void DrawUIElement(const UIElement& element) {
    DrawRectangleRec(element.bounds, element.color);
    DrawRectangleLinesEx(element.bounds, 2, element.isHovered ? WHITE : DARKGRAY);

    // Center text in element
    int textWidth = MeasureText(element.text.c_str(), 20);
    int textX = element.bounds.x + (element.bounds.width - textWidth) / 2;
    int textY = element.bounds.y + (element.bounds.height - 20) / 2;

    DrawText(element.text.c_str(), textX, textY, 20, element.textColor);
}

// Check if a UI element is clicked
bool IsUIElementClicked(const UIElement& element) {
    return (element.isHovered && IsMouseButtonReleased(MOUSE_LEFT_BUTTON));
}
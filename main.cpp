#include "raylib.h"

int main() {
    InitWindow(1280, 720, "Joc 3D");
    SetTargetFPS(120);

    DisableCursor();

    Vector3 playerPosition = {0.0f, 0.5f, 0.0f};
    Vector3 playerVelocity = {0.0f, 0.0f, 0.0f};
    bool isOnGround = false;
    const float gravity = -0.01f;
    const float jumpForce = 0.2f;

    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 5.0f, 15.0f};
    camera.target = (Vector3){0.0f, 1.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Texture2D grass = LoadTexture("resources/forrest_ground_01_diff_4k.jpg");
    Model ground = LoadModelFromMesh(GenMeshPlane(20.0f, 20.0f, 1, 1));
    ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = grass;

    while (!WindowShouldClose()) {
        if (IsKeyDown(KEY_W)) playerPosition.z -= 0.1f;
        if (IsKeyDown(KEY_S)) playerPosition.z += 0.1f;
        if (IsKeyDown(KEY_A)) playerPosition.x -= 0.1f;
        if (IsKeyDown(KEY_D)) playerPosition.x += 0.1f;

        if (IsKeyPressed(KEY_SPACE) && isOnGround) {
            playerVelocity.y = jumpForce;
            isOnGround = false;
        }

        // Graivty
        playerVelocity.y += gravity;
        playerPosition.y += playerVelocity.y;

        if (playerPosition.y <= 0.5f) {
            playerPosition.y = 0.5f;
            playerVelocity.y = 0;
            isOnGround = true;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        // DrawGrid(20, 1.0f);

        // Random cube
        DrawCube((Vector3){0, 0.5f, 0}, 1, 1, 1, RED);

        // Player
        DrawCube(playerPosition, 1.0f, 1.0f, 1.0f, BLUE);

        // Floor
        DrawModel(ground, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
        EndMode3D();

        DrawText("Hello World!", 350, 280, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
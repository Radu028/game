#include "Player.h"
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

    Texture2D grass = LoadTexture("../resources/forrest_ground_01_diff_4k.jpg");
    Model ground = LoadModelFromMesh(GenMeshPlane(20.0f, 20.0f, 1, 1));
    ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = grass;

    Player player1;

    while (!WindowShouldClose()) {
        if (IsKeyDown(KEY_W)) player1.moveForward(0.1f);
        if (IsKeyDown(KEY_S)) player1.moveBackwards(0.1f);
        if (IsKeyDown(KEY_A)) player1.moveLeft(0.1f);
        if (IsKeyDown(KEY_D)) player1.moveRight(0.1f);
        if (IsKeyPressed(KEY_SPACE)) player1.jump(jumpForce);

        player1.applyGravity(gravity);
        player1.checkGroundCollision(0.5f);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        // DrawGrid(20, 1.0f);

        // Random cube
        DrawCube((Vector3){0, 0.5f, 0}, 1, 1, 1, RED);

        // Player
        DrawCube(player1.getPosition(), 1.0f, 1.0f, 1.0f, BLUE);

        // Floor
        DrawModel(ground, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
        EndMode3D();

        DrawText("Hello World!", 350, 280, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
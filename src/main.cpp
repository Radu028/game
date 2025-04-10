#include "GameObject.h"
#include "GameWorld.h"
#include "Player.h"
#include "inputFunctions.h"
#include "raylib.h"

const float GRAVITY = -0.01f;
const float JUMP_FORCE = 0.2f;

int main() {
    InitWindow(1280, 720, "Joc 3D");
    SetTargetFPS(120);

    DisableCursor();

    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 5.0f, 15.0f};
    camera.target = (Vector3){0.0f, 1.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Texture2D grass = LoadTexture("../resources/forrest_ground_01_diff_4k.jpg");
    Model ground = LoadModelFromMesh(GenMeshPlane(50.0f, 50.0f, 1, 1));
    ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = grass;

    Player player1;
    GameWorld world(&player1);
    player1.setWorld(&world);

    world.addObject(std::make_shared<GameObject>(
        (Vector3){3.0f, 0.5f, 0.0f}, (Vector3){1.0f, 1.0f, 1.0f}, RED));

    while (!WindowShouldClose()) {
        handleInput(player1, 0.1f, JUMP_FORCE);

        world.update(GetFrameTime());

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        // Player
        player1.draw();

        world.draw();

        // Floor
        DrawModel(ground, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
        EndMode3D();

        DrawText("Hello World!", 350, 280, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
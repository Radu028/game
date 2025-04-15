#include "ItemPickup.h"

#include "raylib.h"
#include "raymath.h"

ItemPickup::ItemPickup(Vector3 position, const Item& item, Color color, float size)
    : CubeObject(position, size, size, size, color, false),  // No collision for floating effect
      item(item),
      rotationAngle(0.0f),
      floatOffset(0.0f),
      floatTimer(0.0f),
      isCollected(false) {}

bool ItemPickup::interactWith(Character& character) {
    if (!isCollected) {
        character.addItem(item);
        isCollected = true;
        return true;
    }
    return false;
}

void ItemPickup::update(float deltaTime) {
    if (isCollected) return;

    // Rotate the item pickup
    rotationAngle += 90.0f * deltaTime;  // 90 degrees per second
    if (rotationAngle >= 360.0f) {
        rotationAngle -= 360.0f;
    }

    // Float up and down
    floatTimer += deltaTime;
    floatOffset = sinf(floatTimer * 2.0f) * 0.2f;  // Oscillate between -0.2 and 0.2

    // Update the Y position for floating effect
    Vector3 newPosition = position;
    newPosition.y += floatOffset;
    position = newPosition;
}

void ItemPickup::draw() const {
    // Create a default camera for the basic draw call
    Camera camera = {0};
    camera.position = (Vector3){0.0f, 10.0f, 10.0f};
    camera.target = position;
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    drawWithCamera(camera);
}

void ItemPickup::drawWithCamera(const Camera& camera) const {
    if (isCollected) return;

    // Draw the item with rotation
    if (hasTexture) {
        // Draw model with matrix transformation for rotation
        DrawModelEx(model, position, (Vector3){0, 1, 0}, rotationAngle, (Vector3){1, 1, 1}, color);
    } else {
        // Draw cube at the position with rotation
        DrawCube(position, width, height, length, color);
        DrawCubeWires(position, width, height, length, BLACK);
    }

    // Draw item name above (as billboard text)
    Vector3 textPosition = {position.x, position.y + height + 0.3f, position.z};
    // DrawBillboard has 5 parameters: camera, texture, position, scale, tint
    // We need to load a texture from the font
    Image tempImage = ImageText(item.name.c_str(), 20, WHITE);
    Texture2D tempTexture = LoadTextureFromImage(tempImage);

    DrawBillboard(camera, tempTexture, textPosition, 0.02f, WHITE);

    // Clean up resources
    UnloadTexture(tempTexture);
    UnloadImage(tempImage);
}
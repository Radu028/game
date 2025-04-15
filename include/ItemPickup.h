#ifndef ITEMPICKUP_H
#define ITEMPICKUP_H

#include <string>

#include "Character.h"
#include "CubeObject.h"

class ItemPickup : public CubeObject {
   private:
    Item item;
    float rotationAngle;
    float floatOffset;
    float floatTimer;
    bool isCollected;

   public:
    ItemPickup(Vector3 position, const Item& item, Color color = GOLD, float size = 0.5f);

    // Get the item this pickup represents
    const Item& getItem() const { return item; }

    // Check if this item has been collected
    bool getIsCollected() const { return isCollected; }

    // Set collected state
    void setCollected(bool collected) { isCollected = collected; }

    // Interact with a character
    bool interactWith(Character& character);

    // Override update to add floating and rotation effects
    void update(float deltaTime) override;

    // Override draw to include effects
    void draw() const override;

    // Draw with a camera (for billboard rendering)
    void drawWithCamera(const Camera& camera) const;
};

#endif  // ITEMPICKUP_H
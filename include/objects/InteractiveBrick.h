#pragma once

#include "objects/CubeObject.h"
#include "entities/HumanoidCharacter.h"

class InteractiveBrick : public CubeObject {
private:
    bool isPickedUp = false;
    bool isPlaceable = true;
    float interactionRadius = 2.0f;
    HumanoidCharacter* carriedBy = nullptr;
    Vector3 carryOffset = {0, 2.0f, 1.0f}; // Relative to character
    
public:
    InteractiveBrick(Vector3 position, Vector3 size, Color color);
    ~InteractiveBrick() override;
    
    void update(float deltaTime) override;
    void draw() const override;
    void interact() override;
    
    // Building mechanics
    bool canBePickedUp(const HumanoidCharacter* character) const;
    bool canBePlaced(Vector3 position) const;
    void pickUp(HumanoidCharacter* character);
    void place(Vector3 position);
    void drop();
    
    // Status
    bool getIsPickedUp() const { return isPickedUp; }
    bool getIsPlaceable() const { return isPlaceable; }
    
    // Visual feedback
    Color getHighlightColor() const;
    void drawInteractionPrompt(const HumanoidCharacter* character) const;
    
    std::shared_ptr<GameObject> clone() const override {
        return std::make_shared<InteractiveBrick>(*this);
    }
};

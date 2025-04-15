#ifndef GAMEUI_H
#define GAMEUI_H

#include <string>
#include <vector>

#include "Character.h"

// UI component types
enum UIComponentType {
    UI_HEALTH_BAR,
    UI_STAMINA_BAR,
    UI_EXP_BAR,
    UI_INVENTORY,
    UI_EQUIPMENT,
    UI_STATS,
    UI_DIALOG,
    UI_HOTBAR
};

// UI component base class
class UIComponent {
   protected:
    Rectangle bounds;
    bool isVisible;
    UIComponentType type;

   public:
    UIComponent(Rectangle bounds, UIComponentType type, bool isVisible = true)
        : bounds(bounds), isVisible(isVisible), type(type) {}

    virtual ~UIComponent() = default;

    void setVisible(bool visible) { isVisible = visible; }
    bool getVisible() const { return isVisible; }
    UIComponentType getType() const { return type; }
    Rectangle getBounds() const { return bounds; }

    virtual void update() {}
    virtual void draw() const = 0;
    virtual bool handleMouse(Vector2 mousePosition, bool isClicked) { return false; }
};

// Health/Stamina/Exp Bar Component
class StatusBarComponent : public UIComponent {
   private:
    Character* character;
    Color backgroundColor;
    Color foregroundColor;
    std::string label;
    float (*getCurrentValueFn)(const Character*);
    float (*getMaxValueFn)(const Character*);

   public:
    StatusBarComponent(Rectangle bounds, UIComponentType type, Character* character,
                       const std::string& label, Color foreground, Color background,
                       float (*getCurrentValueFn)(const Character*),
                       float (*getMaxValueFn)(const Character*));

    void draw() const override;
};

// Inventory Component
class InventoryComponent : public UIComponent {
   private:
    Character* character;
    int selectedIndex;
    int scrollOffset;

   public:
    InventoryComponent(Rectangle bounds, Character* character);

    void draw() const override;
    bool handleMouse(Vector2 mousePosition, bool isClicked) override;
    void scrollUp();
    void scrollDown();
    void selectItem(int index);
    int getSelectedIndex() const { return selectedIndex; }
};

// Main Game UI class
class GameUI {
   private:
    std::vector<UIComponent*> components;
    Character* character;
    bool isInventoryOpen;

    // Helper functions for creating status bars
    static float getHealthValue(const Character* character) { return character->getHealth(); }
    static float getMaxHealthValue(const Character* character) { return character->getMaxHealth(); }
    static float getStaminaValue(const Character* character) { return character->getStamina(); }
    static float getMaxStaminaValue(const Character* character) {
        return character->getMaxStamina();
    }
    static float getExpValue(const Character* character) { return character->getExperience(); }
    static float getExpMaxValue(const Character* character) {
        return character->getExperienceToNextLevel();
    }

   public:
    GameUI(Character* character);
    ~GameUI();

    void toggleInventory();
    bool getIsInventoryOpen() const { return isInventoryOpen; }

    void update();
    void draw() const;
    void handleInput();

    UIComponent* getComponent(UIComponentType type) const;
};

#endif  // GAMEUI_H
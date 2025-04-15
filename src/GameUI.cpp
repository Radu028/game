#include "GameUI.h"

#include "raylib.h"

// StatusBarComponent Implementation
StatusBarComponent::StatusBarComponent(Rectangle bounds, UIComponentType type, Character* character,
                                       const std::string& label, Color foreground, Color background,
                                       float (*getCurrentValueFn)(const Character*),
                                       float (*getMaxValueFn)(const Character*))
    : UIComponent(bounds, type),
      character(character),
      backgroundColor(background),
      foregroundColor(foreground),
      label(label),
      getCurrentValueFn(getCurrentValueFn),
      getMaxValueFn(getMaxValueFn) {}

void StatusBarComponent::draw() const {
    if (!isVisible) return;

    // Draw background
    DrawRectangleRec(bounds, backgroundColor);

    // Calculate fill amount based on current/max value
    float currentValue = getCurrentValueFn(character);
    float maxValue = getMaxValueFn(character);
    float fillRatio = currentValue / maxValue;

    // Draw filled portion
    Rectangle fillRect = bounds;
    fillRect.width *= fillRatio;
    DrawRectangleRec(fillRect, foregroundColor);

    // Draw border
    DrawRectangleLinesEx(bounds, 1, BLACK);

    // Draw label text
    DrawText(label.c_str(), bounds.x + 5, bounds.y + 5, 18, WHITE);

    // Draw value text (e.g., "75/100")
    char valueText[32];
    snprintf(valueText, sizeof(valueText), "%.0f/%.0f", currentValue, maxValue);
    int textWidth = MeasureText(valueText, 18);
    DrawText(valueText, bounds.x + bounds.width - textWidth - 5, bounds.y + 5, 18, WHITE);
}

// InventoryComponent Implementation
InventoryComponent::InventoryComponent(Rectangle bounds, Character* character)
    : UIComponent(bounds, UI_INVENTORY, false),
      character(character),
      selectedIndex(-1),
      scrollOffset(0) {}

void InventoryComponent::draw() const {
    if (!isVisible) return;

    // Draw inventory background
    DrawRectangleRec(bounds, Fade(DARKGRAY, 0.8f));
    DrawRectangleLinesEx(bounds, 2, BLACK);

    // Draw title
    DrawText("INVENTORY", bounds.x + 10, bounds.y + 10, 24, WHITE);

    // Get inventory items
    const std::vector<Item>& inventory = character->getInventory();

    // Draw items
    const int itemHeight = 40;
    const int maxVisibleItems = (bounds.height - 60) / itemHeight;
    const float startY = bounds.y + 50;

    // Draw scrollbar if needed
    if (inventory.size() > maxVisibleItems) {
        Rectangle scrollBarBG = {
            bounds.x + bounds.width - 20, bounds.y + 50, 10, bounds.height - 60};
        DrawRectangleRec(scrollBarBG, LIGHTGRAY);

        float scrollRatio = (float)maxVisibleItems / inventory.size();
        float scrollHandleHeight = scrollBarBG.height * scrollRatio;
        float scrollHandleY =
            scrollBarBG.y +
            scrollBarBG.height * ((float)scrollOffset / (inventory.size() - maxVisibleItems));
        if (scrollHandleY + scrollHandleHeight > scrollBarBG.y + scrollBarBG.height) {
            scrollHandleY = scrollBarBG.y + scrollBarBG.height - scrollHandleHeight;
        }

        Rectangle scrollHandle = {
            scrollBarBG.x, scrollHandleY, scrollBarBG.width, scrollHandleHeight};
        DrawRectangleRec(scrollHandle, DARKGRAY);
    }

    // Draw item list
    for (int i = 0; i < maxVisibleItems && i + scrollOffset < inventory.size(); i++) {
        int idx = i + scrollOffset;
        const Item& item = inventory[idx];

        Rectangle itemRect = {bounds.x + 10,
                              startY + (float)i * itemHeight,
                              bounds.width - 30,
                              (float)itemHeight - 5};

        // Highlight selected item
        if (idx == selectedIndex) {
            DrawRectangleRec(itemRect, SKYBLUE);
        }

        // Draw item name
        DrawText(item.name.c_str(), itemRect.x + 5, itemRect.y + 5, 20, WHITE);

        // Draw item type/properties
        std::string itemType;
        if (item.isEquippable) itemType += "[Equip] ";
        if (item.isConsumable) itemType += "[Use] ";
        DrawText(itemType.c_str(), itemRect.x + 5, itemRect.y + 25, 15, LIGHTGRAY);

        // Draw line separator
        DrawLine(itemRect.x,
                 itemRect.y + itemHeight - 2,
                 itemRect.x + itemRect.width,
                 itemRect.y + itemHeight - 2,
                 GRAY);
    }

    // Draw empty inventory message
    if (inventory.empty()) {
        const char* emptyText = "Inventory is empty";
        int textWidth = MeasureText(emptyText, 20);
        DrawText(emptyText,
                 bounds.x + (bounds.width - textWidth) / 2,
                 bounds.y + bounds.height / 2 - 10,
                 20,
                 GRAY);
    }

    // Draw usage instructions
    DrawText("Click to select, [E] Equip, [U] Use, [ESC] Close",
             bounds.x + 10,
             bounds.y + bounds.height - 30,
             15,
             LIGHTGRAY);
}

bool InventoryComponent::handleMouse(Vector2 mousePosition, bool isClicked) {
    if (!isVisible) return false;

    // Check if mouse is inside inventory area
    if (!CheckCollisionPointRec(mousePosition, bounds)) return false;

    // Get inventory
    const std::vector<Item>& inventory = character->getInventory();
    if (inventory.empty()) return true;

    // Calculate item click areas
    const int itemHeight = 40;
    const int maxVisibleItems = (bounds.height - 60) / itemHeight;
    const float startY = bounds.y + 50;

    for (int i = 0; i < maxVisibleItems && i + scrollOffset < inventory.size(); i++) {
        Rectangle itemRect = {bounds.x + 10,
                              startY + (float)i * itemHeight,
                              bounds.width - 30,
                              (float)itemHeight - 5};

        if (CheckCollisionPointRec(mousePosition, itemRect)) {
            if (isClicked) {
                selectItem(i + scrollOffset);
            }
            return true;
        }
    }

    // Handle scrollbar
    if (inventory.size() > maxVisibleItems) {
        Rectangle scrollBarBG = {
            bounds.x + bounds.width - 20, bounds.y + 50, 10, bounds.height - 60};

        if (CheckCollisionPointRec(mousePosition, scrollBarBG) && isClicked) {
            // Calculate new scroll position
            float scrollRatio = (mousePosition.y - scrollBarBG.y) / scrollBarBG.height;
            scrollOffset = (int)(scrollRatio * (inventory.size() - maxVisibleItems));

            // Clamp scroll offset
            if (scrollOffset < 0) scrollOffset = 0;
            if (scrollOffset > inventory.size() - maxVisibleItems)
                scrollOffset = inventory.size() - maxVisibleItems;

            return true;
        }
    }

    return true;  // Consumed input but did not process a click
}

void InventoryComponent::scrollUp() {
    if (scrollOffset > 0) {
        scrollOffset--;
    }
}

void InventoryComponent::scrollDown() {
    const std::vector<Item>& inventory = character->getInventory();
    const int maxVisibleItems = (bounds.height - 60) / 40;

    if (scrollOffset < inventory.size() - maxVisibleItems) {
        scrollOffset++;
    }
}

void InventoryComponent::selectItem(int index) {
    const std::vector<Item>& inventory = character->getInventory();
    if (index >= 0 && index < inventory.size()) {
        selectedIndex = index;
    }
}

// GameUI Implementation
GameUI::GameUI(Character* character) : character(character), isInventoryOpen(false) {
    // Create health bar
    components.push_back(new StatusBarComponent((Rectangle){10, 10, 200, 30},
                                                UI_HEALTH_BAR,
                                                character,
                                                "HP",
                                                RED,
                                                DARKGRAY,
                                                getHealthValue,
                                                getMaxHealthValue));

    // Create stamina bar
    components.push_back(new StatusBarComponent((Rectangle){10, 50, 200, 30},
                                                UI_STAMINA_BAR,
                                                character,
                                                "SP",
                                                GREEN,
                                                DARKGRAY,
                                                getStaminaValue,
                                                getMaxStaminaValue));

    // Create experience bar
    components.push_back(new StatusBarComponent((Rectangle){10, 90, 200, 30},
                                                UI_EXP_BAR,
                                                character,
                                                "EXP",
                                                BLUE,
                                                DARKGRAY,
                                                getExpValue,
                                                getExpMaxValue));

    // Create inventory panel
    components.push_back(new InventoryComponent(
        (Rectangle){(float)(GetScreenWidth() - 310), 50, 300, (float)(GetScreenHeight() - 100)},
        character));
}

GameUI::~GameUI() {
    for (auto component : components) {
        delete component;
    }
    components.clear();
}

void GameUI::toggleInventory() {
    isInventoryOpen = !isInventoryOpen;
    UIComponent* inventoryComponent = getComponent(UI_INVENTORY);
    if (inventoryComponent) {
        inventoryComponent->setVisible(isInventoryOpen);
    }

    // If inventory is open, show cursor, otherwise hide it
    if (isInventoryOpen) {
        EnableCursor();
    } else {
        DisableCursor();
    }
}

void GameUI::update() {
    for (auto component : components) {
        component->update();
    }
}

void GameUI::draw() const {
    for (auto component : components) {
        component->draw();
    }

    // Draw additional info when inventory is open
    if (isInventoryOpen) {
        // Draw level text
        char levelText[32];
        snprintf(levelText, sizeof(levelText), "Level: %d", character->getLevel());
        DrawText(levelText, 10, GetScreenHeight() - 40, 20, WHITE);
    }
}

void GameUI::handleInput() {
    // Toggle inventory
    if (IsKeyPressed(KEY_I)) {
        toggleInventory();
    }

    // If inventory is open, handle inventory-specific inputs
    if (isInventoryOpen) {
        InventoryComponent* inventoryComponent =
            static_cast<InventoryComponent*>(getComponent(UI_INVENTORY));

        if (inventoryComponent) {
            // Handle mouse input for inventory
            Vector2 mousePosition = GetMousePosition();
            bool isClicked = IsMouseButtonReleased(MOUSE_LEFT_BUTTON);
            inventoryComponent->handleMouse(mousePosition, isClicked);

            // Handle scroll input
            if (GetMouseWheelMove() > 0) {
                inventoryComponent->scrollUp();
            } else if (GetMouseWheelMove() < 0) {
                inventoryComponent->scrollDown();
            }

            // Handle item use/equip for selected item
            int selectedIndex = inventoryComponent->getSelectedIndex();
            if (selectedIndex >= 0) {
                if (IsKeyPressed(KEY_U)) {
                    character->useItem(selectedIndex);
                } else if (IsKeyPressed(KEY_E)) {
                    character->equipItem(selectedIndex);
                }
            }
        }
    }
}

UIComponent* GameUI::getComponent(UIComponentType type) const {
    for (auto component : components) {
        if (component->getType() == type) {
            return component;
        }
    }
    return nullptr;
}
#pragma once
#include <btBulletDynamicsCommon.h>
#include "raylib.h"

// Using namespace instead of struct for better organization and extensibility
namespace GameSettings {
    // Physics constants
    namespace Physics {
        static constexpr float GRAVITY_ACCELERATION = -19.6f; // Reduced gravity for better stability
    }
    
    // Player/Character constants
    namespace Character {
        static constexpr float HEIGHT = 1.8f;
        static constexpr float RADIUS = 0.3f;
        static constexpr float MASS = 70.0f;
        static constexpr float MOVEMENT_FORCE = 1500.0f;
        static constexpr float JUMP_IMPULSE = 150.0f; // Much stronger jump power
        static constexpr float MAX_SPEED = 8.0f;
        static constexpr float DAMPING_LINEAR = 0.95f; // Balanced damping - smooth but responsive
        static constexpr float DAMPING_ANGULAR = 0.8f;
        static constexpr float MOVEMENT_SPEED = 5.0f;
    }
    
    // Camera constants
    namespace Camera {
        static const Vector3 OFFSET = {0.0f, 3.0f, 8.0f};
        static constexpr float TARGET_Y_OFFSET = 1.5f;
    }
    
    // Body part proportions for Lego-style character
    namespace BodyParts {
        // Head proportions (cubic/blocky, true Lego minifigure head)
        static const Vector3 HEAD_SIZE = {0.5f, 0.5f, 0.5f};
        static const Color HEAD_COLOR = YELLOW;
        
        // Torso proportions (classic Lego brick proportions)
        static const Vector3 TORSO_SIZE = {0.7f, 0.9f, 0.4f};
        static const Color TORSO_COLOR = BLUE;
        
        // Arms proportions (proportional to torso)
        static const Vector3 ARM_SIZE = {0.25f, 0.6f, 0.25f};
        static const Color ARM_COLOR = YELLOW;
        
        // Legs proportions (proportional and stable)
        static const Vector3 LEG_SIZE = {0.3f, 0.7f, 0.3f};
        static const Color LEG_COLOR = RED;
        
        // Positioning offsets (realistic for new proportions)
        static constexpr float ARM_OFFSET_X = 0.48f;     // Distance from center to arms
        static constexpr float LEG_OFFSET_X = 0.18f;     // Distance from center to legs
        static constexpr float HEAD_OFFSET_Y = 0.05f;    // Additional height for head positioning
    }
    
    // Animation constants
    namespace Animation {
        static constexpr float SPEED_THRESHOLD = 0.25f;
        static constexpr float SPEED_MULTIPLIER = 2.0f;
        static constexpr float ARM_SWING_AMOUNT = 20.0f;
        static constexpr float LEG_SWING_AMOUNT = 25.0f;
    }
    
    // Physics collision constants
    namespace Collision {
        static constexpr float GROUND_CHECK_DISTANCE = 0.2f;
        static constexpr float GROUND_CHECK_TOLERANCE = 0.20f;
        static constexpr float VELOCITY_Y_THRESHOLD = 2.0f;
        static constexpr float SHAPE_MARGIN = 0.1f;
        static constexpr float FRICTION = 0.8f; // Balanced friction - no sliding but smooth
        static constexpr float ROLLING_FRICTION = 0.3f; // Moderate rolling friction
        static constexpr float RESTITUTION = 0.0f;
    }
}
# 3D Game with Humanoid Character

A 3D game in C++ that demonstrates Object-Oriented Programming concepts. Uses raylib for graphics and Bullet Physics for realistic physics simulation.

## What it does

- Humanoid character that can walk and jump around the world
- Simple NPCs that wander and interact with the player
- Fruit shop with basic economic system for buying/selling
- Interactive objects (moving cubes, rotating platforms, scaling objects)
- Realistic physics for all objects and character movement
- Complete 3D world with lighting, textures, and collision detection

The game serves as both an entertaining experience and a comprehensive demonstration of advanced OOP techniques in a real-world application.

## OOP Concepts Implemented

### Inheritance ✓
- `GameObject` - base class with pure virtual functions
- `HumanoidCharacter`, `CubeObject`, `Floor` - derived classes  
- `MovingCubeObject`, `RotatingCubeObject` - multi-level inheritance
- **Location**: `include/objects/GameObject.h`, `include/entities/HumanoidCharacter.h`

### Virtual Functions ✓
- `draw()`, `update()`, `clone()`, `getBoundingBox()` - implemented differently in each class
- Pure virtual functions in base class enforce interface contracts
- Polymorphism through base class pointers `std::shared_ptr<GameObject>`
- **Location**: All classes override virtual methods from `GameObject`

### Templates ✓
- `GameObjectManager<T>` - generic container for managing different object types
- `Storage<ItemType>` - template-based inventory system with specializations
- `findObjectOfType<T>()` - type-safe object searching with dynamic casting
- **Location**: `include/utils/GameObjectManager.h`, `include/utils/Storage.h`

### Exception Handling ✓
```
GameException (base class)
├── GameInitException - initialization errors
├── ResourceException - file loading failures
├── PhysicsException - Bullet Physics errors
└── AIException - NPC behavior errors
```
- **Location**: `include/exceptions/GameExceptions.h`
- Try/catch blocks in main() handle different exception types

### Design Patterns ✓
- **Singleton**: `GameWorld`, `ShaderSystem`, `NPCManager` - single global instances
- **Observer**: NPCs observe player actions through `NPCObserver` interface
- **State**: NPC AI uses state machines (`IdleState`, `WalkingState`, `InteractingState`)
- **Location**: `include/ai/NPCStates.h`, `include/systems/ShaderSystem.h`

### Memory Management ✓
- `std::shared_ptr` for shared ownership of game objects
- `std::unique_ptr` for exclusive ownership of internal components
- RAII for automatic resource management
- Copy constructors and copy-and-swap idiom in `BodyPart` class
- **Location**: Throughout codebase, especially `include/BodyPart.h`

### Static Members ✓
- `GameWorld::instance` - singleton instance management
- `HumanoidCharacter::totalCharacters` - global character counting
- Static member functions for class-level operations
- **Location**: `include/GameWorld.h`, `include/entities/HumanoidCharacter.h`

### Dynamic Casting ✓
- `std::dynamic_pointer_cast<T>()` for safe downcasting in template methods
- Runtime type checking prevents invalid object access
- **Location**: `GameWorld::findObjectOfType<T>()` method

### STL Usage ✓
- `std::vector` for object collections
- `std::shared_ptr`/`std::unique_ptr` for memory management
- Standard algorithms for object processing
- **Location**: Throughout the codebase, no raw pointers used

## Technical Details

### Architecture
- Modular design with separated systems (rendering, physics, AI, input)
- Component-based entity system for flexible object composition
- Event-driven communication between game systems

### Graphics
- raylib 3D graphics engine with custom shader support
- Third-person camera with smooth following and collision avoidance
- Dynamic lighting and shadow rendering
- Texture mapping and material properties

### Physics
- Bullet Physics integration for realistic collision detection
- Character controller with proper ground detection
- Rigid body dynamics for all interactive objects
- Collision callbacks for game logic integration

### AI System
- State-based NPC behavior with finite state machines
- Observer pattern for NPCs to react to player actions
- Pathfinding and navigation through the game world

## How to Build and Run

```bash
./build_run.sh
```

Or manually:
```bash
mkdir -p build
cd build
cmake ..
make
./GameProject
```

## Controls

- **WASD** - move character
- **SPACE** - jump
- **Mouse** - look around
- **E** - interact with objects/NPCs
- **ESC** - exit game
- **Left Click** - alternative interaction

## Project Structure

```
src/           - source code implementation
include/       - header files
resources/     - textures and shaders
CMakeLists.txt - build configuration
```

### Code Organization
- Each class has separate .h and .cpp files
- Related classes grouped in directories (entities, objects, systems, exceptions)
- No `using namespace std` anywhere in the code
- Consistent naming conventions and const correctness

### Key Files
- `main.cpp` - entry point with exception handling
- `GameWorld.h/cpp` - main game logic and object management
- `HumanoidCharacter.h/cpp` - player character implementation
- `NPCStates.h/cpp` - AI state machine implementation
- `GameExceptions.h` - custom exception hierarchy
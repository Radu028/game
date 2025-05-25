# 3D Game Project with Humanoid Character Physics

This project implements a 3D game using the raylib library with Bullet physics simulation. The game features a humanoid character controller with unified physics and visual animation system, providing smooth character movement and collision detection.

## Project Structure

The code is organized into header (.h) and source (.cpp) files, with related classes grouped together. The main components are:

- `GameObject` base class and its derivatives
- **HumanoidCharacter** - Main character implementation with unified physics body and visual animation
- Exception handling system
- Game world management
- Various specialized cube objects

## Key Features

### Humanoid Character System
- **Unified Physics Body**: Single physics body for stable and reliable character movement
- **Visual Animation System**: Separate visual components for head, torso, arms, legs, and facial features
- **Professional Character Controller**: Smooth movement with walking, jumping, and idle animations
- **Individual Part Collision Detection**: Visual parts can detect collisions with environment objects
- **Realistic Ground Detection**: Foot-based ground contact detection for accurate jumping
- **State-Based Animation**: Different animations for idle, walking, and jumping states

### Code Organization
- All classes are separated into header (.h) and implementation (.cpp) files
- Related classes are grouped in directories (entities, objects, systems, exceptions)
- No `using namespace std` is used anywhere in the code

## Character Implementation

The project uses `HumanoidCharacter` as the main character implementation:

### HumanoidCharacter Features
- **Single Physics Body**: Uses one `btCapsuleShape` for stable character physics
- **Visual Animation System**: Separate visual parts for head, torso, arms, legs with smooth animations
- **Facial Features**: Eyes and mouth for character expression
- **State-Based Animation**: Idle breathing, walking cycles, and jump poses
- **Collision Prediction**: Can predict and prevent collisions before movement
- **Professional Controls**: WASD movement, Space for jumping, smooth camera following
- **Bullet Physics Integration**: Proper integration with Bullet physics for realistic movement

## Inheritance Hierarchy
- Base class: `GameObject`
- Derived classes: `CubeObject`, `Player`, and various specialized cube objects
- Second-level inheritance: `MovingCubeObject`, `RotatingCubeObject`, and `ScalingCubeObject` derive from `CubeObject`

### Virtual Functions
- Pure virtual functions in `GameObject`: `clone()`, `draw()`, `interact()`, `getBoundingBox()`
- Each derived class implements these functions differently:
  - `update()`: specialized movement in derived classes
  - `interact()`: custom interaction behavior
  - `draw()`: different rendering techniques

### Virtual Clone (Virtual Constructor)
- All classes implement the `clone()` method for proper copying
- Used for object duplication while preserving polymorphic behavior

### Base Class Constructor Calls
- Derived class constructors call their parent constructors
- Example: `CubeObject` constructor calls `GameObject` constructor
- Child classes like `MovingCubeObject` call `CubeObject` constructor

### Polymorphism Through Base Class Pointers
- `GameWorld` stores objects as `std::shared_ptr<GameObject>`
- Virtual functions are called through these base pointers
- Example: `world->update()` calls the appropriate derived implementation

### Copy Operations and Memory Management
- Classes with resources implement copy constructors and assignment operators
- The copy-and-swap idiom is used for safe assignment
- Smart pointers (`std::shared_ptr`) handle dynamic memory

### Dynamic Casting
- `std::dynamic_pointer_cast` is used in `findObjectOfType<T>()` method to safely downcast objects
- This allows retrieving specialized objects from the game world

### Exception Handling
- Custom exception hierarchy based on `std::exception`:
  - `GameException`: Base exception class
  - `GameInitException`: For initialization errors
  - `ResourceException`: For resource loading errors
  - `CollisionException`: For collision handling errors
- Try/catch blocks in main() handle different types of exceptions

### Static Members
- `GameWorld` uses the Singleton pattern with static instance
- Static methods like `getInstance()` control instance creation

### STL Usage
- `std::vector` for object storage
- `std::shared_ptr` for memory management
- Smart use of standard library algorithms

### Const Correctness
- Getter methods marked as `const`
- Parameters passed by const reference where appropriate
- Const methods for operations that don't modify object state

### High-Level Functions
- Functions focus on behavior rather than direct data access
- Minimal getters/setters, with emphasis on cohesive operations

## Build and Run

To build and run the project:

```bash
./build_run.sh
```

Or manually:

```bash
mkdir -p build
cd build
cmake ..
make
./game
```

## Controls
- WASD: Move the character
- SPACE: Jump
- ESC: Exit the game
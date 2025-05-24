# 3D Game Project with Professional Ragdoll Physics

This project implements a 3D game using the raylib library with advanced physics simulation. The game features a professional ragdoll character with individual physics bodies for each limb, providing realistic collision detection and movement.

## Project Structure

The code is organized into header (.h) and source (.cpp) files, with related classes grouped together. The main components are:

- `GameObject` base class and its derivatives
- **FullRagdoll** - Professional ragdoll with individual physics bodies
- SimpleRagdoll - Basic single-body character (legacy)
- Exception handling system
- Game world management
- Various specialized cube objects

## Key Features

### Advanced Ragdoll Physics
- **Individual Physics Bodies**: Separate collision detection for head, torso, arms, and legs
- **Collision Filtering**: Ragdoll parts don't interfere with each other while maintaining environment collision
- **Professional Ground Detection**: Individual leg raycasting for accurate ground contact
- **Constraint System**: Point-to-point joints keep body parts naturally connected
- **Realistic Mass Distribution**: Proper weight distribution across body parts

### Code Organization
- All classes are separated into header (.h) and implementation (.cpp) files
- Related classes are grouped in directories (entities, objects, systems, exceptions)
- No `using namespace std` is used anywhere in the code

## Ragdoll Implementations

This project features two different ragdoll implementations:

### FullRagdoll (Professional Implementation) - ✅ FULLY STABLE
- **Individual Physics Bodies**: Each body part (head, torso, arms, legs) has its own `btRigidBody`
- **Ultra-Strong Constraints**: Massive constraint forces prevent any limb separation or collapse
- **Advanced Collision System**: Uses collision filtering to prevent inter-limb collision while maintaining environment interaction
- **Precise Ground Detection**: Individual leg raycasting for accurate ground contact detection
- **Robust Stabilization**: Powerful stabilization forces keep character upright automatically
- **Realistic Physics**: Proper mass distribution and balanced gravity for natural movement
- **Reliable Jumping**: Enhanced jump system that works consistently
- **Problem Resolution**: Completely eliminates all stability issues, leg clipping, and movement problems

### SimpleRagdoll (Legacy Implementation)
- **Single Physics Body**: Uses one `btCapsuleShape` for the entire character
- **Basic Collision**: Simple body-environment collision detection
- **Limited Ground Detection**: Single-point ground checking
- **Animation-Based Movement**: Visual limb animation without individual physics
- **Known Issues**: Legs can clip through floor, arms pass through objects

### Migration from SimpleRagdoll to FullRagdoll
The project has been updated to use `FullRagdoll` by default for:
- More realistic physics simulation
- Better collision detection per body part
- Improved stability and movement
- Professional-grade character physics

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
- WASD: Move the player
- ESC: Exit the game
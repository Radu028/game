# 3D Game Engine

A comprehensive 3D game engine written in C++ showcasing advanced Object-Oriented Programming techniques, modern software architecture patterns, and real-time systems integration. Built with raylib graphics library and Bullet Physics engine.

## Project Overview

This project demonstrates a complete 3D interactive environment featuring:

- **Character-based gameplay** with realistic physics simulation and responsive controls
- **Intelligent NPC systems** utilizing advanced pathfinding algorithms and behavioral state machines
- **Dynamic economic simulation** with interactive trading mechanics and shopping behaviors
- **Real-time physics integration** supporting complex object interactions and collision detection
- **Modular architecture** enabling extensible gameplay systems and component composition
- **Professional-grade rendering** with custom shader pipelines and dynamic lighting systems

The codebase exemplifies industry-standard C++ development practices and advanced Object-Oriented Programming methodologies in a complex, real-world application.

## Technical Architecture and Implementation

This project implements comprehensive Object-Oriented Programming principles and modern C++ best practices:

### Software Architecture Principles

**Modular Code Organization**
- Complete separation of interface and implementation across all modules
- Header files (`.h`) contain class declarations and interfaces
- Source files (`.cpp`) contain implementation details and business logic
- Example: `include/objects/GameObject.h` paired with `src/objects/GameObject.cpp`

**Object-Oriented Design Patterns**

*Inheritance and Polymorphism*
- Robust inheritance hierarchy with `GameObject` as abstract base class
- Multiple inheritance levels: `CubeObject` → `MovingCubeObject` → specialized behaviors
- Virtual function dispatch enabling runtime polymorphism
- Pure virtual interfaces enforcing consistent contracts across derived classes

```
GameObject (abstract interface)
├── HumanoidCharacter (player and NPC entities)
├── CubeObject (interactive world objects)
│   ├── MovingCubeObject (dynamic movement behaviors)
│   ├── RotatingCubeObject (rotational animations)
│   └── ScalingCubeObject (scaling transformations)
├── Floor (environment geometry)
├── Sphere (alternative geometric primitives)
├── Building (complex architectural structures)
└── Fruit (economic system items)
```

*Memory Management and Resource Handling*
- Smart pointer architecture eliminating manual memory management
- `std::shared_ptr` for shared object ownership in game world
- `std::unique_ptr` for exclusive ownership of system components
- RAII principles ensuring automatic resource cleanup
- Implementation location: Throughout codebase, particularly in `GameWorld` object management

*Template-Based Generic Programming*
- `GameObjectManager<T>` providing type-safe object collection management
- `Storage<ItemType>` implementing generic storage systems with type specialization
- Template functions like `findObjectOfType<T>()` enabling safe runtime type queries
- Advanced template techniques including SFINAE and specialization

*Exception Safety and Error Handling*
- Comprehensive exception hierarchy extending from base `GameException` class
- Specialized exception types: `GameInitException`, `ResourceException`, `PhysicsException`, `AIException`
- Strong exception safety guarantees throughout critical systems
- Centralized error handling in application entry points

*Design Pattern Implementation*
- **Singleton Pattern**: Global system management (`GameWorld`, `ShaderSystem`, `NPCManager`)
- **State Pattern**: NPC behavioral state machines with clean state transitions
- **Observer Pattern**: Event-driven communication between game systems
- **Template Method Pattern**: Algorithm framework definition in base classes

*Advanced C++ Language Features*
- Copy-and-swap idiom implementation for exception-safe assignment operations
- Dynamic casting with `std::dynamic_pointer_cast` for safe runtime type inspection
- Comprehensive const correctness throughout class hierarchies
- STL integration with standard containers and algorithms
- Static member management for global state and utility functions
- Modern C++17 features and best practices compliance

## Pathfinding Systems

**A* Pathfinding Algorithm Implementation**
- Complete A* (A-star) pathfinding system for optimal route calculation
- Manhattan distance heuristic providing efficient path computation
- 3D navigation node structure supporting complex world geometries
- Integration location: `include/ai/NavMesh.h`, `src/ai/NavMesh.cpp`

**Navigation Mesh Technology**
- Automated navigation mesh generation from world geometry
- Dynamic walkable surface identification and traversal analysis
- Real-time obstacle detection with path recalculation capabilities
- Multi-level 3D environment support for complex architectural layouts

**Intelligent Agent Behavior**
- Finite state machine implementation for NPC behavioral modeling
- Dynamic path updating with real-time environment changes
- Collision avoidance systems for multi-agent environments
- Performance optimization through efficient caching and path sharing algorithms

## Core System Implementations

**Physics Integration**
- Complete Bullet Physics engine integration for realistic simulation
- Custom character controller with accurate ground detection and movement
- Collision callback systems enabling complex game logic interactions
- Rigid body dynamics supporting diverse interactive object behaviors

**Rendering and Graphics**
- raylib 3D graphics engine with custom shader pipeline development
- Advanced lighting systems including real-time shadow mapping
- Material property systems with texture mapping and surface attributes
- Third-person camera implementation with smooth following and collision avoidance

**Component-Based Architecture**
- Modular system design separating rendering, physics, and input handling
- Component composition enabling flexible object behavior definition
- Event-driven inter-system communication protocols
- Extensible architecture supporting rapid feature development

**Economic Simulation**
- Complete trading system with dynamic pricing algorithms
- Template-based storage management supporting multiple item categories
- Transaction logging and economic state persistence
- Supply and demand modeling affecting market pricing

**Advanced Resource Management**
- Efficient asset loading and caching systems for 3D models, textures, and audio
- Automatic resource cleanup and memory garbage collection
- Dynamic streaming with proximity-based loading and unloading
- Comprehensive error handling for missing or corrupted assets

**Multi-threading Architecture**
- Separate thread allocation for physics calculations
- Main thread optimization for rendering and input handling
- Thread-safe data structures with proper synchronization mechanisms
- Performance optimization through parallel system execution

## System Architecture Overview

**Application Structure**
```
Game Engine Core
├── Rendering System (raylib graphics integration)
├── Physics System (Bullet Physics simulation)
├── Pathfinding System (pathfinding and behavioral state machines)
├── Input System (user interaction handling)
├── Audio System (3D positional audio processing)
├── UI System (interface and menu management)
└── Economic System (trading and shopping behaviors)
```

**Codebase Statistics**
- **Object hierarchy**: 7+ main entity types with comprehensive inheritance structure
- **Exception handling**: 4 specialized exception types with robust error recovery
- **Template systems**: 12+ generic programming implementations
- **Total classes**: 50+ demonstrating complex software architecture
- **Memory management**: Zero raw pointer usage with complete smart pointer adoption

**Project Organization**
```
include/           (Interface definitions)
  ├── objects/        (Game entity class declarations)
  ├── entities/       (Character and NPC interfaces)
  ├── ai/            (Artificial intelligence systems)
  ├── physics/       (Physics integration interfaces)
  ├── utils/         (Utility classes and templates)
  └── exceptions/    (Error handling hierarchy)

src/              (Implementation code)
  ├── objects/       (Game entity implementations)
  ├── entities/      (Character behavior implementations)
  ├── ai/           (AI system logic)
  └── physics/      (Physics integration code)
```

## Build and Development Environment

**System Requirements**
- C++17 compatible compiler (GCC 7+, Clang 5+, or MSVC 2017+)
- CMake 3.16+ build system
- raylib graphics library
- Bullet Physics simulation library

**Build Process**

*Linux/macOS Environment*
```bash
git clone https://github.com/Radu028/game.git
cd game
mkdir build && cd build
cmake ..
make -j$(nproc)
./GameProject
```

*Windows Environment*
```cmd
git clone https://github.com/Radu028/game.git
cd game
mkdir build && cd build
cmake .. -G "Visual Studio 16 2019"
cmake --build . --config Release
Release\GameProject.exe
```

**Dependency Management**

For a fully self-contained setup run:

```bash
./scripts/setup_dependencies.sh
```

This script downloads and builds raylib and Bullet locally into the
`external/` directory so no system-wide packages are required.

## Application Usage

**Control Scheme**
- Movement: WASD keys for directional character control
- Camera: Mouse control for 3D perspective adjustment
- Interface: F1/F2 for debug modes, Esc for menu systems

**Interactive Features**
- Explore comprehensive 3D environment with physics-based interactions
- Engage with intelligent NPCs demonstrating advanced pathfinding behaviors
- Experience economic simulation through trading system mechanics
- Interact with dynamic objects showcasing various animation and physics systems
- Observe polymorphic behavior patterns across different entity types

## Technical Achievements

**Software Engineering Excellence**
- Comprehensive Object-Oriented Programming implementation with advanced design patterns
- Modern C++ best practices including complete smart pointer adoption and RAII principles
- Robust error handling with custom exception hierarchies and strong safety guarantees
- Professional code organization with clear separation of concerns and modular architecture

**Advanced Algorithm Implementation**
- A* pathfinding algorithm with optimized heuristics for real-time navigation
- Finite state machine systems for complex behavioral modeling
- Template-based generic programming with type specialization techniques
- Multi-threaded architecture with thread-safe data structures and synchronization

**Real-World Application Development**
- Complete 3D graphics integration with custom shader development
- Physics engine integration for realistic simulation and collision detection
- Component-based architecture enabling scalable and maintainable systems
- Economic simulation with dynamic pricing and storage management algorithms

This project demonstrates professional-level software development capabilities combining theoretical computer science knowledge with practical implementation skills in C++. The codebase showcases industry-standard development practices and advanced programming techniques suitable for complex real-time applications.

---

*Advanced C++ project demonstrating comprehensive Object-Oriented Programming principles and modern software architecture patterns*


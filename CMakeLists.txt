cmake_minimum_required(VERSION 3.10)
project(GameProject)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)

include_directories(include)

add_executable(GameExecutable 
    main.cpp
    src/Player.cpp
    src/inputFunctions.cpp
)

target_link_libraries(GameExecutable raylib)
cmake_minimum_required(VERSION 3.10)
project(GameProject)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RAYLIB_DIR "${CMAKE_SOURCE_DIR}/external/raylib")
set(BULLET_DIR "${CMAKE_SOURCE_DIR}/external/bullet")

if(EXISTS "${RAYLIB_DIR}")
    set(raylib_DIR "${RAYLIB_DIR}")
endif()
if(EXISTS "${BULLET_DIR}")
    set(BULLET_ROOT_DIR "${BULLET_DIR}")
endif()

find_package(raylib REQUIRED)
find_package(Bullet REQUIRED)

include_directories(include)

file(GLOB SRC_ROOT_FILES "src/*.cpp")
file(GLOB_RECURSE SRC_SUBDIR_FILES "src/*/*.cpp")

set(ALL_GAME_SOURCES ${SRC_ROOT_FILES} ${SRC_SUBDIR_FILES})

add_executable(${PROJECT_NAME} ${ALL_GAME_SOURCES})
target_link_libraries(${PROJECT_NAME} raylib ${BULLET_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${BULLET_INCLUDE_DIRS})
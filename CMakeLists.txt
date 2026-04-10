cmake_minimum_required(VERSION 3.10)
project(ParticleSimulation LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WIN32)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})

# Include dirs
include_directories(
    ${PROJECT_ROOT}/libs/glad/include
    ${PROJECT_ROOT}/libs/glfw/include
    ${PROJECT_ROOT}/libs/glm
    ${PROJECT_ROOT}/src
)

# Sources
set(SOURCES
    ${PROJECT_ROOT}/src/main.cpp
    ${PROJECT_ROOT}/src/physics/particle_system.cpp
    ${PROJECT_ROOT}/src/render/renderer.cpp
    ${PROJECT_ROOT}/libs/glad/src/glad.c
)

add_executable(ParticleSim ${SOURCES})

# Prefer bundled GLFW if present
if(EXISTS "${PROJECT_ROOT}/libs/glfw/CMakeLists.txt")
    add_subdirectory(${PROJECT_ROOT}/libs/glfw EXCLUDE_FROM_ALL)
    # GLFW target name depends on GLFW version; common targets: glfw, glfw3
    if(TARGET glfw)
        target_link_libraries(ParticleSim PRIVATE glfw)
    elseif(TARGET glfw3)
        target_link_libraries(ParticleSim PRIVATE glfw3)
    else()
        message(FATAL_ERROR "GLFW subdirectory added but no glfw target found.")
    endif()
else()
    find_package(glfw3 REQUIRED)
    target_link_libraries(ParticleSim PRIVATE glfw)
endif()

# Link OpenGL
find_package(OpenGL REQUIRED)
if(TARGET OpenGL::GL)
    target_link_libraries(ParticleSim PRIVATE OpenGL::GL)
else()
    if(WIN32)
        target_link_libraries(ParticleSim PRIVATE opengl32)
    endif()
endif()

if(WIN32)
    target_link_libraries(ParticleSim PRIVATE gdi32)
endif()

# Per-target include dirs (modern)
target_include_directories(ParticleSim PRIVATE
    ${PROJECT_ROOT}/libs/glad/include
    ${PROJECT_ROOT}/libs/glfw/include
    /usr/include
    ${PROJECT_ROOT}/src
)

set_target_properties(ParticleSim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
)

# Quick check for GLM header
if(NOT EXISTS "/usr/include/glm/glm.hpp")
    message(FATAL_ERROR "Arquivo glm.hpp não encontrado! Instale com: sudo apt-get install libglm-dev")
endif()
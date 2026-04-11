#ifndef MOUSE_HPP
#define MOUSE_HPP

#include <vector>
#include <glm/glm.hpp>

#include "physics/particle_system.hpp"

struct MouseState {
        double x, y;               // posição atual em pixels
        bool   holding = false;
        std::vector<int> heldParticles;
        std::vector<Vec2> offsets; // para manter a posição relativa ao clicar
} mouse;

// Dimensões da janela e do mundo simulado
const unsigned int SCR_WIDTH    = 1280;
const unsigned int SCR_HEIGHT   = 720;
const float        WORLD_WIDTH  = 20.0f;
const float        WORLD_HEIGHT = 15.0f;

// =============================================================================
// Protótipos de callbacks GLFW
// =============================================================================

glm::vec2 screenToWorld(double px, double py) {
        float wx = ((float)px / SCR_WIDTH  - 0.5f) *  WORLD_WIDTH;
        float wy = ((float)py / SCR_HEIGHT - 0.5f) * -WORLD_HEIGHT;
        return glm::vec2(wx, wy);
}

void processInput(GLFWwindow* window) {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
        // Para redimensionamento dinâmico (opcional)
        glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
        if (button != GLFW_MOUSE_BUTTON_LEFT) return;

        ParticleSystem* system = static_cast<ParticleSystem*>(glfwGetWindowUserPointer(window));

        if (action == GLFW_PRESS) {
                glfwGetCursorPos(window, &mouse.x, &mouse.y);
                glm::vec2 wp = screenToWorld(mouse.x, mouse.y);
                Vec2 worldPos(wp.x, wp.y);

                float pickRadius = 1.0f;
                std::vector<int>  indices;
                std::vector<Vec2> offsets;

                const auto& particles = system->getParticles();
                for (int i = 0; i < (int)particles.size(); ++i) {
                        Vec2  pos = particles[i].getPosition();
                        float dx  = pos.getX() - worldPos.getX();
                        float dy  = pos.getY() - worldPos.getY();
                        if (std::sqrt(dx*dx + dy*dy) < pickRadius) {
                                indices.push_back(i);
                                offsets.push_back(Vec2(dx, dy));
                        }
                }

                if (!indices.empty())
                        system->startHold(indices, offsets, worldPos);

        } else if (action == GLFW_RELEASE) {
                system->releaseHold();
        }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
        mouse.x = xpos;
        mouse.y = ypos;
        glm::vec2 wp = screenToWorld(xpos, ypos);
        ParticleSystem* system = static_cast<ParticleSystem*>(glfwGetWindowUserPointer(window));
        system->updateMouseTarget(Vec2(wp.x, wp.y));
}


#endif
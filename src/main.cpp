#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cstdlib>

#include "physics/particle_system.hpp"
#include "render/renderer.hpp"
#include "utils/benchmark.hpp"

// =============================================================================
// Constantes de simulação
// Ajuste aqui para mudar o comportamento geral sem tocar na física ou no render.
// =============================================================================

#define MAX_PARTICLES    10000
#define DAMPING          0.99f   // amortecimento global do integrador Verlet
#define TIME_STEP        0.008f  // passo de tempo fixo (~60Hz)
#define COLISION_DAMPING 0.85f   // restituição das colisões (0 = inelástico, 1 = elástico)
#define BLOCK_SIZE       64      // tamanho do bloco para cache blocking no solver

struct MouseState {
        double x, y;               // posição atual em pixels
        bool   holding = false;
        int    heldParticle = -1;  // índice da partícula segurada (-1 = nenhuma)
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
                glm::vec2 worldPos = screenToWorld(mouse.x, mouse.y);

                float pickRadius   = 2.5f;
                float minDist      = pickRadius;
                mouse.heldParticle = -1;

                const auto& particles = system->getParticles();
                for (int i = 0; i < (int)particles.size(); ++i) {
                        float dx = particles[i].getPosition().getX() - worldPos.x;
                        float dy = particles[i].getPosition().getY() - worldPos.y;
                        float d  = std::sqrt(dx*dx + dy*dy);
                        if (d < minDist) {
                                minDist = d;
                                mouse.heldParticle = i;
                        }
                }

                if (mouse.heldParticle != -1)
                        mouse.holding = true;

        } else if (action == GLFW_RELEASE) {
                mouse.holding      = false;
                mouse.heldParticle = -1;
        }
}

// =============================================================================
// main
// =============================================================================

int main(int argc, char** argv) {
        if (argc != 2) {
                std::cout << "Uso: " << argv[0] << " <num_particulas>" << std::endl;
                return 0;
        }

        // Limita o número de partículas ao máximo definido
        size_t numParticles = std::min(
                static_cast<size_t>(MAX_PARTICLES),
                static_cast<size_t>(std::atoi(argv[1]))
        );

        // -------------------------------------------------------------------------
        // Inicialização GLFW
        // -------------------------------------------------------------------------
        if (!glfwInit()) {
                std::cerr << "Falha ao inicializar GLFW" << std::endl;
                return -1;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

        GLFWwindow* window = glfwCreateWindow(
                SCR_WIDTH, SCR_HEIGHT, "Simulação de Partículas", NULL, NULL
        );
        if (!window) {
                std::cerr << "Falha ao criar janela GLFW" << std::endl;
                glfwTerminate();
                return -1;
        }

        glfwMakeContextCurrent(window);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
                std::cerr << "Falha ao inicializar GLAD" << std::endl;
                return -1;
        }

        // -------------------------------------------------------------------------
        // Inicialização do renderer (compila shaders, aloca VAO/VBO)
        // -------------------------------------------------------------------------
        Renderer renderer(SCR_WIDTH, SCR_HEIGHT, WORLD_WIDTH, WORLD_HEIGHT);
        renderer.init();

        // -------------------------------------------------------------------------
        // Inicialização do sistema de partículas
        // O mundo vai de -W/2 a +W/2 em X e -H/2 a +H/2 em Y (origem no centro)
        // -------------------------------------------------------------------------
        ParticleSystem system(
                -WORLD_WIDTH  / 2.0f,  WORLD_WIDTH  / 2.0f,
                -WORLD_HEIGHT / 2.0f,  WORLD_HEIGHT / 2.0f,
                BLOCK_SIZE,
                TIME_STEP,
                DAMPING,
                COLISION_DAMPING
        );

        // Popula com partículas em posições e velocidades aleatórias
        for (size_t i = 0; i < numParticles; ++i) {
                float x = (rand() % 100) * WORLD_WIDTH  / 100.0f - WORLD_WIDTH  / 2.0f;
                float y = (rand() % 100) * WORLD_HEIGHT / 100.0f - WORLD_HEIGHT / 2.0f;

                Particle p(
                        Vec2(x, y),
                        Vec2(rand() % 10 - 5, rand() % 10 - 5),
                        1.0f + (rand() % 100) / 50.0f,
                        0.1f,  //raio
                        glm::vec3(
                                (rand() % 100) / 100.0f,
                                (rand() % 100) / 100.0f,
                                (rand() % 100) / 100.0f
                        )
                );
                system.addParticle(p);
        }

        // Agora que system existe, pode configurar callbacks
        glfwSetWindowUserPointer(window, &system);
        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

        // -------------------------------------------------------------------------
        // Benchmarks de inicialização (roda uma vez antes do loop principal)
        // -------------------------------------------------------------------------
        std::cout << "\n=== BENCHMARKS ===" << std::endl;

        benchmarkTime("Update do sistema", [&]() {
                system.update();
        });

        benchmarkTime("Preparação de posições/cores", [&]() {
                for (const auto& p : system.getParticles()) {
                        (void)p.getPosition();
                }
        });

        benchmarkTime("Renderização", [&]() {
                glClear(GL_COLOR_BUFFER_BIT);
                renderer.draw(system.getParticles(), system.getTimeStep());
        });

        std::cout << "=================\n" << std::endl;

        // -------------------------------------------------------------------------
        // Loop principal
        // -------------------------------------------------------------------------
        double lastTime    = glfwGetTime();
        double startTime   = lastTime;
        double fpsTimer    = 0.0;
        int    frameCount  = 0;
        int    totalFrames = 0;

        while (!glfwWindowShouldClose(window)) {
                processInput(window);

                glClear(GL_COLOR_BUFFER_BIT);

                if (mouse.holding && mouse.heldParticle != -1) {
                        glm::vec2 worldPos = screenToWorld(mouse.x, mouse.y);
                        Vec2 target(worldPos.x, worldPos.y);

                        // Atualiza posição e prev para a mesma posição — velocidade zero
                        // O Verlet vai manter ela parada enquanto você segura
                        system.setParticlePosition(mouse.heldParticle, target);
                }

                // Passo de física
                system.update();

                // Passo de renderização
                renderer.draw(system.getParticles(), system.getTimeStep());

                glfwSwapBuffers(window);
                glfwPollEvents();

                // Contador de FPS: imprime no terminal a cada 1 segundo
                double now   = glfwGetTime();
                double delta = now - lastTime;
                lastTime     = now;
                fpsTimer    += delta;
                frameCount++;
                totalFrames++;

                if (fpsTimer >= 1.0) {
                        std::cout << "\rFPS: " << frameCount << "   " << std::flush;
                        frameCount = 0;
                        fpsTimer   = 0.0;
                }
        }

        // Estatísticas finais
        double totalTime = glfwGetTime() - startTime;
        std::cout << "\nTotal frames renderizados: " << totalFrames           << std::endl;
        std::cout << "Tempo total de execução:   " << totalTime               << " segundos" << std::endl;
        std::cout << "FPS médio:                 " << totalFrames / totalTime << std::endl;

        // -------------------------------------------------------------------------
        // Limpeza
        // -------------------------------------------------------------------------
        renderer.cleanup();
        glfwTerminate();
        return 0;
}
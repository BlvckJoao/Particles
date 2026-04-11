#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cstdlib>
#include <random>

#include "physics/particle_system.hpp"
#include "render/renderer.hpp"
#include "utils/benchmark.hpp"
#include "utils/mouse.hpp"

// =============================================================================
// Constantes de simulação
// Ajuste aqui para mudar o comportamento geral sem tocar na física ou no render.
// =============================================================================

#define MAX_PARTICLES    10000
#define DAMPING          0.99f   // amortecimento global do integrador Verlet
#define TIME_STEP        0.008f  // passo de tempo fixo (~60Hz)
#define COLISION_DAMPING 0.85f   // restituição das colisões (0 = inelástico, 1 = elástico)
#define BLOCK_SIZE       64      // tamanho do bloco para cache blocking no solver

#define RAY_LENGTH       0.1f    // comprimento do "raio" para detecção de clique em partículas

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
        Benchmark benchmark;
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

        // Gerador de números aleatórios de alta qualidade
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist_pos_x(-WORLD_WIDTH  / 2.0f, WORLD_WIDTH  / 2.0f);
        std::uniform_real_distribution<float> dist_pos_y(-WORLD_HEIGHT / 2.0f, WORLD_HEIGHT / 2.0f);
        std::uniform_real_distribution<float> dist_vel(-5.0f, 5.0f);
        std::uniform_real_distribution<float> dist_mass(1.0f, 3.0f);
        std::uniform_real_distribution<float> dist_color(0.0f, 1.0f);

        // Popula com partículas em posições e velocidades aleatórias
        for (size_t i = 0; i < numParticles; ++i) {
                float x = dist_pos_x(gen);
                float y = dist_pos_y(gen);

                Particle p(
                        Vec2(x, y),
                        Vec2(dist_vel(gen), dist_vel(gen)),
                        dist_mass(gen),
                        RAY_LENGTH,  // raio
                        glm::vec3(
                                dist_color(gen),
                                dist_color(gen),
                                dist_color(gen)
                        )
                );
                system.addParticle(p);
        }

        // Agora que system existe, pode configurar callbacks
        glfwSetWindowUserPointer(window, &system);
        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
        glfwSetCursorPosCallback(window, cursor_position_callback);

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
        double startTime   = glfwGetTime();
        int    totalFrames = 0;

        while (!glfwWindowShouldClose(window)) {
                processInput(window);

                glClear(GL_COLOR_BUFFER_BIT);

                // Passo de física
                system.update();

                // Passo de renderização
                renderer.draw(system.getParticles(), system.getTimeStep());

                glfwSwapBuffers(window);
                glfwPollEvents();

                // Atualiza estatísticas de FPS e CPU
                benchmark.tick();
                benchmark.report("Simulação");

                totalFrames++;
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
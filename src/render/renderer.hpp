#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <vector>
#include <thread>
#include <functional>

#include "../physics/particle.hpp"
#include "../utils/thread_pool.hpp"

// =============================================================================
// Renderer
//
// Responsável por toda a comunicação com a GPU:
//   - compilação e linkagem dos shaders GLSL
//   - alocação e atualização dos buffers OpenGL (VAO/VBO)
//   - mapeamento velocidade -> cor (azul lento / vermelho rápido)
//   - desenho de cada partícula como GL_POINT circular com borda suavizada
//
// Uso típico:
//   Renderer r(SCR_WIDTH, SCR_HEIGHT, WORLD_WIDTH, WORLD_HEIGHT);
//   r.init();                         // compila shaders, aloca buffers
//   r.draw(particles, dt);            // chama a cada frame
//   r.cleanup();                      // libera recursos OpenGL
// =============================================================================

class Renderer {
public:
        // Dimensões da janela em pixels
        unsigned int scrWidth;
        unsigned int scrHeight;

        // Dimensões do mundo em unidades de simulação
        float worldWidth;
        float worldHeight;

        // Fator de escala do raio da partícula para tamanho em pixels
        // Aumentar = partículas maiores na tela
        float sizeScale = 40.0f;

        Renderer(unsigned int scrW, unsigned int scrH,
                 float worldW, float worldH);

        // Compila shaders, cria VAO/VBO e calcula a matriz de projeção ortográfica.
        // Deve ser chamado uma única vez, após glfwMakeContextCurrent.
        void init();

        // Prepara os dados das instâncias em paralelo, atualiza o VBO e desenha
        // todas as partículas com uma única chamada instanciada.
        // dt: timestep da simulação, usado para estimar velocidade via Verlet.
        void draw(const std::vector<Particle>& particles, float dt);

        // Deleta VAO, VBO e o shader program.
        void cleanup();

private:
        unsigned int shaderProgram;
        unsigned int VAO, instanceVBO;
        glm::mat4 projection;
        ThreadPool threadPool;
        size_t workerCount;

        struct ParticleInstance {
                glm::vec2 position;
                float radius;
                glm::vec3 color;
        };

        std::vector<float> speeds;
        std::vector<ParticleInstance> instances;

        void parallelFor(size_t count, const std::function<void(size_t, size_t)>& work);

        // Compila um shader GLSL e verifica erros. Retorna o ID do shader.
        unsigned int compileShader(unsigned int type, const char* source);
};

// Callback de redimensionamento de janela — registrar via glfwSetFramebufferSizeCallback
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

#endif

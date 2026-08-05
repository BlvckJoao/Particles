#include "renderer.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <algorithm>
#include <cstddef>
#include <future>

// =============================================================================
// Shaders GLSL
//
// Vertex shader:
//   Recebe a posição 2D da partícula, aplica a projeção ortográfica e
//   define o tamanho do ponto em pixels via gl_PointSize.
//
// Fragment shader:
//   Usa gl_PointCoord para calcular a distância ao centro do ponto e
//   descarta fragmentos fora do raio, criando um círculo. A borda é
//   suavizada com smoothstep para evitar aliasing.
// =============================================================================

static const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 instancePosition;
        layout (location = 1) in float instanceRadius;
        layout (location = 2) in vec3 instanceColor;
        uniform mat4 projection;
        uniform float sizeScale;
        out vec3 ParticleColor;
        void main() {
                gl_Position = projection * vec4(instancePosition, 0.0, 1.0);
                gl_PointSize = instanceRadius * sizeScale;
                ParticleColor = instanceColor;
        }
)";

static const char* fragmentShaderSource = R"(
        #version 330 core
        in vec3 ParticleColor;
        out vec4 FragColor;
        void main() {
                vec2 coord = gl_PointCoord - vec2(0.5);
                float dist = length(coord);

                // Descarta fragmentos fora do círculo
                if (dist > 0.5) discard;

                // Suaviza a borda entre 0.4 e 0.5 do raio
                float alpha = 1.0 - smoothstep(0.4, 0.5, dist);
                FragColor = vec4(ParticleColor, alpha);
        }
)";

// =============================================================================

Renderer::Renderer(unsigned int scrW, unsigned int scrH,
                   float worldW, float worldH)
        : scrWidth(scrW), scrHeight(scrH),
          worldWidth(worldW), worldHeight(worldH),
          shaderProgram(0), VAO(0), instanceVBO(0),
          threadPool(std::max(1u, std::thread::hardware_concurrency())),
          workerCount(std::max(1u, std::thread::hardware_concurrency())) {}

void Renderer::parallelFor(
        size_t count, const std::function<void(size_t, size_t)>& work) {
        if (count == 0) return;

        const size_t taskCount = std::min(workerCount, count);
        const size_t chunkSize = (count + taskCount - 1) / taskCount;
        std::vector<std::future<void>> futures;
        futures.reserve(taskCount);

        for (size_t begin = 0; begin < count; begin += chunkSize) {
                const size_t end = std::min(begin + chunkSize, count);
                futures.emplace_back(threadPool.submit([&, begin, end] {
                        work(begin, end);
                }));
        }

        for (auto& future : futures) future.get();
}

void Renderer::init() {
        // Habilita tamanho de ponto programável e blending para o alpha dos círculos
        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

        // Compila e linka os shaders
        unsigned int vs = compileShader(GL_VERTEX_SHADER,   vertexShaderSource);
        unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vs);
        glAttachShader(shaderProgram, fs);
        glLinkProgram(shaderProgram);

        int success;
        char infoLog[512];
        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
        if (!success) {
                glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
                std::cerr << "ERRO::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }

        // Shaders já linkados podem ser deletados da memória da GPU
        glDeleteShader(vs);
        glDeleteShader(fs);

        // Projeção ortográfica: mapeia coordenadas do mundo para NDC
        // O mundo vai de -W/2 a +W/2 em X e -H/2 a +H/2 em Y
        projection = glm::ortho(
                -worldWidth  / 2.0f,  worldWidth  / 2.0f,
                -worldHeight / 2.0f,  worldHeight / 2.0f,
                -1.0f, 1.0f
        );

        // One interleaved instance buffer stores position, radius and color.
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &instanceVBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance),
                              reinterpret_cast<void*>(offsetof(ParticleInstance, position)));
        glEnableVertexAttribArray(0);
        glVertexAttribDivisor(0, 1);

        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance),
                              reinterpret_cast<void*>(offsetof(ParticleInstance, radius)));
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);

        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance),
                              reinterpret_cast<void*>(offsetof(ParticleInstance, color)));
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
}

void Renderer::draw(const std::vector<Particle>& particles, float dt) {
        // -------------------------------------------------------------------------
        // Coleta posições, raios e estima velocidade escalar via integrador de Verlet:
        //   v ≈ (pos_atual - pos_anterior) / dt
        // Isso evita armazenar velocidade explicitamente na partícula.
        // -------------------------------------------------------------------------
        speeds.resize(particles.size());
        instances.resize(particles.size());

        parallelFor(particles.size(), [&particles, dt, this](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i) {
                        const Vec2 dv = (particles[i].getPosition() -
                                         particles[i].getPrevPosition()) / dt;
                        speeds[i] = dv.length();
                }
        });

        // -------------------------------------------------------------------------
        // Mapeamento de velocidade para cor: azul (parado) → vermelho (rápido)
        //   t = clamp(speed / speedScale, 0, 1)
        //   cor = (t, 0, 1-t)
        // speedScale = max(velocidade_maxima_atual, 5.0) para estabilizar o mapeamento
        // -------------------------------------------------------------------------
        float maxSpeed = 0.0f;
        for (float s : speeds) maxSpeed = std::max(maxSpeed, s);
        const float speedScale = std::max(maxSpeed, 5.0f);

        parallelFor(particles.size(),
                    [&particles, speedScale, this](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
                const Particle& particle = particles[i];
                const Vec2 position = particle.getPosition();
                float t = glm::clamp(speeds[i] / speedScale, 0.0f, 1.0f);

                glm::vec3 color;
                if (t < 0.33f) {
                        float u = t / 0.33f;
                        color = glm::vec3(0.0f, u, 1.0f);
                } else if (t < 0.66f) {
                        float u = (t - 0.33f) / 0.33f;
                        color = glm::vec3(u, 1.0f, 1.0f - u);
                } else {
                        float u = (t - 0.66f) / 0.34f;
                        color = glm::vec3(1.0f, 1.0f - u, 0.0f);
                }

                instances[i] = {
                        glm::vec2(position.getX(), position.getY()),
                        particle.getRadius(),
                        color
                };
        }
        });

        // OpenGL calls stay on the context-owning thread.
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER,
                     instances.size() * sizeof(ParticleInstance),
                     instances.data(), GL_STREAM_DRAW);

        glUseProgram(shaderProgram);

        // Uniforms que não mudam por partícula
        GLint projLoc      = glGetUniformLocation(shaderProgram, "projection");
        GLint sizeScaleLoc = glGetUniformLocation(shaderProgram, "sizeScale");

        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        glUniform1f(sizeScaleLoc, sizeScale);

        // -------------------------------------------------------------------------
        // A single point is emitted for each instance. Position, radius and
        // color advance once per instance because their divisors are one.
        // -------------------------------------------------------------------------
        glBindVertexArray(VAO);
        glDrawArraysInstanced(GL_POINTS, 0, 1,
                              static_cast<GLsizei>(instances.size()));
}

void Renderer::cleanup() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &instanceVBO);
        glDeleteProgram(shaderProgram);
}

unsigned int Renderer::compileShader(unsigned int type, const char* source) {
        unsigned int shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, NULL);
        glCompileShader(shader);

        int success;
        char infoLog[512];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
                glGetShaderInfoLog(shader, 512, NULL, infoLog);
                const char* typeName = (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";
                std::cerr << "ERRO::SHADER::" << typeName << "::COMPILATION_FAILED\n"
                          << infoLog << std::endl;
        }

        return shader;
}

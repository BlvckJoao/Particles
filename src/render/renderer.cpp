#include "renderer.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <algorithm>

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
        layout (location = 0) in vec2 aPos;
        uniform mat4 projection;
        uniform vec3 color;
        uniform float pointSize;
        out vec3 ParticleColor;
        void main() {
                gl_Position = projection * vec4(aPos, 0.0, 1.0);
                gl_PointSize = pointSize;
                ParticleColor = color;
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
          shaderProgram(0), VAO(0), VBO(0) {}

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

        // VAO armazena o layout do buffer; VBO armazena as posições das partículas
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // Atributo 0: vec2 de posição (x, y), sem normalização, stride de 2 floats
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
}

void Renderer::draw(const std::vector<Particle>& particles, float dt) {
        // -------------------------------------------------------------------------
        // Coleta posições, raios e estima velocidade escalar via integrador de Verlet:
        //   v ≈ (pos_atual - pos_anterior) / dt
        // Isso evita armazenar velocidade explicitamente na partícula.
        // -------------------------------------------------------------------------
        std::vector<glm::vec2> positions;
        std::vector<float>     radii;
        std::vector<float>     speeds;

        positions.reserve(particles.size());
        radii.reserve(particles.size());
        speeds.reserve(particles.size());

        for (const auto& p : particles) {
                Vec2 pos  = p.getPosition();
                Vec2 prev = p.getPrevPosition();

                positions.push_back(glm::vec2(pos.getX(), pos.getY()));
                radii.push_back(p.getRadius());

                Vec2  dv = (pos - prev) / dt;
                speeds.push_back(dv.length());
        }

        // -------------------------------------------------------------------------
        // Mapeamento de velocidade para cor: azul (parado) → vermelho (rápido)
        //   t = clamp(speed / speedScale, 0, 1)
        //   cor = (t, 0, 1-t)
        // speedScale = max(velocidade_maxima_atual, 5.0) para estabilizar o mapeamento
        // -------------------------------------------------------------------------
        float maxSpeed = 0.0f;
        for (float s : speeds) maxSpeed = std::max(maxSpeed, s);
        const float speedScale = std::max(maxSpeed, 5.0f);

        std::vector<glm::vec3> colors;
        colors.reserve(speeds.size());
        for (float s : speeds) {
                float t = glm::clamp(s / speedScale, 0.0f, 1.0f);
                colors.push_back(glm::vec3(t, 0.0f, 1.0f - t));
        }

        // Envia todas as posições para a GPU de uma vez (GL_DYNAMIC_DRAW pois muda todo frame)
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     positions.size() * sizeof(glm::vec2),
                     positions.data(),
                     GL_DYNAMIC_DRAW);

        glUseProgram(shaderProgram);

        // Uniforms que não mudam por partícula
        GLint projLoc      = glGetUniformLocation(shaderProgram, "projection");
        GLint colorLoc     = glGetUniformLocation(shaderProgram, "color");
        GLint pointSizeLoc = glGetUniformLocation(shaderProgram, "pointSize");

        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // -------------------------------------------------------------------------
        // Desenho individual por partícula: cada uma pode ter cor e tamanho diferentes.
        // glDrawArrays(GL_POINTS, i, 1) desenha exatamente o ponto i do VBO.
        // Overhead de uniform por partícula é aceitável para N <= ~5000;
        // acima disso considerar instanced rendering.
        // -------------------------------------------------------------------------
        glBindVertexArray(VAO);
        for (size_t i = 0; i < positions.size(); ++i) {
                glUniform3fv(colorLoc, 1, glm::value_ptr(colors[i]));
                glUniform1f(pointSizeLoc, radii[i] * sizeScale);
                glDrawArrays(GL_POINTS, static_cast<int>(i), 1);
        }
}

void Renderer::cleanup() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
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

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
}
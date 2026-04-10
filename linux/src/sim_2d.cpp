#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>

#include "particle_phsx.hpp"
#include "benchmark.hpp"

#define MAX_PARTICLES 2000
#define DAMPING 0.99f
#define TIME_STEP 0.016f
#define COLISION_DAMPING 0.85f
#define BLOCK_SIZE 64

// Funções de callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

// Configurações
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const float WORLD_WIDTH = 20.0f;
const float WORLD_HEIGHT = 15.0f;

// Shaders
const char* vertexShaderSource = R"(
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

const char* fragmentShaderSource = R"(
    #version 330 core
    in vec3 ParticleColor;
    out vec4 FragColor;
    void main() {
        // Cria efeito de círculo suave
        vec2 coord = gl_PointCoord - vec2(0.5);
        float dist = length(coord);
        if (dist > 0.5) discard;
        
        // Suaviza as bordas
        float alpha = 1.0 - smoothstep(0.4, 0.5, dist);
        FragColor = vec4(ParticleColor, alpha);
    }
)";

// Variáveis globais
unsigned int shaderProgram;
unsigned int VAO, VBO;
glm::mat4 projection;

int main(int argc, char** argv) {
    if(argc != 2) {
        std::cout << "Uso: " << argv[0] << std::endl;
        return 0;
    }
    size_t numParticles = std::min(static_cast<size_t>(MAX_PARTICLES), static_cast<size_t>(std::atoi(argv[1])));
    // Inicialização GLFW
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

    // Criação da janela
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Simulação de Partículas", NULL, NULL);
    if (!window) {
        std::cerr << "Falha ao criar janela GLFW" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Carrega GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Falha ao inicializar GLAD" << std::endl;
        return -1;
    }

    // Configurações OpenGL
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Compilação de shaders
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    // Verifica erros do vertex shader
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "ERRO::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    
    // Verifica erros do fragment shader
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "ERRO::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    
    // Linka os shaders
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    
    // Verifica erros de linking
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERRO::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Configuração da matriz de projeção
    projection = glm::ortho(
        -WORLD_WIDTH/2.0f, WORLD_WIDTH/2.0f,
        -WORLD_HEIGHT/2.0f, WORLD_HEIGHT/2.0f,
        -1.0f, 1.0f
    );

    // Configuração dos buffers
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    // Configura o layout do buffer
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Cria o sistema de partículas
    // Note: ParticleSystem constructor is (left,right,top,bottom,timeStep,damp)
    ParticleSystem system(
        -WORLD_WIDTH/2.0f, WORLD_WIDTH/2.0f,
        -WORLD_HEIGHT/2.0f, WORLD_HEIGHT/2.0f,
        BLOCK_SIZE, // blockSize
        TIME_STEP,  // timeStep
        DAMPING,    // damping
        COLISION_DAMPING // collision_damp (0.0 - no reaction, 1.0 - full reaction)
    );

    // Adiciona partículas iniciais
    for (int i = 0; i < numParticles; i++) {
        float x = (rand() % 100) * WORLD_WIDTH / 100.0f - WORLD_WIDTH/2;
        float y = (rand() % 100) * WORLD_HEIGHT / 100.0f - WORLD_HEIGHT/2;
        
        Particle p(
            Vec2(x, y), // Posição
            Vec2((rand() % 10 - 5), (rand() % 10 - 5)),   // Velocidade
            1.0f + (rand() % 100) / 50.0f,            // Massa
            0.1f,           // Raio
            glm::vec3(
                (rand() % 100) / 100.0f,
                (rand() % 100) / 100.0f,
                (rand() % 100) / 100.0f
            )
        );
        system.addParticle(p);
    }

    //BENCHMARKS

    double lastTime = glfwGetTime();
    int frameCount = 0;
    int totalFrames = 0;
    double fpsTimer = 0.0;

    std::cout << "\n=== BENCHMARKS ===" << std::endl;

    // Benchmark da atualização física
    benchmarkTime("Update do sistema", [&]() {
        system.update();
    });

    // Benchmark da preparação de dados
    benchmarkTime("Preparação de posições/cores", [&]() {
        std::vector<glm::vec2> positions;
        std::vector<glm::vec3> colors;
        
        for (const auto& p : system.getParticles()) {
            positions.push_back(glm::vec2(p.getPosition().getX(), p.getPosition().getY()));
            colors.push_back(glm::vec3(1.0f, 1.0f, 1.0f));
        }
    });

    // Benchmark da renderização
    benchmarkTime("Renderização", [&]() {
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, MAX_PARTICLES);
    });

    std::cout << "=================\n" << std::endl;

    // Loop principal
    while (!glfwWindowShouldClose(window)) {
        // Input
        processInput(window);

        // Limpa a tela
        glClear(GL_COLOR_BUFFER_BIT);

        // Atualiza a física
        system.update(); // uses internal dt
        
        // Renderização das partículas
        glUseProgram(shaderProgram);
        
        // Envia matriz de projeção
        GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // Cache das locations que vamos usar para desenhar
        GLint colorLoc = glGetUniformLocation(shaderProgram, "color");
        GLint pointSizeLoc = glGetUniformLocation(shaderProgram, "pointSize");

        // Prepara os dados das partículas
        std::vector<glm::vec2> positions;
        std::vector<glm::vec3> colors;
        std::vector<float> radii;
        std::vector<float> speeds;

            // Get simulation timestep from system
        float simDt = system.getTimeStep();

        // Extract positions, radii and compute speeds through the public API
        for (const auto& p : system.getParticles()) {
            Vec2 pos = p.getPosition();
            Vec2 prev = p.getPrevPosition();
            positions.push_back(glm::vec2(pos.getX(), pos.getY()));
            radii.push_back(p.getRadius());

            // velocity estimate from Verlet positions
            Vec2 dv = (pos - prev) / simDt;
            float sp = dv.length();
            speeds.push_back(sp);
        }

        // Determine a reasonable normalization for speed -> color mapping
        float maxSpeed = 0.0f;
        for (float s : speeds) if (s > maxSpeed) maxSpeed = s;
        // avoid tiny denom; choose a sensible scale (adjustable)
        const float speedScale = std::max(maxSpeed, 5.0f);

        // Build color array by mapping speed: blue (slow) -> red (fast)
        colors.reserve(speeds.size());
        for (float s : speeds) {
            float t = glm::clamp(s / speedScale, 0.0f, 1.0f);
            // simple lerp blue->red via (r,0,b)
            colors.push_back(glm::vec3(t, 0.0f, 1.0f - t));
        }

        // Atualiza o buffer com todas as posições
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec2), positions.data(), GL_DYNAMIC_DRAW);

        // Desenha as partículas: enviando color e pointSize por uniform
        const float sizeScale = 40.0f; // aumento: ajuste este valor para diminuir/aumentar o diâmetro visual
        glBindVertexArray(VAO);
        for (size_t i = 0; i < positions.size(); i++) {
            glUniform3fv(colorLoc, 1, glm::value_ptr(colors[i]));
            float sizePx = radii[i] * sizeScale;
            glUniform1f(pointSizeLoc, sizePx);
            glDrawArrays(GL_POINTS, static_cast<int>(i), 1);
        }

        double currentTime = glfwGetTime();
        double delta = currentTime - lastTime;
        lastTime = currentTime;
        fpsTimer += delta;
        frameCount++;
        totalFrames++;

        if (fpsTimer >= 1.0) {
            std::cout << "\rFPS: " << frameCount << std::flush;
            frameCount = 0;
            fpsTimer = 0.0;
        }
        
        // Troca buffers e verifica eventos
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    std::cout << "\nTotal frames renderizados: " << totalFrames << std::endl;
    std::cout << "Tempo total de execução: " << glfwGetTime() << " segundos" << std::endl;
    std::cout << "FPS médio: " << (totalFrames / glfwGetTime()) << std::endl;

    // Limpeza
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
    
    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
        
    // Adiciona nova partícula com espaço
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        // Implementar se necessário
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        // Implementar interação com mouse se necessário
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
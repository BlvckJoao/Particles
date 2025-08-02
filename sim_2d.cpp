#include "physics/particle_phsx.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>

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
    out vec3 ParticleColor;
    void main() {
        gl_Position = projection * vec4(aPos, 0.0, 1.0);
        gl_PointSize = 20.0; // Tamanho base dos pontos
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

int main() {
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
    ParticleSystem system(
        Vec2(0, -9.8f),    // Gravidade
        0.99f,             // Damping
        Vec2(-WORLD_WIDTH/2, -WORLD_HEIGHT/2),  // Limite inferior
        Vec2(WORLD_WIDTH/2, WORLD_HEIGHT/2),    // Limite superior
        0.8f               // Restituição
    );

    // Adiciona partículas iniciais
    for (int i = 0; i < 100; i++) {
        float x = (rand() % 100) * WORLD_WIDTH / 100.0f - WORLD_WIDTH/2;
        float y = (rand() % 100) * WORLD_HEIGHT / 100.0f - WORLD_HEIGHT/2;
        
        Particle p(
            Vec2(x, y), // Posição
            Vec2((rand() % 10 - 5), (rand() % 10 - 5)),   // Velocidade
            1.0f + (rand() % 100) / 50.0f,            // Massa
            0.2f + (rand() % 100) / 200.0f,           // Raio
            glm::vec3(
                rand() % 100 / 100.0f,
                rand() % 100 / 100.0f,
                rand() % 100 / 100.0f
            )
        );
        system.addParticle(p);
    }

    // Loop principal
    while (!glfwWindowShouldClose(window)) {
        // Input
        processInput(window);

        // Limpa a tela
        glClear(GL_COLOR_BUFFER_BIT);

        // Atualiza a física
        system.updateSystem(0.016f); // ~60 FPS
        
        // Renderização das partículas
        glUseProgram(shaderProgram);
        
        // Envia matriz de projeção
        glUniformMatrix4fv(
            glGetUniformLocation(shaderProgram, "projection"), 
            1, GL_FALSE, glm::value_ptr(projection)
        );
        
        // Prepara os dados das partículas
        std::vector<glm::vec2> positions;
        std::vector<glm::vec3> colors;
        
        for (const auto& p : system.particles) {
            positions.push_back(glm::vec2(p.position.x, p.position.y));
            colors.push_back(p.color);
        }
        
        // Atualiza o buffer
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec2), positions.data(), GL_DYNAMIC_DRAW);
        
        // Desenha as partículas
        glBindVertexArray(VAO);
        for (size_t i = 0; i < system.particles.size(); i++) {
            glUniform3fv(glGetUniformLocation(shaderProgram, "color"), 1, glm::value_ptr(colors[i]));
            glDrawArrays(GL_POINTS, i, 1);
        }
        
        // Troca buffers e verifica eventos
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

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
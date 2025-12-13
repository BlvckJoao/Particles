cmake_minimum_required(VERSION 3.10)
project(Cubo3D C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configurar caminhos para sua instalação do GLFW
set(GLFW_DIR "C:/GLFW" CACHE PATH "Caminho para GLFW instalado")
set(GLM_DIR "C:/libs/glm" CACHE PATH "Caminho para GLM")

# Incluir diretórios
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/glad
    ${GLFW_DIR}/include
    ${GLM_DIR}/include
)

# Adicionar executável
add_executable(Cubo3D
    main.cpp
    glad/glad.c
)

# Vincular bibliotecas - IMPORTANTE: usar o nome correto da lib gerada
target_link_libraries(Cubo3D
    ${GLFW_DIR}/lib/libglfw3.a  # Nome padrão quando compilado com MinGW
    opengl32
    gdi32
)

# Copiar shaders
add_custom_command(TARGET Cubo3D POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    ${CMAKE_CURRENT_BINARY_DIR}/shaders
)
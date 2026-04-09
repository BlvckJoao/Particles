# Particle Simulation - Linux

Simulação de partículas com OpenGL para Linux.

## Requisitos do Sistema

As seguintes dependências são necessárias:
- `build-essential` - Ferramentas de compilação
- `cmake` - Sistema de build
- `libglm-dev` - Biblioteca de matemática GLM
- `xorg-dev` - Bibliotecas de desenvolvimento X11
- `wayland-protocols` - Protocolos Wayland
- `libwayland-dev` - Bibliotecas de desenvolvimento Wayland
- `libxkbcommon-dev` - Biblioteca para keysyms do teclado

## Instalação das Dependências

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libglm-dev xorg-dev wayland-protocols libwayland-dev libxkbcommon-dev
```

## Compilação

### 1. Navegar para a pasta do projeto

```bash
cd linux
```

### 2. Criar diretório de build (se não existir)

```bash
mkdir -p build
cd build
```

### 3. Executar CMake

```bash
cmake ..
```

### 4. Compilar

```bash
make -j$(nproc)
```

ou simplesmente:

```bash
make
```

## Execução

### Opção 1: Usando o script de execução

```bash
cd linux
./run.sh
```

### Opção 2: Executar diretamente

```bash
./build/bin/ParticleSim
```

## Estrutura do Projeto

```
linux/
├── CMakeLists.txt    # Configuração do CMake
├── run.sh            # Script de execução
├── src/              # Código-fonte
│   ├── sim_2d.cpp    # Simulação 2D
│   └── particle_phsx.hpp  # Física das partículas
├── libs/             # Bibliotecas externas
│   ├── glad/         # Loader OpenGL
│   └── glfw/         # Window e input
└── build/            # Diretório de build (criado após compilação)
    └── bin/
        └── ParticleSim  # Executável final
```

## Troubleshooting

### Erro: "wayland-scanner not found"
A causa é falta de `wayland-protocols`. Instale com:
```bash
sudo apt-get install -y wayland-protocols libwayland-dev
```

### Erro: "glm.hpp not found"
Instale a biblioteca GLM:
```bash
sudo apt-get install -y libglm-dev
```

### Erro ao executar: "libGL not found"
Instale as bibliotecas OpenGL:
```bash
sudo apt-get install -y mesa-common-dev
```

## Notas

- O projeto foi compilado com sucesso no Ubuntu 24.04 LTS
- As dependências de Wayland permitem suporte a ambientes moderno com Wayland
- O projeto utiliza GLFW para criar janelas e gerenciar input
- Glad é usado para carregar funções OpenGL
- GLM (OpenGL Mathematics) é usado para cálculos matemáticos

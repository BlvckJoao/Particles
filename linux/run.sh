#!/bin/bash
# Script para rodar a simulação de partículas no Linux

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${PROJECT_DIR}/build"
BIN_DIR="${BUILD_DIR}/bin"

# Verificar se o diretório de build existe
if [ ! -d "${BUILD_DIR}" ]; then
    echo "Erro: Diretório de build não encontrado."
    echo "Execute cmake e make primeiro:"
    echo "  cd ${PROJECT_DIR}/build"
    echo "  cmake .."
    echo "  make -j\$(nproc)"
    exit 1
fi

# Verificar se o executável existe
if [ ! -f "${BIN_DIR}/ParticleSim" ]; then
    echo "Erro: Executável ParticleSim não encontrado em ${BIN_DIR}"
    exit 1
fi

echo "Iniciando Simulação de Partículas..."
exec "${BIN_DIR}/ParticleSim"

#ifndef SPACIAL_GRID_HPP
#define SPACIAL_GRID_HPP

#include <vector>
#include <cmath>
#include <algorithm>

#include "physics/particle.hpp"

struct SpatialGrid {
    float cellSize;
    int cols, rows;
    float left, top;
    std::vector<std::vector<size_t>> cells;

    void build(const std::vector<Particle>& particles, float left, float top,
               float right, float bottom, float cs) {
        cellSize = cs;
        cols = std::max(1, (int)std::ceil((right - left) / cellSize));
        rows = std::max(1, (int)std::ceil((bottom - top) / cellSize));
        this->left = left;
        this->top  = top;

        cells.assign(cols * rows, {});

        for (size_t i = 0; i < particles.size(); ++i) {
            if (!particles[i].isActive()) continue;
            int cx = cellCol(particles[i].getPosition().getX());
            int cy = cellRow(particles[i].getPosition().getY());
            cells[cy * cols + cx].push_back(i);
        }
    }

    int cellCol(float x) const {
        return std::clamp((int)((x - left) / cellSize), 0, cols - 1);
    }
    int cellRow(float y) const {
        return std::clamp((int)((y - top) / cellSize), 0, rows - 1);
    }

    // Retorna índices das partículas nas 9 células ao redor de (cx, cy)
    void getNeighbors(int cx, int cy, std::vector<size_t>& out) const {
        for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            const auto& cell = cells[ny * cols + nx];
            out.insert(out.end(), cell.begin(), cell.end());
        }
    }

    int getCellDensity(int cx, int cy) const {
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return 0;
        return (int)cells[cy * cols + cx].size();
    }

    int getMaxDensity() const {
        int maxD = 0;
        for (const auto& cell : cells)
            maxD = std::max(maxD, (int)cell.size());
        return maxD;
    }

    int getNumCols() const { return cols; }
    int getNumRows() const { return rows; }
};

#endif
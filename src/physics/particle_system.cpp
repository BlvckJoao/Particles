#include <vector>
#include <cmath>
#include <algorithm>
#include <future>
#include <utility>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "particle.hpp"
#include "particle_system.hpp"

// Constructor implementation
ParticleSystem::ParticleSystem(float left, float right, float top, float bottom,
                               size_t blockSize, float timeStep, float damp,
                               float collision_damp, size_t workers)
    : bounds{ left, right, top, bottom }, blockSize(blockSize), damping(damp),
      collision_damping(collision_damp), dt(timeStep),
      threadPool(std::max<size_t>(1, workers)),
      workerCount(std::max<size_t>(1, workers)) {}

void ParticleSystem::parallelFor(
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

// Getter for particles
const std::vector<Particle>& ParticleSystem::getParticles() const {
    return particles;
}

// Getter for timestep
float ParticleSystem::getTimeStep() const {
    return dt;
}

// Distance calculation
float ParticleSystem::distance(const Vec2& a, const Vec2& b) const {
    float dx = a.getX() - b.getX();
    float dy = a.getY() - b.getY();
    return std::sqrt(dx * dx + dy * dy);
}

// Add particle to system
void ParticleSystem::addParticle(const Particle& p) {
    particles.push_back(p);
}

void ParticleSystem::startHold(const std::vector<int>& indices,
                                const std::vector<Vec2>& offsets,
                                const Vec2& target) {
        heldParticles = indices;
        heldOffsets   = offsets;
        mouseTarget   = target;
        mouseHolding  = true;
}

void ParticleSystem::updateMouseTarget(const Vec2& target) {
        mouseTarget = target;
}

void ParticleSystem::releaseHold() {
        mouseHolding = false;
        heldParticles.clear();
        heldOffsets.clear();
}

void ParticleSystem::applyMouseForce() {
        if (!mouseHolding) return;

        const float stiffness = 300.0f; // força de atração — aumenta pra mais rígido
        const float damping   = 20.0f;  // amortece a oscilação

        for (size_t i = 0; i < heldParticles.size(); ++i) {
                int   idx    = heldParticles[i];
                Vec2  target = mouseTarget + heldOffsets[i];
                Vec2  pos    = particles[idx].getPosition();
                Vec2  vel    = (pos - particles[idx].getPrevPosition()) / dt;

                // Força mola: puxa em direção ao alvo
                Vec2  diff  = target - pos;
                Vec2  force = diff * stiffness - vel * damping;

                particles[idx].applyForce(force);
        }
}

void ParticleSystem::handleWorldBoundaries() {
    parallelFor(particles.size(), [this](size_t begin, size_t end) {
    for (size_t i = begin; i < end; ++i) {
        Particle& p = particles[i];
        Vec2  pos  = p.getPosition();
        Vec2  prev = p.getPrevPosition();
        float r    = p.getRadius();

        if (pos.getX() - r < bounds.left) {
            pos.setX(bounds.left + r);
            prev.setX(bounds.left + r + (bounds.left + r - prev.getX()));
        } else if (pos.getX() + r > bounds.right) {
            pos.setX(bounds.right - r);
            prev.setX(bounds.right - r + (bounds.right - r - prev.getX()));
        }

        if (pos.getY() - r < bounds.top) {
            pos.setY(bounds.top + r);
            prev.setY(bounds.top + r + (bounds.top + r - prev.getY()));
        } else if (pos.getY() + r > bounds.bottom) {
            pos.setY(bounds.bottom - r);
            prev.setY(bounds.bottom - r + (bounds.bottom - r - prev.getY()));
        }

        p.setPosition(pos);
        p.setPrevPosition(prev);
    }
    });
}

void ParticleSystem::setParticlePosition(int index, const Vec2& pos) {
        if (index < 0 || index >= (int)particles.size()) return;
        particles[index].setPosition(pos);
        particles[index].setPrevPosition(pos); // zera velocidade
}

// Sequential collision handling
void ParticleSystem::handleCollisionsSeq() {
    const size_t count = particles.size();

    for (size_t i = 0; i < count; ++i) {
        if (!particles[i].isActive()) continue;
        for (size_t j = i + 1; j < count; ++j) {
            if (!particles[j].isActive()) continue;

            Vec2 pos_i = particles[i].getPosition();
            Vec2 pos_j = particles[j].getPosition();

            float ri = particles[i].getRadius();
            float rj = particles[j].getRadius();
            float rsum = ri + rj;

            Vec2 delta = pos_j - pos_i;
            float dist = delta.length();

            if (dist <= 0.0f || dist >= rsum)
                continue;

            // Normal de colisão
            Vec2 n = delta / dist;

            // Penetração
            float penetration = rsum - dist;

            // Massas
            float mi = particles[i].getMass();
            float mj = particles[j].getMass();
            float sum = mi + mj;

            // Pesos (quebram simetria numérica)
            float wi = mj / sum;
            float wj = mi / sum;

            // Correção posicional
            Vec2 correction = n * penetration;

            Vec2 newPosI = pos_i - correction * wi;
            Vec2 newPosJ = pos_j + correction * wj;

            particles[i].setPosition(newPosI);
            particles[j].setPosition(newPosJ);

            // --- ajuste de "velocidade" via prev_position ---
            Vec2 prev_i = particles[i].getPrevPosition();
            Vec2 prev_j = particles[j].getPrevPosition();

            Vec2 vel_i = (pos_i - prev_i) / dt;
            Vec2 vel_j = (pos_j - prev_j) / dt;

            // componente normal
            float vi_n = vel_i.getX() * n.getX() + vel_i.getY() * n.getY();
            float vj_n = vel_j.getX() * n.getX() + vel_j.getY() * n.getY();

            float rel = vi_n - vj_n;

            if (rel < 0.0f) {
                float e = collision_damping;

                float vi_n_new = -vi_n * e;
                float vj_n_new = -vj_n * e;

                Vec2 vel_i_new = vel_i + n * (vi_n_new - vi_n);
                Vec2 vel_j_new = vel_j + n * (vj_n_new - vj_n);

                particles[i].setPrevPosition(newPosI - vel_i_new * dt);
                particles[j].setPrevPosition(newPosJ - vel_j_new * dt);
            } else {
                particles[i].setPrevPosition(newPosI - vel_i * dt);
                particles[j].setPrevPosition(newPosJ - vel_j * dt);
            }
        }
    }
}

// Handle collision between two particles
void ParticleSystem::handleCollision(Particle& p1, Particle& p2) {
        Vec2  pos_i = p1.getPosition();
        Vec2  pos_j = p2.getPosition();
        float rsum  = p1.getRadius() + p2.getRadius();
        Vec2  delta = pos_i - pos_j;
        float dist  = delta.length();

        if (dist >= rsum || dist <= 0.0f) return;

        Vec2  n           = delta / dist;
        float penetration = rsum - dist;

        p1.setPosition(pos_i + n * penetration * 0.5f);
        p2.setPosition(pos_j - n * penetration * 0.5f);
}
// Optimized collision handling with cache blocking UNUSED
void ParticleSystem::optmizedCollisionHandling() {
    //cache blocking and checking only nearby particles would go here
    const size_t count = particles.size();

    for(size_t ii = 0; ii < count; ii += blockSize) {
        for(size_t jj = ii + blockSize; jj < count; jj += blockSize) {
            for(size_t i = ii; i < std::min(ii + blockSize, count); ++i) {
                if (!particles[i].isActive()) continue;
                for(size_t j = jj; j < std::min(jj + blockSize, count); ++j) {
                    if (!particles[j].isActive()) continue;
                    handleCollision(particles[i], particles[j]); // neighbor counts would be passed here if used
                }
            }
        }
    }
}

// Collision handling using spatial grid
void ParticleSystem::handleCollisionsSpatialGrid(const SpatialGrid& grid) {
        using CellPair = std::pair<size_t, size_t>;

        auto solveCellPairs = [this, &grid](const std::vector<CellPair>& cellPairs) {
                parallelFor(cellPairs.size(), [this, &grid, &cellPairs](size_t begin, size_t end) {
                        for (size_t pairIndex = begin; pairIndex < end; ++pairIndex) {
                                const auto [firstCell, secondCell] = cellPairs[pairIndex];
                                const auto& a = grid.cells[firstCell];
                                const auto& b = grid.cells[secondCell];

                                if (firstCell == secondCell) {
                                        for (size_t i = 0; i < a.size(); ++i)
                                                for (size_t j = i + 1; j < a.size(); ++j)
                                                        handleCollision(particles[a[i]], particles[a[j]]);
                                } else {
                                        for (size_t i : a)
                                                for (size_t j : b)
                                                        handleCollision(particles[i], particles[j]);
                                }
                        }
                });
        };

        // Color 0: cells do not share particles, so all within-cell contacts
        // can be solved concurrently without locks.
        std::vector<CellPair> cellPairs;
        cellPairs.reserve(grid.cells.size());
        for (size_t cell = 0; cell < grid.cells.size(); ++cell) {
                if (grid.cells[cell].size() > 1)
                        cellPairs.emplace_back(cell, cell);
        }
        solveCellPairs(cellPairs);

        // Each direction is split by the parity of the changing coordinate.
        // Within one phase the resulting cell pairs are disjoint: no cell, and
        // therefore no particle, can be touched by two workers simultaneously.
        constexpr int neighborOffsets[4][2] = {
                {1, -1}, {1, 0}, {1, 1}, {0, 1}
        };
        for (const auto& offset : neighborOffsets) {
                for (int parity = 0; parity < 2; ++parity) {
                        cellPairs.clear();
                        for (int cy = 0; cy < grid.rows; ++cy) {
                                for (int cx = 0; cx < grid.cols; ++cx) {
                                        const int changingCoordinate = offset[0] != 0 ? cx : cy;
                                        if ((changingCoordinate & 1) != parity) continue;

                                        const int nx = cx + offset[0];
                                        const int ny = cy + offset[1];
                                        if (nx < 0 || nx >= grid.cols || ny < 0 || ny >= grid.rows)
                                                continue;

                                        const size_t first = static_cast<size_t>(cy * grid.cols + cx);
                                        const size_t second = static_cast<size_t>(ny * grid.cols + nx);
                                        if (!grid.cells[first].empty() && !grid.cells[second].empty())
                                                cellPairs.emplace_back(first, second);
                                }
                        }
                        solveCellPairs(cellPairs);
                }
        }
}

// Apply gravity to all particles
void ParticleSystem::applyGravity() {
    // Gravidade aponta para baixo (y negativo)
    Vec2 gravity(0.0f, -9.81f);

    parallelFor(particles.size(), [this, gravity](size_t begin, size_t end) {
    for (size_t i = begin; i < end; ++i) {
        Particle& p = particles[i];
        if (p.isActive()) {
            p.applyForce(gravity * p.getMass());
        }
    }
    });
}

// Update sleep state of particles based on velocity
void ParticleSystem::updateSleepState(float sleepThreshold) {
    parallelFor(particles.size(), [this, sleepThreshold](size_t begin, size_t end) {
    for (size_t i = begin; i < end; ++i) {
        Particle& p = particles[i];
        if (!p.isActive()) continue;
        Vec2 vel = (p.getPosition() - p.getPrevPosition()) / dt;
        if (vel.length() < sleepThreshold)
            p.setSleeping(true);
        else
            p.setSleeping(false);
    }
    });
}

// Main update function
void ParticleSystem::update() {
        constexpr int SUBSTEPS = 8;

        applyGravity();
        applyMouseForce();

        parallelFor(particles.size(), [this](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i)
                        particles[i].verletIntegration(dt, damping);
        });

        handleWorldBoundaries();

        float maxRadius = 0.0f;
        for (const auto& p : particles)
                maxRadius = std::max(maxRadius, p.getRadius());

        if (maxRadius <= 0.0f) return;

        for (int s = 0; s < SUBSTEPS; ++s) {
                SpatialGrid grid;
                grid.build(particles, bounds.left, bounds.top,
                           bounds.right, bounds.bottom, 2.0f * maxRadius);
                handleCollisionsSpatialGrid(grid);
        }
}

void ParticleSystem::clearInactiveParticles() {
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
                       [](const Particle& p) { return !p.isActive(); }),
        particles.end()
    );
}

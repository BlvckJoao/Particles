#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "particle.hpp"
#include "particle_system.hpp"

// Constructor implementation
ParticleSystem::ParticleSystem(float left, float right, float top, float bottom, size_t blockSize, float timeStep, float damp, float collision_damp)
    : bounds{ left, right, top, bottom }, blockSize(blockSize), damping(damp), dt(timeStep), collision_damping(collision_damp) {}

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
    for (auto& p : particles) {
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
void ParticleSystem::handleCollision(Particle& p1, Particle& p2, int neighbors_i, int neighbors_j) {
        Vec2 pos_i = p1.getPosition();
        Vec2 pos_j = p2.getPosition();

        float rsum = p1.getRadius() + p2.getRadius();
        Vec2  delta = pos_i - pos_j;
        float dist  = delta.length();

        if (dist >= rsum || dist <= 0.0f) return;

        Vec2  n           = delta / dist;
        float penetration = rsum - dist;

        float avg_neighbors = (neighbors_i + neighbors_j) * 0.5f;
        float density_push  = 1.0f + avg_neighbors * 0.05f;

        // Só correção posicional, 50/50 — nunca toca em prev_position
        p1.setPosition(pos_i + n * penetration * 0.5f * density_push);
        p2.setPosition(pos_j - n * penetration * 0.5f * density_push);
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
                    handleCollision(particles[i], particles[j], 0, 0); // neighbor counts would be passed here if used
                }
            }
        }
    }
}

// Collision handling using spatial grid
void ParticleSystem::handleCollisionsSpatialGrid(const SpatialGrid& grid) {
        const size_t count = particles.size();

        for (size_t ii = 0; ii < count; ii += blockSize) {
                size_t iEnd = std::min(ii + blockSize, count);

                for (size_t i = ii; i < iEnd; ++i) {
                        if (!particles[i].isActive()) continue;

                        int cx = grid.cellCol(particles[i].getPosition().getX());
                        int cy = grid.cellRow(particles[i].getPosition().getY());

                        // Vizinhos só da célula de i — sem acumular vetor global
                        std::vector<size_t> neighbors;
                        grid.getNeighbors(cx, cy, neighbors);
                        int ni = (int)neighbors.size();

                        for (size_t jIdx = 0; jIdx < neighbors.size(); ++jIdx) {
                                size_t j = neighbors[jIdx];
                                if (j <= i) continue; // evita par duplicado e auto-colisão
                                if (!particles[j].isActive()) continue;

                                handleCollision(particles[i], particles[j], ni, ni);
                        }
                }
        }
}


// Apply gravity to all particles
void ParticleSystem::applyGravity() {
    // Gravidade aponta para baixo (y negativo)
    Vec2 gravity(0.0f, -9.81f);

    for (auto& p : particles) {
        if (p.isActive()) {
            p.applyForce(gravity * p.getMass());
        }
    }
}

// Update sleep state of particles based on velocity
void ParticleSystem::updateSleepState(float sleepThreshold) {
    for (auto& p : particles) {
        if (!p.isActive()) continue;
        Vec2 vel = (p.getPosition() - p.getPrevPosition()) / dt;
        if (vel.length() < sleepThreshold)
            p.setSleeping(true);
        else
            p.setSleeping(false);
    }
}

// Main update function
void ParticleSystem::update() {
        constexpr int SUBSTEPS = 8;

        applyGravity();
        applyMouseForce();

        for (auto& p : particles)
                p.verletIntegration(dt, damping);

        handleWorldBoundaries();

        float maxRadius = 0.0f;
        for (const auto& p : particles)
                maxRadius = std::max(maxRadius, p.getRadius());

        SpatialGrid grid;
        grid.build(particles, bounds.left, bounds.top,
                    bounds.right, bounds.bottom, 2.0f * maxRadius);
        for (int s = 0; s < SUBSTEPS; ++s) {
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
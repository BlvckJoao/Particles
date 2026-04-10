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

// Handle world boundary collisions
void ParticleSystem::handleWorldBoundaries() {
    for (auto& p : particles) {
        Vec2 pos = p.getPosition();
        float r = p.getRadius();

        if (pos.getX() - r < bounds.left) {
            pos.setX(bounds.left + r);
        }
        else if (pos.getX() + r > bounds.right) {
            pos.setX(bounds.right - r);
        }

        if (pos.getY() - r < bounds.top) {
            pos.setY(bounds.top + r);
        }
        else if (pos.getY() + r > bounds.bottom) {
            pos.setY(bounds.bottom - r);
        }

        p.setPosition(pos);
    }
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
    Vec2 pos_i = p1.getPosition();
    Vec2 pos_j = p2.getPosition();

    float ri = p1.getRadius();
    float rj = p2.getRadius();
    float rsum = ri + rj;

    Vec2 delta = pos_j - pos_i;
    float dist = delta.length();

    if (dist <= 0.0f || dist >= rsum)
        return;

    // Normal de colisão
    Vec2 n = delta / dist;

    // Penetração
    float penetration = rsum - dist;

    // Massas
    float mi = p1.getMass();
    float mj = p2.getMass();
    float sum = mi + mj;

    // Pesos (quebram simetria numérica)
    float wi = mj / sum;
    float wj = mi / sum;

    // Correção posicional
    Vec2 correction = n * penetration;

    Vec2 newPosI = pos_i - correction * wi;
    Vec2 newPosJ = pos_j + correction * wj;

    p1.setPosition(newPosI);
    p2.setPosition(newPosJ);

    // --- ajuste de "velocidade" via prev_position ---
    Vec2 prev_i = p1.getPrevPosition();
    Vec2 prev_j = p2.getPrevPosition();

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

        p1.setPrevPosition(newPosI - vel_i_new * dt);
        p2.setPrevPosition(newPosJ - vel_j_new * dt);
    } else {
        p1.setPrevPosition(newPosI - vel_i * dt);
        p2.setPrevPosition(newPosJ - vel_j * dt);
    }
}

// Optimized collision handling with cache blocking
void ParticleSystem::optmizedCollisionHandling() {
    //cache blocking and checking only nearby particles would go here
    const size_t count = particles.size();

    for(size_t ii = 0; ii < count; ii += blockSize) {
        for(size_t jj = ii + blockSize; jj < count; jj += blockSize) {
            for(size_t i = ii; i < std::min(ii + blockSize, count); ++i) {
                if (!particles[i].isActive()) continue;
                for(size_t j = jj; j < std::min(jj + blockSize, count); ++j) {
                    if (!particles[j].isActive()) continue;
                    handleCollision(particles[i], particles[j]);
                }
            }
        }
    }
}

// Collision handling using spatial grid
void ParticleSystem::handleCollisionsSpatialGrid() {

    float maxRadius = 0.0f;
    for (const auto& p : particles)
        maxRadius = std::max(maxRadius, p.getRadius());

    SpatialGrid grid;
    grid.build(particles, bounds.left, bounds.top,
            bounds.right, bounds.bottom, 2.0f * maxRadius);

    const size_t count = particles.size();

    for (size_t ii = 0; ii < count; ii += blockSize) {
        size_t iEnd = std::min(ii + blockSize, count);

        // Coleta células vizinhas de todas as partículas do bloco i
        std::vector<size_t> neighbors;
        for (size_t i = ii; i < iEnd; ++i) {
            if (!particles[i].isActive()) continue;
            int cx = grid.cellCol(particles[i].getPosition().getX());
            int cy = grid.cellRow(particles[i].getPosition().getY());
            grid.getNeighbors(cx, cy, neighbors);
        }

        // Remove duplicatas dos vizinhos coletados
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());

        // Cache blocking sobre os vizinhos
        for (size_t jj = 0; jj < neighbors.size(); jj += blockSize) {
            size_t jEnd = std::min(jj + blockSize, neighbors.size());

            for (size_t i = ii; i < iEnd; ++i) {
                if (!particles[i].isActive()) continue;
                for (size_t jIdx = jj; jIdx < jEnd; ++jIdx) {
                    size_t j = neighbors[jIdx];
                    if (j <= i) continue;
                    if (!particles[j].isActive()) continue;
                    handleCollision(particles[i], particles[j]);
                }
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
    applyGravity();

    for (auto& p : particles)
        p.verletIntegration(dt, damping);

    handleWorldBoundaries();

    // Monta grid uma vez fora do solver
    float maxRadius = 0.0f;
    for (const auto& p : particles)
        maxRadius = std::max(maxRadius, p.getRadius());

    SpatialGrid grid;
    grid.build(particles, bounds.left, bounds.top,
            bounds.right, bounds.bottom, 2.0f * maxRadius);

    int maxDensity = grid.getMaxDensity();

    const int maxIterations = 4;
    const int minIterations = 1;
    const int densityThreshold = 8; // acima disso começa a reduzir

    for (size_t ii = 0; ii < particles.size(); ii += blockSize) {
        size_t iEnd = std::min(ii + blockSize, particles.size());

        // Calcula densidade média do bloco
        int blockDensity = 0;
        int activeInBlock = 0;
        for (size_t i = ii; i < iEnd; ++i) {
            if (!particles[i].isActive()) continue;
            int cx = grid.cellCol(particles[i].getPosition().getX());
            int cy = grid.cellRow(particles[i].getPosition().getY());
            blockDensity += grid.getCellDensity(cx, cy);
            activeInBlock++;
        }

        if (activeInBlock == 0) continue;
        blockDensity /= activeInBlock;

        // Escala iterações inversamente à densidade
        int iterations = maxIterations;
        if (blockDensity > densityThreshold) {
            float densityRatio = (float)densityThreshold / (float)blockDensity;
            iterations = std::max(minIterations,
                                (int)std::round(maxIterations * densityRatio));
        }

        // Roda o solver com iterações adaptativas pra esse bloco
        std::vector<size_t> neighbors;
        for (int k = 0; k < iterations; ++k) {
            neighbors.clear();
            for (size_t i = ii; i < iEnd; ++i) {
                if (!particles[i].isActive()) continue;
                int cx = grid.cellCol(particles[i].getPosition().getX());
                int cy = grid.cellRow(particles[i].getPosition().getY());
                grid.getNeighbors(cx, cy, neighbors);
            }

            std::sort(neighbors.begin(), neighbors.end());
            neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());

            for (size_t jj = 0; jj < neighbors.size(); jj += blockSize) {
                size_t jEnd = std::min(jj + blockSize, neighbors.size());
                for (size_t i = ii; i < iEnd; ++i) {
                    if (!particles[i].isActive()) continue;
                    for (size_t jIdx = jj; jIdx < jEnd; ++jIdx) {
                        size_t j = neighbors[jIdx];
                        if (j <= i) continue;
                        if (!particles[j].isActive()) continue;
                        handleCollision(particles[i], particles[j]);
                    }
                }
            }
        }
    }
}

void ParticleSystem::clearInactiveParticles() {
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
                       [](const Particle& p) { return !p.isActive(); }),
        particles.end()
    );
}
#ifndef PARTICLE_PHSX_HPP
#define PARTICLE_PHSX_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Vec2{
    private:
        float x, y;

    public:
        Vec2() : x(0.0f), y(0.0f) {}
        Vec2(float x, float y) : x(x), y(y) {}

        Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
        Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
        Vec2 operator*(const float scalar) const { return Vec2(x * scalar, y * scalar); }
        Vec2 operator/(const float scalar) const { return Vec2(x / scalar, y / scalar); }

        Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
        Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
        Vec2& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
        Vec2& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }

        float getX() const { return x; }
        float getY() const { return y; }
        void setX(float val) { x = val; }
        void setY(float val) { y = val; }

        float length() const { return std::sqrt(x * x + y * y); }

        void clear() { x = 0.0f; y = 0.0f; }
        static Vec2 zero() { return Vec2(0.0f, 0.0f); }
};

struct Particle{
    private:
        Vec2 position;
        Vec2 prev_position;
        Vec2 velocity;
        Vec2 acceleration;
        Vec2 forceAccumulator;
        float mass;
        float radius;
        glm::vec3 color;
        bool is_active;
        bool is_sleeping;

    public:

        // Ajustado: construtor aceita (pos, vel, mass, radius, color[, acc])
        Particle(const Vec2& pos, const Vec2& vel, float m, float rad,
                 const glm::vec3& col = glm::vec3(1.0f), const Vec2& acc = Vec2::zero()) :
            position(pos),
            prev_position(pos),
            velocity(vel),
            acceleration(acc),
            forceAccumulator(Vec2::zero()),
            mass(std::max(1e-6f, m)),
            radius(rad),
            color(col),
            is_active(true),
            is_sleeping(false) {}

        void applyForce(const Vec2& f) { forceAccumulator += f; }
        void clearForces() { forceAccumulator.clear(); }

        const Vec2& getPosition() const noexcept { return position; }
        void setPosition(const Vec2& p) noexcept { position = p; }
        const Vec2& getPrevPosition() const noexcept { return prev_position; }
        void setPrevPosition(const Vec2& p) noexcept { prev_position = p; }
        const glm::vec3& getColor() const { return color; }

        float getMass() const { return mass; }
        void setMass(float m) { mass = std::max(1e-6f, m); }

        float getRadius() const { return radius; }
        void setRadius(float r) { radius = r; }

        bool isActive() const noexcept { return is_active; }
        void deactivate(bool p) noexcept { is_active = !p; }

        bool isSleeping() const noexcept { return is_sleeping; }
        void setSleeping(bool s) noexcept { is_sleeping = s; }

        void verletIntegration(float dt, float damping = 0.999f){
            if(!is_active){
                prev_position = position;
                clearForces();
                acceleration.clear();
                return;
            }

            acceleration = forceAccumulator / mass;

            Vec2 temp = position;
            position += (position - prev_position) * damping + acceleration * (dt * dt);
            prev_position = temp;

            clearForces();
        }

        void velocityVerlet(float dt, float damping = 0.999f){
            if (!is_active) {
                prev_position = position;
                clearForces();
                acceleration.clear();
                return;
            }

            Vec2 a_old = acceleration;
            acceleration = forceAccumulator / mass;

            position = position + velocity * dt + a_old * (0.5f * dt * dt);

            velocity = velocity + (a_old + acceleration) * (0.5f * dt);

            velocity = velocity * damping;
            prev_position = position;

            clearForces();
        }
};

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
};

class ParticleSystem {
    private:
        typedef struct {
            float left, right, top, bottom;
        }world_boundary;

        std::vector<Particle> particles;
        world_boundary bounds;
        size_t blockSize;
        float damping;
        float collision_damping;
        float dt;

    public:

        ParticleSystem(float left, float right, float top, float bottom, size_t blockSize, float timeStep, float damp, float collision_damp = 0.85f) :
            bounds{ left, right, top, bottom }, blockSize(blockSize), damping(damp), dt(timeStep), collision_damping(collision_damp) {}

        const std::vector<Particle>& getParticles() const {
           return particles;
        }

        // Expose timestep so render can compute per-particle velocities
        float getTimeStep() const { return dt; }

        float distance(const Vec2& a, const Vec2& b) const {
            float dx = a.getX() - b.getX();
            float dy = a.getY() - b.getY();
            return std::sqrt(dx * dx + dy * dy);
        }
        
        void addParticle(const Particle& p) {
           particles.push_back(p);
        }

        void handleWorldBoundaries() {
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


        void handleCollisionsSeq() {
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

        void handleCollision(Particle& p1, Particle& p2) {
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

        void optmizedCollisionHandling() {
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

        void handleCollisionsSpatialGrid() {

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

        void applyGravity() {
            // Gravidade aponta para baixo (y negativo)
            Vec2 gravity(0.0f, -9.81f);
            for (auto& p : particles) {
                if (p.isActive()) {
                    p.applyForce(gravity * p.getMass());
                }
            }

        }

        void updateSleepState(float sleepThreshold = 0.001f) {
            for (auto& p : particles) {
                if (!p.isActive()) continue;
                Vec2 vel = (p.getPosition() - p.getPrevPosition()) / dt;
                if (vel.length() < sleepThreshold)
                    p.setSleeping(true);
                else
                    p.setSleeping(false);
            }
        }

        void update() {
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

        void clearInactiveParticles() {
            particles.erase(
                std::remove_if(particles.begin(), particles.end(),
                               [](const Particle& p) { return !p.isActive(); }),
                particles.end()
            );
        }
    };

#endif
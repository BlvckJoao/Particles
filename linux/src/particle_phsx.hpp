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
            is_active(true) {}

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

class ParticleSystem {
    private:
        typedef struct {
            float left, right, top, bottom;
        }world_boundary;

        std::vector<Particle> particles;
        world_boundary bounds;
        float damping;
        float collision_damping;
        float dt;

    public:

        ParticleSystem(float left, float right, float top, float bottom, float timeStep, float damp, float collision_damp = 0.85f) :
            bounds{ left, right, top, bottom }, damping(damp), dt(timeStep), collision_damping(collision_damp) {}

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


        void handleCollisions() {
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

        void applyGravity() {
            // Gravidade aponta para baixo (y negativo)
            Vec2 gravity(0.0f, -9.81f);
            for (auto& p : particles) {
                if (p.isActive()) {
                    p.applyForce(gravity * p.getMass());
                }
            }

        }

        void update() {
            applyGravity();

            for (auto& p : particles)
                p.verletIntegration(dt, damping);

            handleWorldBoundaries();

            const int solverIterations = 4;
            for (int k = 0; k < solverIterations; ++k)
                handleCollisions();
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
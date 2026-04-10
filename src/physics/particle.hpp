#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "math/vec2.hpp"

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

#endif
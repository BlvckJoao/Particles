#ifndef PARTICLE_PHSX_HPP
#define PARTICLE_PHSX_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "math/vec2.hpp"
#include "physics/particle.hpp"
#include "spatial/spacial_grid.hpp"

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

        void handleCollision(Particle& p1, Particle& p2);

    public:
        ParticleSystem(float left, float right, float top, float bottom, size_t blockSize, float timeStep, float damp, float collision_damp = 0.85f);

        const std::vector<Particle>& getParticles() const;

        // Expose timestep so render can compute per-particle velocities
        float getTimeStep() const;

        float distance(const Vec2& a, const Vec2& b) const;
        
        void addParticle(const Particle& p);

        void handleWorldBoundaries();

        void handleCollisionsSeq();

        void optmizedCollisionHandling();

        void handleCollisionsSpatialGrid();

        void applyGravity();

        void updateSleepState(float sleepThreshold = 0.001f);

        void update();

        void clearInactiveParticles();
};

#endif
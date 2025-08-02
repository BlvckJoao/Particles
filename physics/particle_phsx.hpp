#ifndef PARTICLE_PHSX_HPP
#define PARTICLE_PHSX_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Vec2 {
public:
    float x, y;
    
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    
    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
    
    float length() const {
        return std::sqrt(x*x + y*y);
    }
    
    Vec2 normalized() const {
        float len = length();
        if (len > 0) {
            return Vec2(x/len, y/len);
        }
        return *this;
    }
};

struct Particle {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    Vec2 force_accumulator;
    float mass;
    float radius;
    bool is_active;
    glm::vec3 color;
    
    Particle(Vec2 pos = Vec2(), Vec2 vel = Vec2(), float m = 1.0f, float r = 5.0f, glm::vec3 col = glm::vec3(1.0f)) 
        : position(pos), velocity(vel), mass(m), radius(r), 
          acceleration(Vec2()), force_accumulator(Vec2()), is_active(true), color(col) {}
    
    void applyForce(const Vec2& force) {
        force_accumulator += force;
    }
    
    void update(float dt) {
        if (!is_active) return;
        
        const float MAX_DT = 0.016f;
        dt = std::min(dt, MAX_DT);
        
        acceleration = force_accumulator * (1.0f/mass);
        velocity += acceleration * dt;
        position += velocity * dt;
        
        force_accumulator = Vec2();
    }
};

class ParticleSystem {
private:
    Vec2 gravity;
    float damping;
    Vec2 world_bounds_min;
    Vec2 world_bounds_max;
    float restitution;
    
public:
    std::vector<Particle> particles;

    ParticleSystem(Vec2 g = Vec2(0, 9.8f), float damp = 0.99f, 
                 Vec2 bounds_min = Vec2(-1000, -1000), Vec2 bounds_max = Vec2(1000, 1000),
                 float rest = 0.8f)
        : gravity(g), damping(damp), 
          world_bounds_min(bounds_min), world_bounds_max(bounds_max),
          restitution(rest) {}

    void addParticle(const Particle& particle) {
        particles.push_back(particle);
    }

    void updateSystem(float dt) {
        for(auto& particle : particles) {
            if (!particle.is_active) continue;
            
            particle.applyForce(gravity * particle.mass);
            particle.update(dt);
            particle.velocity = particle.velocity * damping;
            
            handleWorldBoundaries(particle);
        }
        
        handleCollisions();
    }

    void handleWorldBoundaries(Particle& p) {
        if (p.position.x - p.radius < world_bounds_min.x) {
            p.position.x = world_bounds_min.x + p.radius;
            p.velocity.x = -p.velocity.x * restitution;
        } 
        else if (p.position.x + p.radius > world_bounds_max.x) {
            p.position.x = world_bounds_max.x - p.radius;
            p.velocity.x = -p.velocity.x * restitution;
        }
        
        if (p.position.y - p.radius < world_bounds_min.y) {
            p.position.y = world_bounds_min.y + p.radius;
            p.velocity.y = -p.velocity.y * restitution;
        } 
        else if (p.position.y + p.radius > world_bounds_max.y) {
            p.position.y = world_bounds_max.y - p.radius;
            p.velocity.y = -p.velocity.y * restitution;
        }
    }

    void handleCollisions() {
        for(size_t i = 0; i < particles.size(); ++i) {
            if (!particles[i].is_active) continue;
            
            for(size_t j = i + 1; j < particles.size(); ++j) {
                if (!particles[j].is_active) continue;
                
                Vec2 diff = particles[i].position - particles[j].position;
                float distance = diff.length();
                float min_distance = particles[i].radius + particles[j].radius;
                
                if(distance < min_distance) {
                    Vec2 normal = diff.normalized();
                    float overlap = min_distance - distance;
                    
                    particles[i].position += normal * (overlap * 0.5f);
                    particles[j].position -= normal * (overlap * 0.5f);
                    
                    Vec2 relativeVelocity = particles[i].velocity - particles[j].velocity;
                    float velocityAlongNormal = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;
                    
                    if(velocityAlongNormal < 0) {
                        float impulse = -(1 + restitution) * velocityAlongNormal;
                        impulse /= (1.0f/particles[i].mass + 1.0f/particles[j].mass);
                        
                        particles[i].velocity += normal * (impulse / particles[i].mass);
                        particles[j].velocity -= normal * (impulse / particles[j].mass);
                    }
                }
            }
        }
    }
    
    void clearInactiveParticles() {
        particles.erase(
            std::remove_if(particles.begin(), particles.end(), 
                         [](const Particle& p) { return !p.is_active; }),
            particles.end()
        );
    }
};
#endif
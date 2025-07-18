#ifndef PARTICLE_PHSX_HPP
#define PARTICLE_PHSX_HPP

#include <vector>
#include <cmath>

typedef struct{

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
}Vec2;

typedef struct{
    Vec2 posisition;
    Vec2 velocity;
    Vec2 acceleration;
    float mass;
    float radius;
}Particle;

class ParticleSystem{
    public:
        std::vector<Particle> particles;

        void addParticle(const Particle& particle) {
            particles.push_back(particle);
        }
        // Adiciona uma partícula ao sistema
    void addParticle(const Particle& p) {
        particles.push_back(p);
    }
    // Atualiza todas as partículas no sistema
    void update(float dt) {
        // Aplica forças e atualiza cada partícula
        for (auto& p : particles) {
            // Aplica gravidade (F = m*a)
            p.applyForce(gravity * p.mass);
            
            // Atualiza a partícula
            p.update(dt);
        }
        
        // Aqui você poderia adicionar detecção de colisões
    }  
    // Obtém todas as partículas (para renderização)
    const std::vector<Particle>& getParticles() const {
        return particles;
    }
};
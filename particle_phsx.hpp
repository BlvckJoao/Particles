#ifndef PARTICLE_PHSX_HPP
#define PARTICLE_PHSX_HPP

typedef struct{
    float x, y, z; //position
    float vx, vy, vz; //velocity
    float ax, ay, az; //acceleration
    float radius; //radius
    float mass = 1.0f;
}Particle;

//Particle structure

#endif
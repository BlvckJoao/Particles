# Particle Physics

The particle physics implementation is divided in three main archives, `particle.hpp`, `particle_system.hpp`, and `particle_system.cpp.

---

## Overview:

The numerical method used to compute the position of each paticle is the Verlet’s method, wich aproximates the solution for the position of the particle at each given frame based only on the current position, past position and acceleration of the particle. Following the formula:

$$
x(t + \Delta t) \approx 2x(t) - x(t- \Delta t) + a(t)\cdot \Delta t²
$$

With $\Delta t$ representing a small timestep, for this simulation we are using a 1/120 s timestep.

The simulation simply consists of particles in a closed box with gravity and collisions applied to them.

---

## Particle.hpp:

Defines the `Particle` struct, the atomic unit of the simulation. Each particle encapsulates its kinematic state, physical properties, and integration logic.

---

### Member Variables

| Field | Type | Description |
| --- | --- | --- |
| `position` | `Vec2` | Current position x(t)x(t)
x(t) |
| `prev_position` | `Vec2` | Position at previous timestep x(t−Δt)x(t - \Delta t)
x(t−Δt) |
| `velocity` | `Vec2` | Used only by `velocityVerlet` |
| `acceleration` | `Vec2` | Derived from accumulated forces each step |
| `forceAccumulator` | `Vec2` | Sum of all forces applied before integration |
| `mass` | `float` | Clamped to ≥10−6\geq 10^{-6}
≥10−6 to avoid division by zero |
| `radius` | `float` | Used for collision detection |
| `color` | `glm::vec3` | RGB, purely visual |
| `is_active` | `bool` | Inactive particles skip integration |
| `is_sleeping` | `bool` | Reserved for sleep optimization |

---

### Notes

- `forceAccumulator` is cleared at the end of every integration step — forces must be re-applied each frame.
- Inactive particles have their `prev_position` frozen to `position`, preventing accumulated drift from resuming incorrectly when reactivated.
- `is_sleeping` is declared but not yet used by the integrator — intended for a future optimization where low-energy particles skip integration.
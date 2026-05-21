#pragma once
#include <glm/glm.hpp>

// =============================================================================
// ShapeType
//   Enum that drives both the physics extents (radius vs halfHeight) and the
//   rendering branch (which mesh + shader to use).
// =============================================================================
enum class ShapeType { Triangle, Square, Circle, Ramp };

// =============================================================================
// RigidBody
//   A single simulated object. Holds all kinematic state, material properties,
//   and geometric extents. Call configure() to initialise, then update() each
//   fixed timestep.
// =============================================================================
struct RigidBody {

    ShapeType type;

    // ---- Linear kinematics ----
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 gravity;      // Per-body gravity allows easy customisation later

    // ---- Rotational kinematics ----
    float angle;            // Current orientation in radians
    float angularVelocity;  // Radians per second
    float torque;           // Accumulated this frame; reset after integration

    // ---- Mass & inertia ----
    float mass;
    float invMass;          // 1/mass; 0 == infinite mass (static body)
    float inertia;          // Moment of inertia (resistance to angular acceleration)
    float invInertia;       // 1/inertia; 0 == non-rotating

    // ---- Material ----
    float dragCoefficient;  // Linear air resistance scalar
    float restitution;      // Bounciness [0 = dead stop, 1 = perfectly elastic]
    float friction;         // Surface traction coefficient

    // ---- Shape extents ----
    float     radius;       // Used when type == Circle
    float     halfHeight;   // Used when type == Triangle or Square
    glm::vec3 rampEnd;      // Second endpoint of the segment when type == Ramp

    // ---- Freeze State ----
    bool  isFrozen = false;
    float originalMass = 1.0f; // Remembers mass configuration while frozen

    // ---- Constructor: safe zero/default state ----
    RigidBody()
        : type(ShapeType::Triangle),
          position(0.0f), velocity(0.0f), gravity(0.0f, -9.81f, 0.0f),
          angle(0.0f), angularVelocity(0.0f), torque(0.0f),
          mass(0.0f), invMass(0.0f), inertia(0.0f), invInertia(0.0f),
          dragCoefficient(0.15f), restitution(0.7f), friction(0.3f),
          radius(0.0f), halfHeight(0.5f), rampEnd(0.0f),
          isFrozen(false), originalMass(1.0f) {}

    // -------------------------------------------------------------------------
    // configure()
    //   Resets and fully initialises this body for a given shape preset.
    //   Pass mass == 0 to create an infinite-mass (static) body.
    // -------------------------------------------------------------------------
    void configure(ShapeType t, glm::vec3 startPos,
                   float m, float size,
                   float drag, float bounce, float surfFric)
    {
        type     = t;
        position = startPos;
        velocity = glm::vec3(0.0f);
        gravity  = glm::vec3(0.0f, -9.81f, 0.0f);

        angle           = 0.0f;
        angularVelocity = 0.0f;
        torque          = 0.0f;

        mass            = m;
        invMass         = (mass > 0.0f) ? 1.0f / mass : 0.0f;
        dragCoefficient = drag;
        restitution     = bounce;
        friction        = surfFric;

        // Moment of inertia differs by geometry
        switch (type) {
            case ShapeType::Circle:
                radius  = size;
                // Solid disk: I = ½mr²
                inertia = 0.5f * mass * (radius * radius);
                break;

            case ShapeType::Square:
            case ShapeType::Triangle: {
                halfHeight    = size;
                float side    = halfHeight * 2.0f;
                // Solid rectangular plate: I = (1/12)m(w²+h²)
                inertia       = (1.0f / 12.0f) * mass * (side * side + side * side);
                break;
            }

            case ShapeType::Ramp:
                halfHeight = 0.0f;
                inertia    = 0.0f; // Static segments never rotate
                break;
        }

        invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
    }

    // -------------------------------------------------------------------------
    // Freeze / Unfreeze System
    // -------------------------------------------------------------------------
    void freeze() {
        if (isFrozen) return;
        isFrozen = true;
        originalMass = mass; // Stash the current simulated weight
        
        mass       = 0.0f;
        invMass    = 0.0f;
        invInertia = 0.0f;
        
        velocity        = glm::vec3(0.0f);
        angularVelocity = 0.0f;
        torque          = 0.0f;
    }

    void unfreeze() {
        if (!isFrozen) return;
        isFrozen = false;
        
        // Recalculate tensor configurations safely using the original mass properties
        configure(type, position, originalMass, (type == ShapeType::Circle ? radius : halfHeight), 
                  dragCoefficient, restitution, friction);
    }

    // -------------------------------------------------------------------------
    // update()
    //   Semi-implicit Euler integration for one fixed timestep dt.
    //   Static bodies (invMass == 0) are skipped entirely.
    // -------------------------------------------------------------------------
    void update(float dt) {
        if (invMass == 0.0f) return; // Infinite-mass / static body — nothing to integrate

        // -- Linear --
        // Net force = gravity + drag (simple linear drag model: F_drag = -c * v)
        glm::vec3 netForce    = mass * gravity + (-dragCoefficient * velocity);
        glm::vec3 acceleration = netForce * invMass;

        velocity.x += acceleration.x * dt;
        velocity.y += acceleration.y * dt;
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        // -- Rotational --
        // Alpha = torque / inertia
        float angularAccel  = torque * invInertia;
        angularVelocity    += angularAccel * dt;
        angle              += angularVelocity * dt;

        torque = 0.0f; // Reset accumulator for next frame
    }
};

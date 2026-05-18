#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <cmath>
#include "physics.h"

// =============================================================================
// Material Science Presets
// =============================================================================
// All values are SI-coherent (metres, kg, seconds).
//
//  shearModulusG    : Shear modulus in GPa.  Wahl spring stiffness formula.
//  ultimateStress   : Ultimate tensile strength in MPa.  Physically derives the
//                     rod breaking force from cross-section area — correct
//                     failure criterion instead of an arbitrary impulse cap.
//  maxElasticStrain : Yield strain (dimensionless). Spring takes permanent set
//                     beyond this.
//  breakingStrain   : Fracture strain.  Spring snaps if exceeded.
//  youngModulus     : Young's modulus in GPa.  Reserved for rod axial stiffness.
//
// Sources: Shigley's Mechanical Engineering Design, Machinery's Handbook,
//          ASM International datasheets.
// =============================================================================
struct MaterialPreset {
    std::string name;
    float shearModulusG;      // GPa
    float ultimateStress;     // MPa
    float maxElasticStrain;   // dimensionless
    float breakingStrain;     // dimensionless
    float youngModulus;       // GPa
};

const std::vector<MaterialPreset> MATERIAL_DATABASE = {
    // name                      G(GPa)  UTS(MPa) e_yield e_break  E(GPa)
    { "Music Wire (High-Carbon)", 79.3f,  2200.0f, 0.0080f, 0.050f, 207.0f },
    { "Chrome Silicon",           75.0f,  1900.0f, 0.0075f, 0.040f, 200.0f },
    { "Oil-Tempered Carbon",      78.0f,  1700.0f, 0.0065f, 0.045f, 203.0f },
    { "Stainless Steel (302)",    69.0f,  1300.0f, 0.0050f, 0.300f, 193.0f },
    { "Beryllium Copper",         50.0f,  1200.0f, 0.0070f, 0.040f, 128.0f },
    { "Titanium Alloy",           41.0f,  1100.0f, 0.0095f, 0.100f, 114.0f },
    { "Phosphor Bronze",          40.0f,   700.0f, 0.0045f, 0.150f, 110.0f },
    { "Inconel X-750",            75.8f,  1250.0f, 0.0055f, 0.200f, 207.0f },
    { "Nylon / Polyamide",         1.2f,    80.0f, 0.0250f, 0.500f,   3.0f },
    { "Structural/Mild Steel",    79.3f,   400.0f, 0.0012f, 0.250f, 200.0f }
};

// =============================================================================
// SpringJoint
// =============================================================================
// Physics method:
//   - Stiffness from Wahl coil-spring formula: k = (G*d^4)/(8*D^3*Na)
//   - Each sub-step applies spring force*sub_dt as a velocity impulse (semi-
//     implicit Euler), plus a damping velocity impulse (no extra dt factor).
//   - Breaking evaluated once per full tick on strain, not per-sub-step, so
//     fast transients don't cause premature snapping due to numerical overshoot.
//   - Plastic permanent set: rest length creeps when strain exceeds yield limit.
// =============================================================================
struct SpringJoint {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    glm::vec3 localAnchorA{0.0f};
    glm::vec3 localAnchorB{0.0f};

    int   materialIndex  = 0;
    float wireDiameter_d = 0.15f;   // m
    float coilDiameter_D = 1.20f;   // m
    float activeCoils_Na = 10.0f;

    float restLength_L0  = 1.0f;
    float damping        = 0.8f;    // N·s/m  (light structural damping)
    bool  isBroken       = false;

    // Telemetry — updated each sub-step, readable by ImGui
    float currentStrain  = 0.0f;
    float calculatedK    = 0.0f;

    float computeStiffness() const {
        // Wahl formula: k = (G * d^4) / (8 * D^3 * Na)
        // G in GPa, d/D in metres → k in GPa·m (= 1e9 N/m).
        // Our scene is metre-scale so we want N/m, not GN/m.
        // Divide by 1e9 to convert GPa → Pa·m unit chain → N/m.
        // (Equivalently: store G in Pa, compute normally.)
        float G  = MATERIAL_DATABASE[materialIndex].shearModulusG * 1e9f; // Pa
        float d4 = std::pow(wireDiameter_d, 4.0f);
        float D3 = std::pow(coilDiameter_D, 3.0f);
        return std::max((G * d4) / (8.0f * D3 * activeCoils_Na), 0.01f);
    }

    // One sub-step. sub_dt = FIXED_DT / SPRING_SUBSTEPS.
    void updateAndApplyForces(float sub_dt) {
        if (isBroken || !bodyA || !bodyB) return;

        calculatedK = computeStiffness();

        // w=0: rotating an offset vector, not transforming a point
        glm::mat4 rotA = glm::rotate(glm::mat4(1.0f), bodyA->angle, glm::vec3(0,0,1));
        glm::mat4 rotB = glm::rotate(glm::mat4(1.0f), bodyB->angle, glm::vec3(0,0,1));
        glm::vec3 wA   = bodyA->position + glm::vec3(rotA * glm::vec4(localAnchorA, 0.0f));
        glm::vec3 wB   = bodyB->position + glm::vec3(rotB * glm::vec4(localAnchorB, 0.0f));

        glm::vec3 delta = wB - wA;
        float     L     = glm::length(delta);
        if (L < 1e-5f) return;
        glm::vec3 n = delta / L;

        currentStrain = (L - restLength_L0) / restLength_L0;

        // Plastic permanent set
        const auto& mat = MATERIAL_DATABASE[materialIndex];
        if (std::abs(currentStrain) > mat.maxElasticStrain) {
            float sign  = (currentStrain > 0.0f) ? 1.0f : -1.0f;
            float newL0 = L - sign * mat.maxElasticStrain * restLength_L0;
            if (newL0 > 0.01f) {
                restLength_L0 = newL0;
                currentStrain = (L - restLength_L0) / restLength_L0;
            }
        }

        // Spring impulse (semi-implicit: force*sub_dt)
        float springImpulse = (L - restLength_L0) * calculatedK * sub_dt;

        // Damping impulse (velocity-level, no extra dt)
        glm::vec3 rA   = wA - bodyA->position;
        glm::vec3 rB   = wB - bodyB->position;
        glm::vec3 velA = bodyA->velocity + glm::vec3(-bodyA->angularVelocity * rA.y,
                                                      bodyA->angularVelocity * rA.x, 0.0f);
        glm::vec3 velB = bodyB->velocity + glm::vec3(-bodyB->angularVelocity * rB.y,
                                                      bodyB->angularVelocity * rB.x, 0.0f);
        float relVel      = glm::dot(velB - velA, n);
        float dampImpulse = -relVel * damping;

        glm::vec3 J = n * (springImpulse + dampImpulse);

        if (bodyA->invMass    > 0.0f) bodyA->velocity        += J * bodyA->invMass;
        if (bodyB->invMass    > 0.0f) bodyB->velocity        -= J * bodyB->invMass;
        if (bodyA->invInertia > 0.0f) bodyA->angularVelocity += (rA.x*J.y - rA.y*J.x) * bodyA->invInertia;
        if (bodyB->invInertia > 0.0f) bodyB->angularVelocity -= (rB.x*J.y - rB.y*J.x) * bodyB->invInertia;
    }

    // Call once per full physics tick after all sub-steps.
    // Strain-based failure matches real material behaviour.
    void checkBreaking() {
        if (isBroken) return;
        if (std::abs(currentStrain) > MATERIAL_DATABASE[materialIndex].breakingStrain)
            isBroken = true;
    }
};

// =============================================================================
// DistanceRod
// =============================================================================
// Physics method:
//   - Sequential impulses iterated VELOCITY_ITERATIONS times per tick
//     (Box2D/Bullet approach) for convergence without requiring tiny dt.
//   - Breaking via axial stress: sigma = |lambda/dt| / crossSectionArea.
//     Breaks when sigma > UTS. This is the physically correct criterion.
//   - Position correction via a separate projection pass (not Baumgarte).
//     Projection moves bodies directly in position space with zero velocity
//     change, so it injects no energy and produces no frame-1 impulse spike.
// =============================================================================
struct DistanceRod {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    glm::vec3 localAnchorA{0.0f};
    glm::vec3 localAnchorB{0.0f};
    float targetLength    = 1.0f;

    int   materialIndex   = 0;
    bool  isBroken        = false;

    // Rod cross-section area in m². Drives stress-based breaking.
    // Default: 10 mm diameter → A = pi*(0.005)^2 = 7.854e-5 m^2
    float crossSectionArea = 7.854e-5f;

    // Telemetry
    float lastAxialStress  = 0.0f;  // Pa

    // One velocity-constraint iteration. Call this VELOCITY_ITERATIONS times per tick.
    void resolveVelocity(float dt) {
        if (isBroken || !bodyA || !bodyB) return;

        glm::mat4 rotA = glm::rotate(glm::mat4(1.0f), bodyA->angle, glm::vec3(0,0,1));
        glm::mat4 rotB = glm::rotate(glm::mat4(1.0f), bodyB->angle, glm::vec3(0,0,1));
        glm::vec3 wA   = bodyA->position + glm::vec3(rotA * glm::vec4(localAnchorA, 0.0f));
        glm::vec3 wB   = bodyB->position + glm::vec3(rotB * glm::vec4(localAnchorB, 0.0f));

        glm::vec3 delta = wB - wA;
        float     L     = glm::length(delta);
        if (L < 1e-5f) return;
        glm::vec3 n = delta / L;

        glm::vec3 rA   = wA - bodyA->position;
        glm::vec3 rB   = wB - bodyB->position;
        glm::vec3 velA = bodyA->velocity + glm::vec3(-bodyA->angularVelocity * rA.y,
                                                      bodyA->angularVelocity * rA.x, 0.0f);
        glm::vec3 velB = bodyB->velocity + glm::vec3(-bodyB->angularVelocity * rB.y,
                                                      bodyB->angularVelocity * rB.x, 0.0f);

        float velAlongN = glm::dot(velB - velA, n);
        float rnA       = rA.x * n.y - rA.y * n.x;
        float rnB       = rB.x * n.y - rB.y * n.x;
        float effMass   = bodyA->invMass  + bodyB->invMass
                        + rnA*rnA * bodyA->invInertia
                        + rnB*rnB * bodyB->invInertia;
        if (effMass < 1e-10f) return;

        float lambda = -velAlongN / effMass;

        // Stress = |force| / area = |lambda / dt| / area
        float axialForce  = std::abs(lambda) / dt;
        lastAxialStress   = axialForce / crossSectionArea;
        float UTS_Pa      = MATERIAL_DATABASE[materialIndex].ultimateStress * 1e6f;
        if (lastAxialStress > UTS_Pa) {
            isBroken = true;
            return;
        }

        glm::vec3 J = n * lambda;
        if (bodyA->invMass    > 0.0f) bodyA->velocity        -= J * bodyA->invMass;
        if (bodyB->invMass    > 0.0f) bodyB->velocity        += J * bodyB->invMass;
        if (bodyA->invInertia > 0.0f) bodyA->angularVelocity -= (rA.x*J.y - rA.y*J.x) * bodyA->invInertia;
        if (bodyB->invInertia > 0.0f) bodyB->angularVelocity += (rB.x*J.y - rB.y*J.x) * bodyB->invInertia;
    }

    // Position projection — corrects drift with zero energy injection.
    // No Baumgarte: this moves positions directly, so no frame-1 bias spike.
    void resolvePosition() {
        if (isBroken || !bodyA || !bodyB) return;

        glm::mat4 rotA = glm::rotate(glm::mat4(1.0f), bodyA->angle, glm::vec3(0,0,1));
        glm::mat4 rotB = glm::rotate(glm::mat4(1.0f), bodyB->angle, glm::vec3(0,0,1));
        glm::vec3 wA   = bodyA->position + glm::vec3(rotA * glm::vec4(localAnchorA, 0.0f));
        glm::vec3 wB   = bodyB->position + glm::vec3(rotB * glm::vec4(localAnchorB, 0.0f));

        float     L     = glm::length(wB - wA);
        float     error = L - targetLength;

        constexpr float SLOP   = 0.005f;  // 5 mm tolerance
        constexpr float FACTOR = 0.4f;    // correction fraction per tick
        if (std::abs(error) <= SLOP) return;

        glm::vec3 n = (wB - wA) / L;
        float correction = FACTOR * (error - glm::sign(error) * SLOP);

        glm::vec3 rA  = wA - bodyA->position;
        glm::vec3 rB  = wB - bodyB->position;
        float rnA     = rA.x * n.y - rA.y * n.x;
        float rnB     = rB.x * n.y - rB.y * n.x;
        float effMass = bodyA->invMass  + bodyB->invMass
                      + rnA*rnA * bodyA->invInertia
                      + rnB*rnB * bodyB->invInertia;
        if (effMass < 1e-10f) return;

        glm::vec3 posJ = n * (correction / effMass);

        // Pure position/angle correction — velocities untouched
        if (bodyA->invMass    > 0.0f) bodyA->position += posJ * bodyA->invMass;
        if (bodyB->invMass    > 0.0f) bodyB->position -= posJ * bodyB->invMass;
        if (bodyA->invInertia > 0.0f) bodyA->angle    += (rA.x*posJ.y - rA.y*posJ.x) * bodyA->invInertia;
        if (bodyB->invInertia > 0.0f) bodyB->angle    -= (rB.x*posJ.y - rB.y*posJ.x) * bodyB->invInertia;
    }
};

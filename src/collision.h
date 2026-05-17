#pragma once
#include <glm/glm.hpp>
#include "physics.h"

// =============================================================================
// resolveCircleVsRamp
//   Handles collision detection and response between a circular body and a
//   static line-segment ramp. Uses closest-point projection to find penetration,
//   then applies a normal impulse + friction torque to the circle.
// =============================================================================
inline void resolveCircleVsRamp(RigidBody& circle, const RigidBody& ramp) {
    if (circle.type != ShapeType::Circle || ramp.type != ShapeType::Ramp) return;

    // Build ramp basis vectors
    glm::vec3 rampVector = ramp.rampEnd - ramp.position;
    glm::vec3 rampUnit   = glm::normalize(rampVector);

    // Normal always points "upward" relative to the ramp direction (left-hand perp)
    glm::vec3 rampNormal = glm::normalize(glm::vec3(-rampVector.y, rampVector.x, 0.0f));

    // Project circle centre onto the ramp segment and clamp to endpoints
    float projection  = glm::dot(circle.position - ramp.position, rampUnit);
    projection        = glm::clamp(projection, 0.0f, glm::length(rampVector));
    glm::vec3 closest = ramp.position + rampUnit * projection;

    // Distance from circle centre to the closest point on the segment
    float distance = glm::length(circle.position - closest);
    if (distance >= circle.radius) return; // No penetration — early out

    // ---- Positional correction ----
    float penetrationDepth = circle.radius - distance;
    circle.position += rampNormal * penetrationDepth;

    // ---- Velocity response ----
    float relativeNormalVel = glm::dot(circle.velocity, rampNormal);
    if (relativeNormalVel >= 0.0f) return; // Moving away — skip impulse

    float combinedBounce   = circle.restitution * ramp.restitution;
    float impulseMagnitude = -(1.0f + combinedBounce) * relativeNormalVel;
    circle.velocity       += rampNormal * impulseMagnitude;

    // ---- Friction + rolling torque along the ramp surface ----
    glm::vec3 tangent          = circle.velocity - rampNormal * glm::dot(circle.velocity, rampNormal);
    float     tangentLen       = glm::length(tangent);
    if (tangentLen > 0.001f) {
        tangent                   /= tangentLen; // safe normalise
        float relativeTangentVel   = glm::dot(circle.velocity, tangent);
        float frictionImpulse      = -relativeTangentVel * (circle.friction * ramp.friction);
        circle.velocity           += tangent * frictionImpulse;
        circle.torque             += circle.radius * frictionImpulse * 10.0f;
    }
}

// =============================================================================
// resolveWorldBoundaries
//   Keeps a body inside the world rectangle [minX, maxX] x [minY, maxY].
//   Uses a simple reflection impulse scaled by restitution, and damps
//   angularVelocity by the friction coefficient on each wall hit.
// =============================================================================
inline void resolveWorldBoundaries(RigidBody& body, float minX, float maxX, float minY, float maxY) {
    if (body.type == ShapeType::Ramp) return;

    // Use radius for circles, halfHeight for box-like shapes
    float extent = (body.type == ShapeType::Circle) ? body.radius : body.halfHeight;

    // ---- Floor ----
    if (body.position.y - extent < minY) {
        body.position.y      = minY + extent;
        body.velocity.y      = -body.velocity.y * body.restitution;
        body.angularVelocity *= (1.0f - body.friction);
    }

    // ---- Ceiling ----
    if (body.position.y + extent > maxY) {
        body.position.y      = maxY - extent;
        body.velocity.y      = -body.velocity.y * body.restitution;
        body.angularVelocity *= (1.0f - body.friction);
    }

    // ---- Left wall ----
    if (body.position.x - extent < minX) {
        body.position.x      = minX + extent;
        body.velocity.x      = -body.velocity.x * body.restitution;
        body.angularVelocity *= (1.0f - body.friction);
    }

    // ---- Right wall ----
    // BUG FIX: was "< maxX" (always true for interior objects), now correctly "> maxX"
    if (body.position.x + extent > maxX) {
        body.position.x      = maxX - extent;
        body.velocity.x      = -body.velocity.x * body.restitution;
        body.angularVelocity *= (1.0f - body.friction);
    }
}

// =============================================================================
// resolveObjectCollisions
//   Broad-phase overlap test using summed bounding radii, followed by an
//   impulse-based velocity response. Works for any combination of circle/box
//   since both use a single bounding radius for collision extent.
// =============================================================================
inline void resolveObjectCollisions(RigidBody& a, RigidBody& b) {
    glm::vec3 delta    = b.position - a.position;
    float     distance = glm::length(delta);

    float extentA     = (a.type == ShapeType::Circle) ? a.radius : a.halfHeight;
    float extentB     = (b.type == ShapeType::Circle) ? b.radius : b.halfHeight;
    float totalExtent = extentA + extentB;

    if (distance >= totalExtent) return; // No overlap — early out
    if (distance == 0.0f)        return; // Perfectly overlapping — skip to avoid NaN

    float     penetration     = totalExtent - distance;
    glm::vec3 collisionNormal = delta / distance; // pre-divided == normalise

    // ---- Positional correction (push apart proportional to inverse mass) ----
    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass == 0.0f) return; // Both static — nothing to do

    glm::vec3 correction = collisionNormal * (penetration / totalInvMass);
    a.position          -= correction * a.invMass;
    b.position          += correction * b.invMass;

    // ---- Impulse response (only if objects are approaching each other) ----
    glm::vec3 relativeVelocity = b.velocity - a.velocity;
    float     velAlongNormal   = glm::dot(relativeVelocity, collisionNormal);
    if (velAlongNormal >= 0.0f) return; // Already separating — skip

    float combinedRestitution = glm::min(a.restitution, b.restitution);
    float impulseScalar       = -(1.0f + combinedRestitution) * velAlongNormal / totalInvMass;

    a.velocity -= collisionNormal * (impulseScalar * a.invMass);
    b.velocity += collisionNormal * (impulseScalar * b.invMass);
}

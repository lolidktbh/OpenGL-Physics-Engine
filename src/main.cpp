#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>

#include "shader.h"
#include "mesh.h"
#include "physics.h"
#include "collision.h"

// =============================================================================
// World constants
//   WORLD_HEIGHT is fixed; WORLD_WIDTH is derived dynamically from the window
//   aspect ratio so the projection always fills the viewport without distortion.
// =============================================================================
constexpr int   WINDOW_WIDTH  = 800;
constexpr int   WINDOW_HEIGHT = 600;
constexpr float WORLD_HEIGHT  = 12.0f;

// =============================================================================
// Global simulation state
// =============================================================================
std::vector<RigidBody> sceneObjects;
RigidBody              staticRamp;
bool                   rampActive = false; // True once the player spawns a ramp with R

// =============================================================================
// Vertex data (unit-space; the model matrix applies scale per object)
// =============================================================================
float triangleVerts[] = {
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.0f,  0.5f, 0.0f
};

// Two-triangle quad; used as the base mesh for squares AND circles
float squareVerts[] = {
    -0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f,  -0.5f, -0.5f, 0.0f,
    -0.5f,  0.5f, 0.0f,   0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f
};

// =============================================================================
// getMouseWorldPos
//   Converts the raw GLFW cursor position (top-left origin, pixels) into the
//   orthographic world space used by the physics simulation.
// =============================================================================
glm::vec3 getMouseWorldPos(GLFWwindow* window) {
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    int   winW, winH;
    glfwGetWindowSize(window, &winW, &winH);

    // Match the dynamic projection: width scales with aspect ratio
    float aspect           = (float)winW / (float)winH;
    float dynamicWorldWidth = WORLD_HEIGHT * aspect;

    float worldX = ((float)mouseX / (float)winW) * dynamicWorldWidth;
    float worldY = (1.0f - ((float)mouseY / (float)winH)) * WORLD_HEIGHT;

    return glm::vec3(worldX, worldY, 0.0f);
}

// =============================================================================
// isPointInsideBody
//   Simple point-in-shape test used for click-to-drag picking.
//   Circles use exact radial distance; everything else uses an AABB.
// =============================================================================
bool isPointInsideBody(const glm::vec3& point, const RigidBody& body) {
    if (body.type == ShapeType::Circle) {
        return glm::distance(point, body.position) <= body.radius;
    }
    // AABB using halfHeight as the uniform half-extent for box/triangle
    return (point.x >= body.position.x - body.halfHeight &&
            point.x <= body.position.x + body.halfHeight &&
            point.y >= body.position.y - body.halfHeight &&
            point.y <= body.position.y + body.halfHeight);
}

// =============================================================================
// keyCallback
//   Spawns exactly one object per key-press event (GLFW_PRESS fires once).
//   Spawn positions are randomised along a horizontal band near the top.
// =============================================================================
void keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;

    switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, true);
            break;

        case GLFW_KEY_T: {
            RigidBody tri;
            float rx = 4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            // mass=0.5, halfHeight=0.5, drag=0.2, bounce=0.6, friction=0.2
            tri.configure(ShapeType::Triangle, glm::vec3(rx, 10.0f, 0.0f), 0.5f, 0.5f, 0.2f, 0.6f, 0.2f);
            sceneObjects.push_back(tri);
            break;
        }

        case GLFW_KEY_B: {
            RigidBody box;
            float rx = 4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            // mass=15, halfHeight=0.5, drag=0.05, bounce=0.4, friction=0.4
            box.configure(ShapeType::Square, glm::vec3(rx, 10.0f, 0.0f), 15.0f, 0.5f, 0.05f, 0.4f, 0.4f);
            sceneObjects.push_back(box);
            break;
        }

        case GLFW_KEY_C: {
            RigidBody circ;
            float rx = 4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            // mass=2.5, radius=0.5, drag=0.05, bounce=0.7, friction=0.5
            circ.configure(ShapeType::Circle, glm::vec3(rx, 11.0f, 0.0f), 2.5f, 0.5f, 0.05f, 0.7f, 0.5f);
            sceneObjects.push_back(circ);
            break;
        }

        case GLFW_KEY_R:
            // Configure ramp as an infinite-mass static segment (mass == 0 → invMass == 0)
            staticRamp.configure(ShapeType::Ramp, glm::vec3(1.0f, 7.0f, 0.0f), 0.0f, 0.0f, 0.0f, 0.2f, 0.6f);
            staticRamp.rampEnd = glm::vec3(13.0f, 2.0f, 0.0f);
            rampActive         = true;
            break;

        default: break;
    }
}

// =============================================================================
// handleMouseDragging
//   Left-click picks the top-most shape under the cursor; holding and moving
//   the mouse drags it. Velocity is calculated from movement over time so that
//   releasing the mouse "throws" the object.
// =============================================================================
void handleMouseDragging(GLFWwindow* window,
                         RigidBody*& draggedBody,
                         bool&       isDragging,
                         glm::vec3&  dragOffset,
                         float       dt) // Track frame delta-time for velocity calculations
{
    glm::vec3 mouseWorld = getMouseWorldPos(window);
    int       mouseState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);

    if (mouseState == GLFW_PRESS) {
        if (!isDragging) {
            // Iterate back-to-front so topmost (last-drawn) shape wins
            for (int i = (int)sceneObjects.size() - 1; i >= 0; --i) {
                if (isPointInsideBody(mouseWorld, sceneObjects[i])) {
                    isDragging  = true;
                    draggedBody = &sceneObjects[i];
                    dragOffset  = sceneObjects[i].position - mouseWorld;
                    break;
                }
            }
        }
        if (isDragging && draggedBody) {
            glm::vec3 newPosition = mouseWorld + dragOffset;

            // Calculate throwing velocity dynamically based on movement speed over time
            if (dt > 0.001f) {
                draggedBody->velocity = (newPosition - draggedBody->position) / dt;
            }

            draggedBody->position        = newPosition;
            draggedBody->angularVelocity = 0.0f; // Keep orientation stable during drag
        }
    } else {
        // Button released — drop the body back into simulation (retains calculated throw momentum)
        isDragging  = false;
        draggedBody = nullptr;
    }
}

// =============================================================================
// framebuffer_size_callback
//   Keeps the OpenGL viewport in sync with the OS window size.
// =============================================================================
void framebuffer_size_callback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

// =============================================================================
// buildTransform
//   Constructs the model matrix for a body: translate → rotate → scale.
// =============================================================================
glm::mat4 buildTransform(const RigidBody& body) {
    glm::mat4 m = glm::translate(glm::mat4(1.0f), body.position);
    m           = glm::rotate(m, body.angle, glm::vec3(0.0f, 0.0f, 1.0f));

    float s = (body.type == ShapeType::Circle)
              ? body.radius * 2.0f         // Diameter so the unit quad spans the full circle
              : body.halfHeight * 2.0f;    // Full side length for triangles and boxes

    m = glm::scale(m, glm::vec3(s, s, 1.0f));
    return m;
}

// =============================================================================
// main
// =============================================================================
int main() {
    srand(static_cast<unsigned int>(time(nullptr)));

    // ---- Window + context setup ----
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
                                          "OpenGL Physics Engine", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, keyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    // ---- Shaders & meshes ----
    Shader standardShader("shader.vert", "shader.frag");
    Shader circleShader  ("shader.vert", "circle.frag");
    Mesh   triangleMesh  (triangleVerts, sizeof(triangleVerts), 3);
    Mesh   squareMesh    (squareVerts,   sizeof(squareVerts),   6);

    // ---- Drag state ----
    RigidBody* draggedBody = nullptr;
    bool       isDragging  = false;
    glm::vec3  dragOffset(0.0f);

    // ---- Fixed timestep accumulator ----
    constexpr float FIXED_DT  = 1.0f / 60.0f;
    float           lastFrame = 0.0f;
    float           accumulator = 0.0f;

    // ---- Projection (rebuilt each frame to handle window resize) ----
    glm::mat4 projection(1.0f);

    // ==========================================================================
    // Main loop
    // ==========================================================================
    while (!glfwWindowShouldClose(window)) {

        // ---- Frame timing ----
        float currentFrame = (float)glfwGetTime();
        float frameTime    = glm::min(currentFrame - lastFrame, 0.25f); // clamp spiral of death
        lastFrame          = currentFrame;
        accumulator       += frameTime;

        // ---- Dynamic world width (recalculated once per frame for resize support) ----
        int   winW, winH;
        glfwGetWindowSize(window, &winW, &winH);
        float aspect           = (float)winW / (float)winH;
        float dynamicWorldWidth = WORLD_HEIGHT * aspect;

        // Rebuild projection to match current aspect ratio
        projection = glm::ortho(0.0f, dynamicWorldWidth, 0.0f, WORLD_HEIGHT, -1.0f, 1.0f);

        // ---- Input ----
        handleMouseDragging(window, draggedBody, isDragging, dragOffset, frameTime);

        // ---- Fixed-step physics ----
        while (accumulator >= FIXED_DT) {

            // Integrate all non-dragged bodies
            for (auto& obj : sceneObjects) {
                if (&obj == draggedBody) continue;
                obj.update(FIXED_DT);
            }

            // Collision resolution
            for (size_t i = 0; i < sceneObjects.size(); ++i) {
                // Ramp collisions (circles only, guarded inside the function)
                if (rampActive)
                    resolveCircleVsRamp(sceneObjects[i], staticRamp);

                // World boundary clamp
                resolveWorldBoundaries(sceneObjects[i], 0.0f, dynamicWorldWidth, 0.0f, WORLD_HEIGHT);

                // Object-to-object collisions (upper triangle of the pair matrix)
                for (size_t j = i + 1; j < sceneObjects.size(); ++j)
                    resolveObjectCollisions(sceneObjects[i], sceneObjects[j]);
            }

            accumulator -= FIXED_DT;
        }

        // ---- Render ----
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // -- Static ramp --
        if (rampActive) {
            glm::vec3 dir      = staticRamp.rampEnd - staticRamp.position;
            float     rampAngle = atan2(dir.y, dir.x);

            glm::mat4 rampTransform = glm::translate(glm::mat4(1.0f), staticRamp.position);
            rampTransform           = glm::rotate(rampTransform, rampAngle, glm::vec3(0.0f, 0.0f, 1.0f));
            // Scale X to ramp length, Y to a thin visible strip
            rampTransform           = glm::scale(rampTransform, glm::vec3(glm::length(dir), 0.1f, 1.0f));

            standardShader.use();
            standardShader.setMat4("projection", projection);
            standardShader.setMat4("transform",  rampTransform);
            squareMesh.draw();
        }

        // -- Dynamic bodies --
        for (const auto& obj : sceneObjects) {
            glm::mat4 transform = buildTransform(obj);

            if (obj.type == ShapeType::Circle) {
                circleShader.use();
                circleShader.setMat4("projection", projection);
                circleShader.setMat4("transform",  transform);
                squareMesh.draw(); // Circle SDF is applied in circle.frag
            } else {
                // Triangle and Square share the standard (flat colour) shader
                standardShader.use();
                standardShader.setMat4("projection", projection);
                standardShader.setMat4("transform",  transform);

                if (obj.type == ShapeType::Triangle)
                    triangleMesh.draw();
                else
                    squareMesh.draw();
            }
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

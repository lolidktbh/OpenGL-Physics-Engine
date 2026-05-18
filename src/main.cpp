#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <ctime>

#include "shader.h"
#include "mesh.h"
#include "physics.h"
#include "collision.h"
#include "joints.h"

// Include Dear ImGui headers from your subdirectory folder
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// =============================================================================
// World constants
// =============================================================================
constexpr int   WINDOW_WIDTH  = 1024; // Widened slightly to fit sidebar menus comfortably
constexpr int   WINDOW_HEIGHT = 768;
constexpr float WORLD_HEIGHT  = 12.0f;

// =============================================================================
// Global simulation state
// =============================================================================
// ARCHITECTURE FIX: sceneObjects uses std::deque instead of std::vector.
// std::vector invalidates ALL pointers/iterators on any push_back that causes
// reallocation. Since bodyA/bodyB in joints, draggedBody, selectedInspectorBody,
// and linkFirstSelection all store raw RigidBody* into this container, any
// reallocation (from spawning or chain creation) silently turns every stored
// pointer into a dangling pointer — causing the "malloc: invalid size (unsorted)"
// heap corruption crash. std::deque guarantees pointer stability on push_back
// with no reservation needed, solving this permanently for all call sites.
std::deque<RigidBody>  sceneObjects;
RigidBody              staticRamp;
bool                   rampActive = false;

enum class LinkMode { NONE, SPRING, ROD };
LinkMode currentLinkMode = LinkMode::NONE;
RigidBody* linkFirstSelection = nullptr;

std::vector<SpringJoint> globalSprings;
std::vector<DistanceRod> globalRods;
int selectedMaterialIndex = 0;

// Global Environment Tweaks controlled by ImGui
glm::vec3 globalGravity(0.0f, -9.81f, 0.0f);
float     simulationTimeScale = 1.0f;

// Spawner Settings Template (Tweak in UI before spawning)
int   uiSelectedShapeType = 2; // Default: Circle
float uiSpawnMass         = 5.0f;
float uiSpawnSize         = 0.5f;
float uiSpawnRestitution  = 0.6f;
float uiSpawnFriction     = 0.3f;
float uiSpawnDrag         = 0.05f;

// Pointer to an item currently selected/clicked by user for inspector view
RigidBody* selectedInspectorBody = nullptr;

// =============================================================================
// createChain
// =============================================================================
void createChain(glm::vec3 start, glm::vec3 end, int totalSegments, bool useSpring) {
    if (totalSegments < 2) return;

    glm::vec3 step = (end - start) / static_cast<float>(totalSegments);
    RigidBody* previousBody = nullptr;

    // Pointer stability is guaranteed by std::deque — no reserve() needed.
    for (int i = 0; i <= totalSegments; ++i) {
        RigidBody node;
        // Make endpoints static anchors (infinite mass) if it's the start or end of a hanging bridge
        bool isAnchor = (i == 0 || i == totalSegments);

        //Define configure parameters for chain segments
        glm::vec3 nodePos = start + step * static_cast<float>(i);
        float nodeMass    = isAnchor ? 0.0f : 2.0f;
        float nodeSize    = 0.25f;
        float nodeDrag    = 0.05f;
        float nodeRest    = 0.3f;
        float nodeFrict   = 0.4f;
        
        //Arguments for physics.h
        node.configure(ShapeType::Square, nodePos, nodeMass, nodeSize, nodeDrag, nodeRest, nodeFrict);
        node.velocity = glm::vec3(0.0f);
        node.angle = 0.0f;
        node.angularVelocity = 0.0f;
        node.gravity = globalGravity;

        sceneObjects.push_back(node);
        RigidBody* currentBody = &sceneObjects.back();

        if (previousBody != nullptr) {
            float dist = glm::distance(currentBody->position, previousBody->position);
            if (useSpring) {
                SpringJoint s;
                s.bodyA = previousBody;
                s.bodyB = currentBody;
                s.restLength_L0 = dist;
                s.materialIndex = selectedMaterialIndex;
                globalSprings.push_back(s);
            } else {
                DistanceRod r;
                r.bodyA = previousBody;
                r.bodyB = currentBody;
                r.targetLength = dist;
                r.materialIndex = selectedMaterialIndex;
                globalRods.push_back(r);
            }
        }
        previousBody = currentBody;
    }
}

// =============================================================================
// Vertex data
// =============================================================================
float triangleVerts[] = {
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.0f,  0.5f, 0.0f
};

float squareVerts[] = {
    -0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f,  -0.5f, -0.5f, 0.0f,
    -0.5f,  0.5f, 0.0f,   0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f
};

// =============================================================================
// getMouseWorldPos
// =============================================================================
glm::vec3 getMouseWorldPos(GLFWwindow* window) {
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    int winW, winH;
    glfwGetWindowSize(window, &winW, &winH);

    float aspect           = (float)winW / (float)winH;
    float dynamicWorldWidth = WORLD_HEIGHT * aspect;

    float worldX = ((float)mouseX / (float)winW) * dynamicWorldWidth;
    float worldY = (1.0f - ((float)mouseY / (float)winH)) * WORLD_HEIGHT;

    return glm::vec3(worldX, worldY, 0.0f);
}

// =============================================================================
// isPointInsideBody
// =============================================================================
bool isPointInsideBody(const glm::vec3& point, const RigidBody& body) {
    if (body.type == ShapeType::Circle) {
        return glm::distance(point, body.position) <= body.radius;
    }
    float extent = body.halfHeight;
    return (point.x >= body.position.x - extent &&
            point.x <= body.position.x + extent &&
            point.y >= body.position.y - extent &&
            point.y <= body.position.y + extent);
}

// =============================================================================
// handleMouseDragging & Right-Click Linker State Machine
// =============================================================================
void handleMouseDragging(GLFWwindow* window,
                         RigidBody*& draggedBody,
                         bool&       isDragging,
                         glm::vec3&  dragOffset,
                         float       dt) 
{
    // GUARD: If clicking on top of an ImGui window overlay, ignore physics interaction!
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    glm::vec3 mouseWorld = getMouseWorldPos(window);
    int leftState  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rightState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

    // --- Left Click Dragging Handler ---
    if (leftState == GLFW_PRESS) {
        if (!isDragging) {
            selectedInspectorBody = nullptr; // Reset inspection selection
            for (int i = (int)sceneObjects.size() - 1; i >= 0; --i) {
                if (isPointInsideBody(mouseWorld, sceneObjects[i])) {
                    isDragging            = true;
                    draggedBody           = &sceneObjects[i];
                    selectedInspectorBody = &sceneObjects[i]; // Bind to inspector window
                    dragOffset            = sceneObjects[i].position - mouseWorld;
                    break;
                }
            }
        }
        if (isDragging && draggedBody) {
            glm::vec3 newPosition = mouseWorld + dragOffset;
            if (dt > 0.001f) {
                draggedBody->velocity = (newPosition - draggedBody->position) / dt;
            }
            draggedBody->position        = newPosition;
            draggedBody->angularVelocity = 0.0f;
        }
    } else {
        isDragging  = false;
        draggedBody = nullptr;
    }

    // --- Right Click Connection Handler ---
    static bool rightButtonLatch = false;
    if (rightState == GLFW_PRESS) {
        if (!rightButtonLatch) {
            rightButtonLatch = true; // Simple push latch mechanism
            
            if (currentLinkMode != LinkMode::NONE) {
                RigidBody* clickedTarget = nullptr;
                for (int i = (int)sceneObjects.size() - 1; i >= 0; --i) {
                    if (isPointInsideBody(mouseWorld, sceneObjects[i])) {
                        clickedTarget = &sceneObjects[i];
                        break;
                    }
                }

                if (clickedTarget) {
                    if (!linkFirstSelection) {
                        linkFirstSelection = clickedTarget;
                    } else if (linkFirstSelection != clickedTarget) {
                        float currentDist = glm::distance(linkFirstSelection->position, clickedTarget->position);
                        
                        if (currentLinkMode == LinkMode::SPRING) {
                            SpringJoint sj;
                            sj.bodyA = linkFirstSelection;
                            sj.bodyB = clickedTarget;
                            sj.restLength_L0 = currentDist;
                            sj.materialIndex = selectedMaterialIndex;
                            globalSprings.push_back(sj);
                        } else if (currentLinkMode == LinkMode::ROD) {
                            DistanceRod dr;
                            dr.bodyA = linkFirstSelection;
                            dr.bodyB = clickedTarget;
                            dr.targetLength = currentDist;
                            dr.materialIndex = selectedMaterialIndex;
                            globalRods.push_back(dr);
                        }
                        
                        // BUG FIX: Only clear the first-selection latch, NOT the link mode.
                        // Previously the tool deactivated after every single connection,
                        // forcing the user to re-equip it each time. The mode now persists
                        // so you can chain-link multiple objects without re-equipping.
                        linkFirstSelection = nullptr;
                        // currentLinkMode intentionally left active
                    }
                }
            }
        }
    } else {
        rightButtonLatch = false;
    }
}

// =============================================================================
// Helper Spawner Logic
// =============================================================================
void spawnObjectFromTemplate(ShapeType type, float mass, float size, float rest, float frict, float drag) {
    RigidBody body;
    float rx = 3.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 6.0f));
    
    body.configure(type, glm::vec3(rx, 10.5f, 0.0f), mass, size, drag, rest, frict);
    body.gravity = globalGravity; // Set base gravity vector
    sceneObjects.push_back(body);
}

void keyCallback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;

    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, true);
    if (key == GLFW_KEY_T)      spawnObjectFromTemplate(ShapeType::Triangle, 1.0f, 0.5f, 0.5f, 0.2f, 0.05f);
    if (key == GLFW_KEY_B)      spawnObjectFromTemplate(ShapeType::Square, 10.0f, 0.5f, 0.3f, 0.4f, 0.05f);
    if (key == GLFW_KEY_C)      spawnObjectFromTemplate(ShapeType::Circle, 2.5f, 0.5f, 0.7f, 0.4f, 0.02f);
    if (key == GLFW_KEY_R) {
        staticRamp.configure(ShapeType::Ramp, glm::vec3(1.0f, 6.0f, 0.0f), 0.0f, 0.0f, 0.0f, 0.2f, 0.5f);
        staticRamp.rampEnd = glm::vec3(13.0f, 2.5f, 0.0f);
        rampActive         = true;
    }
}

void framebuffer_size_callback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

glm::mat4 buildTransform(const RigidBody& body) {
    glm::mat4 m = glm::translate(glm::mat4(1.0f), body.position);
    m           = glm::rotate(m, body.angle, glm::vec3(0.0f, 0.0f, 1.0f));
    float s     = (body.type == ShapeType::Circle) ? body.radius * 2.0f : body.halfHeight * 2.0f;
    m           = glm::scale(m, glm::vec3(s, s, 1.0f));
    return m;
}

// =============================================================================
// main
// =============================================================================
int main() {
    srand(static_cast<unsigned int>(time(nullptr)));

    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL Material Engineering Sandbox", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, keyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    // ---- Setup Dear ImGui context ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ---- Shaders & meshes ----
    Shader standardShader("shader.vert", "shader.frag");
    Shader circleShader  ("shader.vert", "circle.frag");
    Mesh   triangleMesh  (triangleVerts, sizeof(triangleVerts), 3);
    Mesh   squareMesh    (squareVerts,   sizeof(squareVerts),   6);

    // ---- Drag state ----
    RigidBody* draggedBody = nullptr;
    bool       isDragging  = false;
    glm::vec3  dragOffset(0.0f);

    constexpr float FIXED_DT  = 1.0f / 60.0f;
    float           lastFrame = 0.0f;
    float           accumulator = 0.0f;
    glm::mat4       projection(1.0f);

    while (!glfwWindowShouldClose(window)) {
        float currentFrame = (float)glfwGetTime();
        float frameTime    = glm::min(currentFrame - lastFrame, 0.25f);
        lastFrame          = currentFrame;

        accumulator += frameTime * simulationTimeScale;

        int winW, winH;
        glfwGetWindowSize(window, &winW, &winH);
        float aspect           = (float)winW / (float)winH;
        float dynamicWorldWidth = WORLD_HEIGHT * aspect;

        projection = glm::ortho(0.0f, dynamicWorldWidth, 0.0f, WORLD_HEIGHT, -1.0f, 1.0f);

        // ---- Start ImGui Frame Context ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        handleMouseDragging(window, draggedBody, isDragging, dragOffset, frameTime);

        // =====================================================================
        // ImGui Windows Layout Definitions
        // =====================================================================
        {
            // --- PANEL 1: Global Settings ---
            ImGui::Begin("Global Physics Control Panel");
            ImGui::Text("Simulation Statistics:");
            ImGui::Text("Loop speed: %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("Objects: %d | Springs: %d | Rods: %d", (int)sceneObjects.size(), (int)globalSprings.size(), (int)globalRods.size());
            
            ImGui::Separator();
            ImGui::Text("Environment Constants:");
            if (ImGui::SliderFloat2("World Gravity Vector", &globalGravity.x, -20.0f, 20.0f)) {
                for (auto& obj : sceneObjects) obj.gravity = globalGravity;
            }
            ImGui::SliderFloat("Simulation Speed (TimeScale)", &simulationTimeScale, 0.0f, 2.0f, "%.2fx");

            ImGui::Separator();
            if (ImGui::Button("Clear All System Entities", ImVec2(-1, 25))) {
                sceneObjects.clear();
                globalSprings.clear();
                globalRods.clear();
                selectedInspectorBody = nullptr;
                linkFirstSelection = nullptr;
            }
            if (ImGui::Button("Toggle Ground Ramp Line", ImVec2(-1, 25))) {
                if (!rampActive) {
                    staticRamp.configure(ShapeType::Ramp, glm::vec3(1.0f, 5.5f, 0.0f), 0.0f, 0.0f, 0.0f, 0.2f, 0.5f);
                    staticRamp.rampEnd = glm::vec3(dynamicWorldWidth - 1.0f, 2.0f, 0.0f);
                    rampActive = true;
                } else {
                    rampActive = false;
                }
            }
            ImGui::End();

            // --- PANEL 2: Interactive Spawner Settings ---
            ImGui::Begin("Shape Spawner Menu");
            const char* shapesList[] = { "Triangle", "Square", "Circle" };
            ImGui::Combo("Spawn Geometry Model", &uiSelectedShapeType, shapesList, IM_ARRAYSIZE(shapesList));
            
            ImGui::SliderFloat("Mass Setting", &uiSpawnMass, 0.1f, 100.0f, "%.1f kg");
            ImGui::SliderFloat("Extent Bounds / Size", &uiSpawnSize, 0.1f, 2.0f, "%.2f units");
            ImGui::SliderFloat("Restitution (Bounciness)", &uiSpawnRestitution, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Surface Friction Coefficient", &uiSpawnFriction, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Linear Air Drag Resistance", &uiSpawnDrag, 0.0f, 0.5f, "%.3f");

            if (ImGui::Button("Drop Shape Into Scene Box", ImVec2(-1, 30))) {
                spawnObjectFromTemplate(static_cast<ShapeType>(uiSelectedShapeType), 
                                        uiSpawnMass, uiSpawnSize, uiSpawnRestitution, 
                                        uiSpawnFriction, uiSpawnDrag);
            }

            ImGui::Separator();
            ImGui::Text("Engineering Structural Connections:");
            
            if (ImGui::BeginCombo("Material Preset", MATERIAL_DATABASE[selectedMaterialIndex].name.c_str())) {
                for (int i = 0; i < (int)MATERIAL_DATABASE.size(); i++) {
                    bool isSelected = (selectedMaterialIndex == i);
                    if (ImGui::Selectable(MATERIAL_DATABASE[i].name.c_str(), isSelected)) {
                        selectedMaterialIndex = i;
                    }
                }
                ImGui::EndCombo();
            }

            if (currentLinkMode == LinkMode::SPRING) {
                ImGui::TextColored(ImVec4(0,1,0,1), "Tool: Right-Click 2 objects to bind via Spring.");
            } else if (currentLinkMode == LinkMode::ROD) {
                ImGui::TextColored(ImVec4(0,0,1,1), "Tool: Right-Click 2 objects to bind via Rigid Rod.");
            } else {
                ImGui::Text("Linker Status: Inactive");
            }

            if (ImGui::Button("Equip Spring Linker")) { currentLinkMode = LinkMode::SPRING; linkFirstSelection = nullptr; }
            ImGui::SameLine();
            if (ImGui::Button("Equip Rod Linker")) { currentLinkMode = LinkMode::ROD; linkFirstSelection = nullptr; }

            ImGui::Separator();
            static int chainSegments = 8;
            static bool chainTypeSpring = true;
            ImGui::SliderInt("Chain Links", &chainSegments, 2, 25);
            ImGui::Checkbox("Use Elastic Spring Links", &chainTypeSpring);
            if (ImGui::Button("Spawn Structural Chain Bridge", ImVec2(-1, 25))) {
                createChain(glm::vec3(2.0f, 9.5f, 0.0f), glm::vec3(dynamicWorldWidth - 2.0f, 9.5f, 0.0f), chainSegments, chainTypeSpring);
            }
            ImGui::End();

            // --- PANEL 3: Live Selected Object Properties Inspector ---
            ImGui::Begin("Active Properties Inspector");
            if (selectedInspectorBody == nullptr) {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Click/drag an object to inspect it.");
            } else {
                ImGui::Text("Inspecting Shape: %s", 
                            (selectedInspectorBody->type == ShapeType::Circle) ? "Circle" : 
                            (selectedInspectorBody->type == ShapeType::Square) ? "Square" : "Triangle");
                
                ImGui::Separator();
                ImGui::Text("Position: X:%.2f, Y:%.2f", selectedInspectorBody->position.x, selectedInspectorBody->position.y);
                ImGui::Text("Velocity: X:%.2f, Y:%.2f", selectedInspectorBody->velocity.x, selectedInspectorBody->velocity.y);
                ImGui::Text("Angular Speed: %.2f rad/s", selectedInspectorBody->angularVelocity);

                ImGui::Separator();
                if (ImGui::SliderFloat("Object Mass", &selectedInspectorBody->mass, 0.1f, 100.0f, "%.1f")) {
                    selectedInspectorBody->invMass = (selectedInspectorBody->mass > 0.0f) ? 1.0f / selectedInspectorBody->mass : 0.0f;
                    float size = selectedInspectorBody->halfHeight * 2.0f;
                    if (selectedInspectorBody->type == ShapeType::Circle) {
                        selectedInspectorBody->inertia = 0.5f * selectedInspectorBody->mass * (selectedInspectorBody->radius * selectedInspectorBody->radius);
                    } else {
                        selectedInspectorBody->inertia = (1.0f / 12.0f) * selectedInspectorBody->mass * (size * size + size * size);
                    }
                    selectedInspectorBody->invInertia = (selectedInspectorBody->inertia > 0.0f) ? 1.0f / selectedInspectorBody->inertia : 0.0f;
                }
                ImGui::SliderFloat("Restitution", &selectedInspectorBody->restitution, 0.0f, 1.0f);
                ImGui::SliderFloat("Friction",    &selectedInspectorBody->friction,    0.0f, 1.0f);
                ImGui::SliderFloat("Air Drag",    &selectedInspectorBody->dragCoefficient, 0.0f, 0.5f);
            }
            ImGui::End();
        }

        // ---- Fixed-step physics pipeline ----
        //
        // Architecture (matches Box2D / Bullet approach):
        //
        //  Per tick:
        //   1. Sub-step springs  (SPRING_SUBSTEPS mini-steps of FIXED_DT/N)
        //      Sub-stepping lets the spring ODE stay stable with physically
        //      correct stiffness values from the Wahl formula — no fudge factors.
        //   2. Integrate bodies  (one full FIXED_DT step)
        //   3. Iterate rod velocity constraints  (VELOCITY_ITERATIONS passes)
        //      Multiple passes let impulses propagate through chains (convergence).
        //   4. Resolve collisions
        //   5. Rod position projection  (one pass, zero energy injection)
        //      Corrects positional drift without Baumgarte's energy bias or its
        //      frame-1 impulse spike that was shattering chains on spawn.
        //   6. Spring breaking check  (once per tick, after all sub-steps)
        //   7. Prune broken joints

        constexpr int   SPRING_SUBSTEPS      = 8;   // sub-steps per tick for springs
        constexpr int   VELOCITY_ITERATIONS  = 10;  // sequential impulse iterations for rods
        constexpr float SUB_DT               = FIXED_DT / static_cast<float>(SPRING_SUBSTEPS);

        while (accumulator >= FIXED_DT) {

            // 1. Spring sub-steps — stable integration of stiff spring ODEs
            for (int sub = 0; sub < SPRING_SUBSTEPS; ++sub) {
                for (auto& spring : globalSprings) {
                    spring.updateAndApplyForces(SUB_DT);
                }
            }

            // 2. Integrate bodies
            for (auto& obj : sceneObjects) {
                if (&obj == draggedBody) continue;
                obj.update(FIXED_DT);
            }

            // 3. Rod velocity constraint iterations (sequential impulses)
            for (int iter = 0; iter < VELOCITY_ITERATIONS; ++iter) {
                for (auto& rod : globalRods) {
                    rod.resolveVelocity(FIXED_DT);
                }
            }

            // 4. Collision resolution
            for (size_t i = 0; i < sceneObjects.size(); ++i) {
                if (rampActive) resolveCircleVsRamp(sceneObjects[i], staticRamp);
                resolveWorldBoundaries(sceneObjects[i], 0.0f, dynamicWorldWidth, 0.0f, WORLD_HEIGHT);
                for (size_t j = i + 1; j < sceneObjects.size(); ++j) {
                    resolveObjectCollisions(sceneObjects[i], sceneObjects[j]);
                }
            }

            // 5. Rod position projection (drift correction, no energy added)
            for (auto& rod : globalRods) {
                rod.resolvePosition();
            }

            // 6. Spring breaking check — evaluated on strain after all sub-steps
            for (auto& spring : globalSprings) {
                spring.checkBreaking();
            }

            // 7. Prune broken joints
            globalSprings.erase(
                std::remove_if(globalSprings.begin(), globalSprings.end(),
                    [](const SpringJoint& s) { return s.isBroken; }),
                globalSprings.end()
            );
            globalRods.erase(
                std::remove_if(globalRods.begin(), globalRods.end(),
                    [](const DistanceRod& r) { return r.isBroken; }),
                globalRods.end()
            );

            accumulator -= FIXED_DT;
        }

        // ---- Render Output Framework ----
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Render Ramp
        if (rampActive) {
            glm::vec3 dir       = staticRamp.rampEnd - staticRamp.position;
            float     rampAngle = atan2(dir.y, dir.x);

            glm::mat4 rampTransform = glm::translate(glm::mat4(1.0f), staticRamp.position);
            rampTransform           = glm::rotate(rampTransform, rampAngle, glm::vec3(0.0f, 0.0f, 1.0f));
            rampTransform           = glm::scale(rampTransform, glm::vec3(glm::length(dir), 0.1f, 1.0f));

            standardShader.use();
            standardShader.setMat4("projection", projection);
            standardShader.setMat4("transform",  rampTransform);
            squareMesh.draw();
        }

        // Render Sim Objects
        for (const auto& obj : sceneObjects) {
            glm::mat4 transform = buildTransform(obj);

            if (obj.type == ShapeType::Circle) {
                circleShader.use();
                circleShader.setMat4("projection", projection);
                circleShader.setMat4("transform",  transform);
                squareMesh.draw();
            } else {
                standardShader.use();
                standardShader.setMat4("projection", projection);
                standardShader.setMat4("transform",  transform);

                if (obj.type == ShapeType::Triangle)
                    triangleMesh.draw();
                else
                    squareMesh.draw();
            }
        }

        // Render Joint Links via Direct Pipeline Line Drawing
        glUseProgram(0); // Unbind programmable core shaders to use standard pipeline matrices
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glLineWidth(3.0f);
        glBegin(GL_LINES);
        // Draw Dynamic Springs (Green -> Red gradient shift depending on strain stress)
        for (const auto& spring : globalSprings) {
            float breakingLimit = MATERIAL_DATABASE[spring.materialIndex].breakingStrain;
            float strainFactor = std::min(std::abs(spring.currentStrain) / breakingLimit, 1.0f);
            glColor3f(strainFactor, 1.0f - strainFactor, 0.0f);
            glVertex2f(spring.bodyA->position.x, spring.bodyA->position.y);
            glVertex2f(spring.bodyB->position.x, spring.bodyB->position.y);
        }
        // Draw Rigid Distance Rods (Blue color layout)
        glColor3f(0.2f, 0.6f, 1.0f);
        for (const auto& rod : globalRods) {
            glVertex2f(rod.bodyA->position.x, rod.bodyA->position.y);
            glVertex2f(rod.bodyB->position.x, rod.bodyB->position.y);
        }
        glEnd();
        glColor3f(1.0f, 1.0f, 1.0f); // Reset color map profile

        // ---- Finalize and render the ImGui application drawer layout ----
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ---- Cleanup Allocations ----
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}

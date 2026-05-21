#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>

#include <iostream>
#include <vector>
#include <deque>
#include <list>
#include <algorithm>
#include <ctime>
#include <cmath>

#include "shader.h"
#include "mesh.h"
#include "physics.h"
#include "collision.h"
#include "joints.h"

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
std::list<RigidBody>   sceneObjects;
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
// drawProceduralSpring
//   Generates a 3D-projected helical wire spring using immediate OpenGL lines
// =============================================================================
void drawProceduralSpring(glm::vec3 start, glm::vec3 end, int numCoils, float springWidth) {
    glm::vec3 delta = end - start;
    float currentLength = glm::length(delta);
    
    if (currentLength < 0.001f) return; // Guard division by zero

    glm::vec3 dir = delta / currentLength;                     
    glm::vec3 perp = glm::vec3(-dir.y, dir.x, 0.0f);           

    constexpr int segmentsPerCoil = 24;
    int totalPoints = numCoils * segmentsPerCoil;

    float leadInFrac = 0.05f; // 5% straight leads at ends
    float coiledLength = currentLength * (1.0f - 2.0f * leadInFrac);
    glm::vec3 coilStart = start + dir * (currentLength * leadInFrac);

    glBegin(GL_LINE_STRIP);
    glVertex2f(start.x, start.y);
    glVertex2f(coilStart.x, coilStart.y);

    for (int i = 0; i <= totalPoints; ++i) {
        float t = (float)i / (float)totalPoints;
        float progressAlong = t * coiledLength;
        glm::vec3 corePoint = coilStart + dir * progressAlong;

        float angle = t * numCoils * 2.0f * glm::pi<float>();
        float lateralOffset = std::sin(angle) * springWidth;

        glm::vec3 vertexPos = corePoint + perp * lateralOffset;
        glVertex2f(vertexPos.x, vertexPos.y);
    }

    glVertex2f(end.x, end.y);
    glEnd();
}

// =============================================================================
// createChain
// =============================================================================
void createChain(glm::vec3 start, glm::vec3 end, int totalSegments, bool useSpring) {
    if (totalSegments < 2) return;

    glm::vec3 step = (end - start) / static_cast<float>(totalSegments);

    // Collect stable pointers as we push. std::list never invalidates existing
    // node pointers on push_back, so this is safe.
    std::vector<RigidBody*> nodes;
    nodes.reserve(totalSegments + 1);

    for (int i = 0; i <= totalSegments; ++i) {
        bool isAnchor = (i == 0 || i == totalSegments);

        RigidBody node;
        node.configure(ShapeType::Square,
                       start + step * static_cast<float>(i),
                       isAnchor ? 0.0f : 2.0f,
                       0.25f, 0.05f, 0.3f, 0.4f);
        node.velocity        = glm::vec3(0.0f);
        node.angle           = 0.0f;
        node.angularVelocity = 0.0f;
        node.gravity         = globalGravity;

        sceneObjects.push_back(node);
        RigidBody* ptr = &sceneObjects.back(); // stable — list never moves nodes

        if (isAnchor) ptr->freeze();
        nodes.push_back(ptr);
    }

    // Wire up joints between consecutive nodes using the stable pointer array.
    for (int i = 1; i <= totalSegments; ++i) {
        float dist = glm::distance(nodes[i]->position, nodes[i-1]->position);
        if (useSpring) {
            SpringJoint s;
            s.bodyA         = nodes[i-1];
            s.bodyB         = nodes[i];
            s.restLength_L0 = dist;
            s.materialIndex = selectedMaterialIndex;
            globalSprings.push_back(s);
        } else {
            DistanceRod r;
            r.bodyA        = nodes[i-1];
            r.bodyB        = nodes[i];
            r.targetLength = dist;
            r.materialIndex = selectedMaterialIndex;
            globalRods.push_back(r);
        }
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
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    glm::vec3 mouseWorld = getMouseWorldPos(window);
    int leftState  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rightState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

    if (leftState == GLFW_PRESS) {
        if (!isDragging) {
            selectedInspectorBody = nullptr; 
            for (auto it = sceneObjects.rbegin(); it != sceneObjects.rend(); ++it) {
                if (isPointInsideBody(mouseWorld, *it)) {
                    isDragging            = true;
                    draggedBody           = &(*it);
                    selectedInspectorBody = &(*it);
                    dragOffset            = it->position - mouseWorld;
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

    static bool rightButtonLatch = false;
    if (rightState == GLFW_PRESS) {
        if (!rightButtonLatch) {
            rightButtonLatch = true; 
            
            if (currentLinkMode != LinkMode::NONE) {
                RigidBody* clickedTarget = nullptr;
                for (auto it = sceneObjects.rbegin(); it != sceneObjects.rend(); ++it) {
                    if (isPointInsideBody(mouseWorld, *it)) {
                        clickedTarget = &(*it);
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
                        
                        linkFirstSelection = nullptr;
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
    body.gravity = globalGravity; 
    sceneObjects.push_back(body);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
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

    // --- Dynamic Freeze Key Hook ---
    if (key == GLFW_KEY_F) {
        if (selectedInspectorBody != nullptr) {
            if (selectedInspectorBody->isFrozen) {
                selectedInspectorBody->unfreeze();
            } else {
                selectedInspectorBody->freeze();
            }
        }
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
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGL Material Engineering Sandbox", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, keyCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    Shader standardShader("shader.vert", "shader.frag");
    Shader circleShader  ("shader.vert", "circle.frag");
    Mesh   triangleMesh  (triangleVerts, sizeof(triangleVerts), 3);
    Mesh   squareMesh    (squareVerts,   sizeof(squareVerts),   6);

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

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        handleMouseDragging(window, draggedBody, isDragging, dragOffset, frameTime);

        // =====================================================================
        // ImGui Windows Layout Definitions
        // =====================================================================
        {
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

            ImGui::Begin("Active Properties Inspector");
            if (selectedInspectorBody == nullptr) {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Click/drag an object to inspect it.");
            } else {
                ImGui::Text("Inspecting Shape: %s", 
                            (selectedInspectorBody->type == ShapeType::Circle) ? "Circle" : 
                            (selectedInspectorBody->type == ShapeType::Square) ? "Square" : "Triangle");
                
                ImGui::Separator();
                if (selectedInspectorBody->isFrozen) {
                    ImGui::TextColored(ImVec4(0.3f, 0.7f, 1.0f, 1.0f), "STATE: LOCKED/STATIC [Press F to Unfreeze]");
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "STATE: ACTIVE DYNAMIC [Press F to Freeze]");
                }

                ImGui::Separator();
                ImGui::Text("Position: X:%.2f, Y:%.2f", selectedInspectorBody->position.x, selectedInspectorBody->position.y);
                ImGui::Text("Velocity: X:%.2f, Y:%.2f", selectedInspectorBody->velocity.x, selectedInspectorBody->velocity.y);
                ImGui::Text("Angular Speed: %.2f rad/s", selectedInspectorBody->angularVelocity);

                ImGui::Separator();
                // Handle mass tracking adjustments while honoring stashed values if frozen
                float editableMass = selectedInspectorBody->isFrozen ? selectedInspectorBody->originalMass : selectedInspectorBody->mass;
                if (ImGui::SliderFloat("Object Mass", &editableMass, 0.1f, 100.0f, "%.1f")) {
                    if (selectedInspectorBody->isFrozen) {
                        selectedInspectorBody->originalMass = editableMass;
                    } else {
                        selectedInspectorBody->mass = editableMass;
                        selectedInspectorBody->invMass = (selectedInspectorBody->mass > 0.0f) ? 1.0f / selectedInspectorBody->mass : 0.0f;
                        float size = selectedInspectorBody->halfHeight * 2.0f;
                        if (selectedInspectorBody->type == ShapeType::Circle) {
                            selectedInspectorBody->inertia = 0.5f * selectedInspectorBody->mass * (selectedInspectorBody->radius * selectedInspectorBody->radius);
                        } else {
                            selectedInspectorBody->inertia = (1.0f / 12.0f) * selectedInspectorBody->mass * (size * size + size * size);
                        }
                        selectedInspectorBody->invInertia = (selectedInspectorBody->inertia > 0.0f) ? 1.0f / selectedInspectorBody->inertia : 0.0f;
                    }
                }
                ImGui::SliderFloat("Restitution", &selectedInspectorBody->restitution, 0.0f, 1.0f);
                ImGui::SliderFloat("Friction",    &selectedInspectorBody->friction,    0.0f, 1.0f);
                ImGui::SliderFloat("Air Drag",    &selectedInspectorBody->dragCoefficient, 0.0f, 0.5f);
            }
            ImGui::End();
        }

        // ---- Fixed-step physics pipeline ----
        constexpr int   SPRING_SUBSTEPS      = 8;   
        constexpr int   VELOCITY_ITERATIONS  = 10;  
        constexpr float SUB_DT               = FIXED_DT / static_cast<float>(SPRING_SUBSTEPS);

        while (accumulator >= FIXED_DT) {

            // 1. Spring sub-steps
            for (int sub = 0; sub < SPRING_SUBSTEPS; ++sub) {
                for (auto& spring : globalSprings) {
                    spring.updateAndApplyForces(SUB_DT);
                }
            }

            // 2. Integrate bodies
            for (auto& obj : sceneObjects) {
                if (&obj == draggedBody) continue;
                if (obj.isFrozen) continue; // BASE SAFETY GUARD: Skip gravity calculations on frozen items!
                obj.update(FIXED_DT);
            }

            // 3. Rod velocity constraint iterations (sequential impulses)
            for (int iter = 0; iter < VELOCITY_ITERATIONS; ++iter) {
                for (auto& rod : globalRods) {
                    rod.resolveVelocity(FIXED_DT);
                }
            }

            // 4. Collision resolution
            for (auto itI = sceneObjects.begin(); itI != sceneObjects.end(); ++itI) {
                if (rampActive && itI->type == ShapeType::Circle) // Fix #6: only circles vs ramp
                    resolveCircleVsRamp(*itI, staticRamp);
                resolveWorldBoundaries(*itI, 0.0f, dynamicWorldWidth, 0.0f, WORLD_HEIGHT);
                for (auto itJ = std::next(itI); itJ != sceneObjects.end(); ++itJ) {
                    resolveObjectCollisions(*itI, *itJ);
                }
            }

            // 5. Rod position projection
            for (auto& rod : globalRods) {
                rod.resolvePosition();
            }

            // 6. Spring breaking check
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

        for (const auto& obj : sceneObjects) {
            glm::mat4 transform = buildTransform(obj);

            if (obj.type == ShapeType::Circle) {
                circleShader.use();
                circleShader.setMat4("projection", projection);
                circleShader.setMat4("transform",  transform);
                
                // Color override if frozen
                if (obj.isFrozen)
                    glUniform4f(glGetUniformLocation(circleShader.ID, "borderColor"), 0.3f, 0.7f, 1.0f, 1.0f);
                else
                    glUniform4f(glGetUniformLocation(circleShader.ID, "borderColor"), 1.0f, 1.0f, 1.0f, 1.0f);
                squareMesh.draw();
            } else {
                standardShader.use();
                standardShader.setMat4("projection", projection);
                standardShader.setMat4("transform",  transform);

                if (obj.isFrozen) {
                    glLineWidth(2.0f); // Frosted/steel accent
                }

                if (obj.type == ShapeType::Triangle)
                    triangleMesh.draw();
                else
                    squareMesh.draw();

                glLineWidth(1.0f); // Reset after each object
            }
        }

        // ---- Joint Render Output Core Pipeline ----
        glUseProgram(0); 
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // 1. Draw Dynamic Springs as Realistic Coils
        glLineWidth(2.5f);
        for (const auto& spring : globalSprings) {
            int matIdx = spring.materialIndex;
            if (matIdx < 0 || matIdx >= (int)MATERIAL_DATABASE.size()){
                matIdx = 0;
            }

            float breakingLimit = MATERIAL_DATABASE[matIdx].breakingStrain;
            float strainFactor = std::min(std::abs(spring.currentStrain) / breakingLimit, 1.0f);

            glColor3f(strainFactor, 1.0f - strainFactor, 0.0f);
            
            drawProceduralSpring(spring.bodyA->position, spring.bodyB->position, 14, 0.12f);
        }

        // 2. Draw Rigid Distance Rods as Parallel Structural Beams
        for (const auto& rod : globalRods) {
            glColor3f(0.4f, 0.6f, 0.7f); // Clean steel bar layout tint
            
            glm::vec3 delta = rod.bodyB->position - rod.bodyA->position;
            float len = glm::length(delta);
            if (len > 0.001f) {
                glm::vec3 dir = delta / len;
                glm::vec3 perp = glm::vec3(-dir.y, dir.x, 0.0f) * 0.05f; // Side displacement
                
                glBegin(GL_LINES);
                glVertex2f(rod.bodyA->position.x, rod.bodyA->position.y);
                glVertex2f(rod.bodyB->position.x, rod.bodyB->position.y);
                
                glVertex2f(rod.bodyA->position.x + perp.x, rod.bodyA->position.y + perp.y);
                glVertex2f(rod.bodyB->position.x + perp.x, rod.bodyB->position.y + perp.y);
                
                glVertex2f(rod.bodyA->position.x - perp.x, rod.bodyA->position.y - perp.y);
                glVertex2f(rod.bodyB->position.x - perp.x, rod.bodyB->position.y - perp.y);
                glEnd();
            }
        }
        glColor3f(1.0f, 1.0f, 1.0f); 

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}

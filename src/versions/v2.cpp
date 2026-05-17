#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>


// ======================================
// Shader Source Code
// ======================================

// Vertex Shader
const char* vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"     // Vertex position input
    "uniform mat4 transform;\n"                 // Transformation matrix
    "void main() {\n"
    "   gl_Position = transform * vec4(aPos, 1.0);\n"
    "}\0";

// Fragment Shader
const char* fragmentShaderSource = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n" // Orange color
    "}\n\0";


int main() {

    // ======================================
    // Initialize GLFW
    // ======================================
    if (!glfwInit()) {
        std::cout << "Error: Couldn't initialize GLFW!\n";
        return -1;
    }

    // Tell GLFW which OpenGL version we want (3.3 Core)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    // ======================================
    // Create Window
    // ======================================
    GLFWwindow* window = glfwCreateWindow(800, 600, "test", NULL, NULL);

    if (!window) {
        std::cout << "Error: Couldn't create window!\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);


    // ======================================
    // Initialize GLAD
    // ======================================
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Error: GLAD failed to load!\n";
        return -1;
    }


    // ======================================
    // Vertex Data
    // ======================================

    // Triangle vertices
    float triangleVerts[] = {
        -0.5f, -0.5f, 0.0f,
         0.5f, -0.5f, 0.0f,
         0.0f,  0.5f, 0.0f
    };

    // Square vertices (2 triangles)
    float squareVerts[] = {
        -0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f,  -0.5f, -0.5f, 0.0f,
        -0.5f,  0.5f, 0.0f,   0.5f,  0.5f, 0.0f,   0.5f, -0.5f, 0.0f
    };


    // ======================================
    // Triangle VAO/VBO Setup
    // ======================================
    unsigned int triangleVBO, triangleVAO;

    glGenVertexArrays(1, &triangleVAO);
    glGenBuffers(1, &triangleVBO);

    glBindVertexArray(triangleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(triangleVerts), triangleVerts, GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,                      // Attribute location
        3,                      // 3 values per vertex (x, y, z)
        GL_FLOAT,
        GL_FALSE,
        3 * sizeof(float),
        (void*)0
    );

    glEnableVertexAttribArray(0);


    // ======================================
    // Square VAO/VBO Setup
    // ======================================
    unsigned int squareVBO, squareVAO;

    glGenVertexArrays(1, &squareVAO);
    glGenBuffers(1, &squareVBO);

    glBindVertexArray(squareVAO);
    glBindBuffer(GL_ARRAY_BUFFER, squareVBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(squareVerts), squareVerts, GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        3 * sizeof(float),
        (void*)0
    );

    glEnableVertexAttribArray(0);


    // ======================================
    // Shader Compilation
    // ======================================
    int success;
    char infoLog[512];

    // Vertex Shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cout << "ERROR: Vertex Shader Compilation Failed\n" << infoLog << std::endl;
    }

    // Fragment Shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cout << "ERROR: Fragment Shader Compilation Failed\n" << infoLog << std::endl;
    }


    // ======================================
    // Shader Program Linking
    // ======================================
    unsigned int shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout << "ERROR: Shader Program Linking Failed\n" << infoLog << std::endl;
    }

    // Individual shaders are no longer needed after linking
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);


    // ======================================
    // Shape Toggle State
    // ======================================
    bool show_triangle = false;
    bool show_square = false;


    // ======================================
    // Main Render Loop
    // ======================================
    while (!glfwWindowShouldClose(window)) {

        // Close window with ESC key
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, true);
        }

        // Press T to show triangle
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
            show_triangle = true;
            show_square = false;
        }

        // Press B to show square
        if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS) {
            show_triangle = false;
            show_square = true;
        }


        // ======================================
        // Clear Screen
        // ======================================
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);


        // ======================================
        // Apply Shader + Rotation Transform
        // ======================================
        glUseProgram(shaderProgram);

        glm::mat4 transform = glm::mat4(1.0f);

        float time = (float)glfwGetTime();

        // Rotate around Z-axis over time
        transform = glm::rotate(
            transform,
            time,
            glm::vec3(0.0f, 0.0f, 1.0f)
        );

        unsigned int transformLoc = glGetUniformLocation(shaderProgram, "transform");
        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));


        // ======================================
        // Draw Selected Shape
        // ======================================
        if (show_triangle) {
            glBindVertexArray(triangleVAO);
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
        else if (show_square) {
            glBindVertexArray(squareVAO);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }


        // ======================================
        // Update Window
        // ======================================
        glfwSwapBuffers(window);
        glfwPollEvents();
    }


    // ======================================
    // Cleanup
    // ======================================
    glDeleteVertexArrays(1, &triangleVAO);
    glDeleteBuffers(1, &triangleVBO);

    glDeleteVertexArrays(1, &squareVAO);
    glDeleteBuffers(1, &squareVBO);

    glDeleteProgram(shaderProgram);

    glfwTerminate();

    return 0;
}

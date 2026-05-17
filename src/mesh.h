#pragma once
#include <glad/glad.h>

// =============================================================================
// Mesh
//   Thin RAII wrapper around a VAO + VBO pair for static triangle-list geometry.
//   Vertex layout: vec3 position only (location 0).
// =============================================================================
struct Mesh {
    unsigned int VAO, VBO;
    int          vertCount;

    // Uploads vertex data to the GPU and sets up the attribute pointer.
    // byteSize: total size of the verts array in bytes (use sizeof(array))
    // count:    number of vertices (not triangles)
    Mesh(float* verts, int byteSize, int count) : vertCount(count) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, byteSize, verts, GL_STATIC_DRAW);

        // layout(location = 0) in vec3 aPos
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0); // Unbind for safety
    }

    ~Mesh() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    void draw() const {
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, vertCount);
    }
};

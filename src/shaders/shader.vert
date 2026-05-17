#version 330 core

layout (location = 0) in vec3 aPos;

out vec2 TexCoords; // Output variable linked to fragment program

uniform mat4 transform;
uniform mat4 projection;

void main() {
    gl_Position = projection * transform * vec4(aPos, 1.0);
    
    // Convert generic bounding box vertex offsets into clean 0.0 to 1.0 texture spaces
    TexCoords = vec2(aPos.x + 0.5, aPos.y + 0.5);
}



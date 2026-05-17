#version 330 core

out vec4 FragColor;
in vec2 TexCoords;

void main() {
    // Bring the coordinate grid mapping space to center (0.0, 0.0)
    vec2 uv = TexCoords * 2.0 - 1.0;
    
    // Compute exact radial vector length
    float dist = length(uv);
    
    // Discard any fragments past the safe unit sphere perimeter bounds
    if (dist > 1.0) {
        discard;
    }
    
    FragColor = vec4(0.0f, 0.8f, 0.7f, 1.0f); 
}    
    
    



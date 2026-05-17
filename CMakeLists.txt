cmake_minimum_required(VERSION 3.10)
project(OpenGLPhysicsEngine)
set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(include)

add_executable(app
    src/main.cpp
    src/glad.c
)

# Pass the shader directory as an absolute path so the executable can find
# shader.vert / shader.frag regardless of where it's launched from
target_compile_definitions(app PRIVATE
    SHADER_DIR="${CMAKE_SOURCE_DIR}/src/shaders/"
)

target_link_libraries(app
    glfw
    OpenGL::GL
    dl
)

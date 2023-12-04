#version 330 core

// Vertex buffer
layout (location = 0) in vec3 vertex;

// Uniform (global) buffer
uniform mat4 view_mat;
uniform mat4 projection_mat;

void main() {
    gl_Position = projection_mat * view_mat * vec4(vertex, 1.0);
}

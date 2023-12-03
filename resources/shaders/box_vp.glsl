// Material with no illumination simulation

#version 330 core

// Vertex buffer
layout (location = 0) in vec3 vertex;
layout (location = 1) in vec2 uv;

// Uniform (global) buffer
uniform mat4 world_mat;
uniform mat4 view_mat;
uniform mat4 projection_mat;

out vec2 uv_interp;
void main()
{
    gl_Position = projection_mat * view_mat * world_mat * vec4(vertex, 1.0);
    uv_interp = uv;
}

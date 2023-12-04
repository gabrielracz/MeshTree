#version 330

// Attributes passed from the vertex shader
uniform vec4 color;


void main() 
{
	gl_FragColor = color;
}

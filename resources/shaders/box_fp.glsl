// Material with no illumination simulation

#version 330

in vec2 uv_interp;

// Attributes passed from the vertex shader
uniform vec4 color;
uniform int draw_edges;


void main() 
{
    vec4 col = color;
    float edgeThreshold = 0.005;
    bool onEdge = (uv_interp.x < edgeThreshold || uv_interp.x > 1.0 - edgeThreshold ||
                    uv_interp.y < edgeThreshold || uv_interp.y > 1.0 - edgeThreshold);
    if(onEdge && draw_edges == 1) {
        col.a = 0.4;
    }
	gl_FragColor = col;
}

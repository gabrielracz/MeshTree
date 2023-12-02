#version 130

// Attributes passed from the vertex shader
in vec3 position_interp;
in vec3 normal_interp;
in vec4 color_interp;
in vec2 uv_interp;
in vec3 light_pos;

// Uniform (global) buffer
uniform sampler2D texture_map;
uniform vec4 light_col;
uniform float spec_power;
uniform float amb;
uniform vec4 ambcol;
uniform float timer;

float phong_specular(vec3 lv, vec3 n) {
	vec3 v = vec3(0,0,0);
    vec3 r = -lv + 2*(dot(lv, n)*n);
	vec3 vv = normalize(v - position_interp);
    float spec = max(0.0, dot(vv, r));
    spec = pow(spec, spec_power/4); //4x less intense than blinn-phong
    return spec;

}

float blinnphong_specular(vec3 lv, vec3 n) {
	vec3 v = vec3(0,0,0);
	vec3 vv = normalize(v - position_interp);
	vec3 h = normalize((vv+lv)/2); 

    float spec = max(0.0, dot(n,h)); 
    spec = pow(spec, spec_power);

    return spec;
}


vec4 lighting(vec4 pixel) {
	vec3 lv = normalize(light_pos - position_interp); // light direction, object position as origin
	vec3 n = normalize(normal_interp); 

	float diffuse = max(0.0, dot(n,lv)); 
    // float spec = blinnphong_specular(lv, n);
    float spec = phong_specular(lv, n);
		
    return diffuse*light_col*pixel + amb*ambcol*pixel + spec*light_col*spec;
}

void main() 
{
	vec4 pixel = color_interp;
    vec2 uv = uv_interp;
    float check = 0.1;

    vec4 black = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 green = vec4(0.0, 1.0, 0.0, 1.0);
    vec4 purple = vec4(1.0, 0.0, 1.0, 1.0);

    float lerper = (sin(timer*3) + 1)/3;
    vec4 black_lerp = mix(black, purple, lerper);
    // vec4 green_lerp = mix(green, black, lerper);

    if(mod(uv.x, check) <= check/2.0) {
        if(mod(uv.y, check) <= check/2.0) {
            pixel = black_lerp;
        } else {
            pixel = green;
        }
    } else {
        if(mod(uv.y, check) <= check/2.0) {
            pixel = green;
        } else {
            pixel = black_lerp;
        }
    }

    vec4 lit_pixel = lighting(pixel);
    gl_FragColor = lit_pixel;
}
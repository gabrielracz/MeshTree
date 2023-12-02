#include <iostream>
#include <exception>

#include "glm/ext/quaternion_trigonometric.hpp"
#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

void CheckControls(KeyMap& keys, View& view, Camera& camera);
void MouseControls(Camera& camera, MouseMap& mouse_buttons, Mouse& mouse);

int main(void){

    View view;
    view.Init("[] MeshTree - Gabriel Racz (c)", 800, 800);

    Mesh      bunny_mesh(RESOURCES_DIRECTORY"/bunny_full.obj");
    Shader    shader(SHADER_DIRECTORY"/lit_vp.glsl", SHADER_DIRECTORY"/lit_fp.glsl");
    Transform bunny_transform;

    Light light({0.9, 0.9, 0.9, 1.0});
    light.transform.SetPosition({10.0, 10.0, 0.0});

    Camera& camera = view.GetCamera();
    camera.transform.SetJoint(-config::camera_position);
    camera.Attach(&bunny_transform);
    light.Attach(&camera.transform);

    MouseMap& mouse_buttons = view.GetMouseButtons();

    auto mouse_controls = [&camera, &mouse_buttons](Mouse& mouse) {
        MouseControls(camera, mouse_buttons, mouse);
    };
    view.SetMouseHandler(mouse_controls);

    while(!view.Closed()) {
        CheckControls(view.GetKeys(), view, camera);

        view.Clear();
        camera.Update();
        view.RenderObj(bunny_transform, bunny_mesh, shader, light);
        view.Update();
    }

    return 0;
}

void CheckControls(KeyMap& keys, View& view, Camera& camera) {
    if(keys[GLFW_KEY_Q]) {
        view.ToggleMouseCapture();
        keys[GLFW_KEY_Q] = false;
    }

    float orbit = 0.02f;
    if(keys[GLFW_KEY_A]) {
        camera.OrbitYaw(-orbit);
    }
    if(keys[GLFW_KEY_D]) {
        camera.OrbitYaw(orbit);
    }
    if(keys[GLFW_KEY_W]) {
        camera.OrbitPitch(-orbit);
    }
    if(keys[GLFW_KEY_S]) {
        camera.OrbitPitch(orbit);
    }
}

void MouseControls(Camera& camera, MouseMap& buttons,  Mouse& mouse) {
    float mouse_sens = -0.003f;
	glm::vec2 look = mouse.move * mouse_sens;
    if(buttons[GLFW_MOUSE_BUTTON_1]) {
        camera.OrbitYaw(-look.x);
        camera.OrbitPitch(-look.y);
    }
}

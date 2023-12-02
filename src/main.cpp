
#include <iostream>
#include <exception>

#include "glm/ext/quaternion_trigonometric.hpp"
#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

void CheckControls(KeyMap& keys, Camera& camera);

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

    while(!view.Closed()) {
        CheckControls(view.GetKeys(), camera);

        view.Clear();
        camera.Update();
        view.RenderObj(bunny_transform, bunny_mesh, shader, light);
        view.Update();
    }

    return 0;
}

void CheckControls(KeyMap& keys, Camera& camera) {

    float orbit = 0.015f;
    glm::quat camera_ori = glm::inverse(glm::mat4_cast(camera.transform.GetOrientation()));
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

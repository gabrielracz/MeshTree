
#include <iostream>
#include <exception>

#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

int main(void){

    View view;
    view.Init("MeshTree - Gabriel Racz (c)", 800, 800);

    Mesh      bunny_mesh(RESOURCES_DIRECTORY"/bunny_full.obj");
    Shader    shader(SHADER_DIRECTORY"/lit_vp.glsl", SHADER_DIRECTORY"/lit_fp.glsl");
    Transform bunny_transform;
    bunny_transform.SetScale({4, 4, 4});

    Light light(Colors::White);
    light.transform.SetPosition({10.0, 10.0, 0.0});

    while(!view.Closed()) {
        view.Clear();
        view.GetCamera().Update();
        view.RenderObj(bunny_transform, bunny_mesh, shader, light);
        view.Update();
    }

    return 0;
}

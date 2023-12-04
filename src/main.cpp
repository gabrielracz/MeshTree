#include <iostream>
#include <exception>

// rendering engine
#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

// MeshTree
#include "structures.h"
#include "kdtree.h"

void CheckControls(KeyMap& keys, View& view, Camera& camera);
void MouseControls(Camera& camera, MouseMap& mouse_buttons, Mouse& mouse);
void RenderKDTree(View& view, Shader& shader, KDNode* tree);
void RenderRay(View& view, Mesh& line_mesh, Shader& line_shader, Ray& ray);

const std::vector<float> line_verts = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

KDTree* tree = nullptr;
int tree_depth = 1;
int max_elements = 1;
bool render_obj = true;
int hit_triangle = -1;
Ray ray;

int main(void){

    // RENDERING
    View view;
    view.Init("[] MeshTree - Gabriel Racz (c)", 800, 800);

    Mesh      bunny_mesh(RESOURCES_DIRECTORY"/bunny_full.obj");
    Shader    shader(SHADER_DIRECTORY"/lit_vp.glsl", SHADER_DIRECTORY"/lit_fp.glsl");
    Transform bunny_transform;

    Shader box_shader(SHADER_DIRECTORY"/box_vp.glsl", SHADER_DIRECTORY"/box_fp.glsl");

    Mesh line_mesh(line_verts, {}, {{FLOAT3, "position"}});
    Shader line_shader(SHADER_DIRECTORY"/line_vp.glsl", SHADER_DIRECTORY"/line_fp.glsl");

    Light light({0.9, 0.9, 0.9, 1.0});
    light.transform.SetPosition({10.0, 10.0, 0.0});

    Camera& camera = view.GetCamera();
    camera.transform.SetJoint(-config::camera_position);
    camera.Attach(&bunny_transform);
    camera.OrbitYaw(PI/2.0f);
    light.Attach(&camera.transform);

    MouseMap& mouse_buttons = view.GetMouseButtons();

    auto mouse_controls = [&camera, &mouse_buttons](Mouse& mouse) {
        MouseControls(camera, mouse_buttons, mouse);
    };
    view.SetMouseHandler(mouse_controls);


    // MESHTREE
    std::vector<Triangle>     triangles;
    std::vector<glm::vec3>    vertices = bunny_mesh.GetVertices();
    std::vector<unsigned int> indices = bunny_mesh.GetIndices();
    for(int i = 0; i < indices.size() - 2; i += 3) {
        triangles.emplace_back(Triangle({vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]]}));
    } 

    KDTree kdtree(triangles);
    kdtree.Build(0, 1);
    tree = &kdtree;

    ray.origin = {-1.0, 0.0, 3.0};
    // ray.direction = glm::normalize(glm::vec3{-0.4, -0.577, -0.577});
    ray.direction = glm::normalize(glm::vec3(0.0, 0.0, -1.0));
    ray.tmax = 10.0f;

    Intersection intersection;
    // if(kdtree.RayIntersect(ray, &intersection)) {
    //     glm::vec3 point = ray.direction * intersection.t0;
    //     std::cout << glm::to_string(point) << std::endl;
    //     view.RenderLine(line_mesh, line_shader, ray.origin, point, {1.0, 0.0, 0.0, 1.0});
    //     hit_triangle = intersection.triangle_id;
    //     std::cout << intersection.triangle_id << " " << intersection.t0 << std::endl;
    // } else {
    //     view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
    // }

    while(!view.Closed()) {
        CheckControls(view.GetKeys(), view, camera);

        view.Clear();
        camera.Update();
        if(render_obj) {
            view.RenderObj(bunny_transform, bunny_mesh, shader, light, hit_triangle);
        }

        Intersection intersection;
        if(kdtree.RayIntersect(ray, &intersection)) {
            glm::vec3 point = ray.direction * intersection.t0;
            // std::cout << glm::to_string(point) << std::endl;
            view.RenderLine(line_mesh, line_shader, ray.origin, point, {1.0, 0.0, 0.0, 1.0});
            hit_triangle = intersection.triangle_id;
            // std::cout << intersection.triangle_id << " " << intersection.t0 << std::endl;
        } else {
            view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
        }
        view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});

        RenderKDTree(view, box_shader, &kdtree.GetTree());
        view.Update();
    }

    return 0;
}

void RenderKDTree(View& view, Shader& shader, KDNode* node) {
    if(node == nullptr) {
        return;
    }
    view.RenderBox(shader, node->aabb.min, node->aabb.max, {1.0, 1.0, 1.0, 0.005});
    RenderKDTree(view, shader, node->left_child);
    RenderKDTree(view, shader, node->right_child);
}

void RenderRay(View& view, Mesh& line_mesh, Shader& line_shader, Ray& ray) {
    view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
}   

void CheckControls(KeyMap& keys, View& view, Camera& camera) {
    if(keys[GLFW_KEY_Q]) {
        render_obj = !render_obj;
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

    float ray_move = 0.01f;
    if(keys[GLFW_KEY_UP]) {
        ray.origin += glm::vec3(0.0, ray_move, 0.0);
    }
    if(keys[GLFW_KEY_DOWN]) {
        ray.origin += glm::vec3(0.0, -ray_move, 0.0);
    }
    if(keys[GLFW_KEY_LEFT]) {
        ray.origin += glm::vec3(-ray_move, 0.0, 0.0);
    }
    if(keys[GLFW_KEY_RIGHT]) {
        ray.origin += glm::vec3(ray_move, 0.0, 0.0);
    }
    
    if(keys[GLFW_KEY_E]) {
        view.ToggleRenderMode();
        keys[GLFW_KEY_E] = false;
    }
    if(keys[GLFW_KEY_T]) {
        tree_depth = glm::clamp(++tree_depth, 0, 20);
        tree->Build(tree_depth, max_elements);
        keys[GLFW_KEY_T] = false;
    }

    if(keys[GLFW_KEY_R]) {
        tree_depth = glm::clamp(--tree_depth, 0, 20);
        tree->Build(tree_depth, max_elements);
        keys[GLFW_KEY_R] = false;
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

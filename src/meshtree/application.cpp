#include "application.h"
#include <bits/chrono.h>
#include <functional>
#include "glm/gtc/random.hpp"
#include "glm/gtx/intersect.hpp"
#include <chrono>
#include <format>
#include <locale>



const std::vector<float> line_verts = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

void Application::Init() {
    srand(1337);
    view.Init("[] MeshTree - Gabriel Racz (c)", 800, 800);

    mesh_shader = Shader(SHADER_DIRECTORY"/lit_vp.glsl", SHADER_DIRECTORY"/lit_fp.glsl");
    box_shader = Shader(SHADER_DIRECTORY"/box_vp.glsl", SHADER_DIRECTORY"/box_fp.glsl");
    line_shader = Shader(SHADER_DIRECTORY"/line_vp.glsl", SHADER_DIRECTORY"/line_fp.glsl");
    line_mesh = Mesh(line_verts, {}, {{FLOAT3, "position"}});

    light = Light({0.9, 0.9, 0.9, 1.0});
    light.transform.SetPosition({10.0, 10.0, 0.0});

    Camera& camera = view.GetCamera();
    camera.Attach(&ray_transform);
    camera.OrbitYaw(PI/2.0f);
    light.Attach(&camera.transform);

    MouseMap& mouse_buttons = view.GetMouseButtons();

    auto mouse_controls = [this, &camera, &mouse_buttons](Mouse& mouse) {
        MouseControls(camera, mouse_buttons, mouse);
    };
    view.SetMouseHandler(mouse_controls);
    auto scroll_controls = [this, &camera](double xoffset, double yoffset) {
        ScrollControls(camera, xoffset, yoffset);
    };
    view.SetScrollHandler(scroll_controls);

    // MESHTREE
    ray_mesh = Mesh(RESOURCES_DIRECTORY"/bunny_full.obj");
    ray_triangles = GetMeshTriangles(ray_mesh);
    kdtrees[RAYTREE] = new KDTree(ray_triangles);
    kdtrees[RAYTREE]->Build(3, 1);

    col_mesh1 = Mesh(RESOURCES_DIRECTORY"/dragon.obj");
    col_transform1.SetPosition({-3.2, 0.0, 0.7});
    col_triangles = GetMeshTriangles(col_mesh1, col_transform1);

    kdtrees[COLTREE1] = new KDTree(col_triangles);
    kdtrees[COLTREE1]->Build(tree_depth, 1);
    // kdtree_col
    ray.tmax = 10.0f;
}


void Application::Update() {
    view.Clear();
    view.GetCamera().Update();
    CheckControls();

    switch(active_scene) {
        case Scene::RAYSCASTDEMO:
            UpdateRaycastDemo();
            break;
        case Scene::COLLISIONDEMO:
            UpdateCollisionDemo();
            break;
        case Scene::RAYCASTBENCHMARK:
            UpdateRaycastBenchmark();
            break;
        case Scene::COLLISIONBENCHMARK:
            UpdateCollisionBenchmark();
            break;
        default:
            break;
    }
    view.Update();
}

void Application::UpdateRaycastDemo() {
    // Render ray-intersection
    Intersection intersection;
    if(kdtrees[RAYTREE]->RayIntersect(ray, &intersection)) {
        glm::vec3 point = ray.direction * intersection.t0;
        view.RenderLine(line_mesh, line_shader, ray.origin, point, {1.0, 0.0, 0.0, 1.0});
    } else {
        view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
    }

    if(render_obj) {
        view.RenderObj(ray_transform, ray_mesh, mesh_shader, light, intersection.triangle_id);
    }

    RenderKDTree(&kdtrees[RAYTREE]->GetTree());
}
void Application::UpdateCollisionDemo() {

    NodeIntersection nodeinter;
    KDTree::TreeIntersect(*kdtrees[RAYTREE], *kdtrees[COLTREE1], &nodeinter);

    if(render_obj) {
        view.RenderObj(ray_transform, ray_mesh, mesh_shader, light, nodeinter.triangle_id1);
        view.RenderObj(col_transform1, col_mesh1, mesh_shader, light, nodeinter.triangle_id2);
    }

    RenderKDTree(&kdtrees[RAYTREE]->GetTree(), nodeinter.node_id1);
    RenderKDTree(&kdtrees[COLTREE1]->GetTree(), nodeinter.node_id2);
}

void Application::UpdateRaycastBenchmark() {
    int num_rays = 1'000'000;
    std::cout.imbue(std::locale(""));
    std::cout << "Running raycast benchmark with " << num_rays << " rays." << std::endl;

    // generate rays
    std::vector<Ray> test_rays;
    test_rays.reserve(num_rays);
    glm::vec3 simulated_camera_pos = {0.0, 0.0, 2.0f};
    float width = 8.0f;
    float height = 4.5f;
    float xstep = width / num_rays;
    float ystep = height / num_rays;
    float far_plane = -10.0f;
    for(int i = 0; i < num_rays; i++) {
        glm::vec3 screen_point = glm::vec3(xstep*(float)i, ystep*(float)i, far_plane);
        Ray r;
        bool random = true;
        if(random) {
            r.origin = glm::sphericalRand(3.0f);
            r.direction = glm::normalize(glm::sphericalRand(3.0f));
            r.tmax = 10.0f;
        } else {
            r.origin = simulated_camera_pos;
            r.direction = glm::normalize(screen_point);
            r.tmax = std::sqrt(glm::dot(screen_point, screen_point));
        }
        test_rays.push_back(r);
    }

    int tree_elapsed = 0;
    int naive_elapsed = 0;
    std::chrono::steady_clock::time_point start_time, end_time;

    Intersection isect;
    int tree_hit_count = 0;
    start_time = std::chrono::steady_clock::now();
    for(Ray& r : test_rays) {
        bool hit = kdtrees[RAYTREE]->RayIntersect(r, &isect);
        if(hit) {
            tree_hit_count++;
        }
    }
    end_time = std::chrono::steady_clock::now();
    tree_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << std::format("{:<20}", "KDTree (depth " + std::to_string(tree_depth) + "):") << std::format("{:>5}", tree_elapsed) << " ms " << std::format(std::locale(""), "{:>10L} hits", tree_hit_count) << std::endl;

    int naive_hit_count = 0;
    float time = 0.0f;
    start_time = std::chrono::steady_clock::now();
    for(Ray& r : test_rays) {
        for(Triangle& t : ray_triangles) {
            bool hit = t.Intersect(r, &time);
            if(hit && time > 0.0f) {
                naive_hit_count++;
                break;
            }
        }
    }
    end_time = std::chrono::steady_clock::now();
    naive_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << std::format("{:<20}", "Naive:") << std::format("{:>5}", naive_elapsed) << " ms "<< std::format(std::locale(""), "{:>10L} hits", naive_hit_count) << "\n" << std::endl;
    active_scene = Scene::RAYSCASTDEMO;
}

void Application::UpdateCollisionBenchmark() {
    std::cout << "Magnificent collision speed!" << std::endl;
}

void Application::RenderKDTree(KDNode* node, int hit_node) {
    if(node == nullptr) {
        return;
    }
    glm::vec4 color = {1.0, 1.0, 1.0, 0.05f/(tree_depth+1)};
    if(node->id == hit_node) {
        color = {1.0, 0.0, 0.0, 0.25};
    }
    view.RenderBox(box_shader, node->aabb.min, node->aabb.max, color, draw_edges);
    RenderKDTree(node->left_child, hit_node);
    RenderKDTree(node->right_child, hit_node);
}

std::vector<Triangle> Application::GetMeshTriangles(Mesh& mesh, Transform transform) {
    std::vector<Triangle>     triangles;
    std::vector<glm::vec3>    vertices = mesh.GetVertices();
    std::vector<unsigned int> indices = mesh.GetIndices();
    for(int i = 0; i < indices.size() - 2; i += 3) {
        glm::mat4 transf_mat = transform.GetLocalMatrix();
        glm::vec3 v1 = transf_mat * glm::vec4(vertices[indices[i]]  , 1.0f);
        glm::vec3 v2 = transf_mat * glm::vec4(vertices[indices[i+1]], 1.0f);
        glm::vec3 v3 = transf_mat * glm::vec4(vertices[indices[i+2]], 1.0f);

        triangles.emplace_back(Triangle({v1, v2, v3}));
    }
    return triangles;
}
void Application::CheckControls() {
    KeyMap& keys = view.GetKeys();
    Camera& camera = view.GetCamera();

    if(keys[GLFW_KEY_1]) {
        active_scene = Scene::RAYSCASTDEMO;
        keys[GLFW_KEY_1] = false;
    }
    if(keys[GLFW_KEY_2]) {
        active_scene = Scene::COLLISIONDEMO;
        keys[GLFW_KEY_2] = false;
    }
    if(keys[GLFW_KEY_3]) {
        active_scene = Scene::RAYCASTBENCHMARK;
        keys[GLFW_KEY_3] = false;
    }
    if(keys[GLFW_KEY_4]) {
        active_scene = Scene::COLLISIONBENCHMARK;
        keys[GLFW_KEY_4] = false;
    }

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
    if(keys[GLFW_KEY_Z]) {
        camera.distance += orbit;
    }
    if(keys[GLFW_KEY_X]) {
        camera.distance -= orbit;
    }

    float ray_move = 0.01f;
    glm::mat3 camera_ori = glm::inverse(camera.GetViewMatrix());
    if(keys[GLFW_KEY_UP]) {
        // ray.origin += glm::vec3(0.0, ray_move, 0.0);
        ray.origin += camera_ori * glm::vec3(0.0, ray_move, 0.0);
    }
    if(keys[GLFW_KEY_DOWN]) {
        ray.origin += camera_ori * glm::vec3(0.0, -ray_move, 0.0);
    }
    if(keys[GLFW_KEY_LEFT]) {
        ray.origin += camera_ori * glm::vec3(-ray_move, 0.0, 0.0);
    }
    if(keys[GLFW_KEY_RIGHT]) {
        ray.origin += camera_ori * glm::vec3(ray_move, 0.0, 0.0);
    }
    float ray_rotate = 0.01f;
    if(keys[GLFW_KEY_I]) {
        ray.direction = ray.direction * glm::angleAxis(-ray_rotate,  camera_ori * glm::vec3(1.0, 0.0, 0.0));
    }
    if(keys[GLFW_KEY_K]) {
        ray.direction = ray.direction * glm::angleAxis(ray_rotate, camera_ori * glm::vec3(1.0, 0.0, 0.0));
    }
    if(keys[GLFW_KEY_J]) {
        ray.direction = ray.direction * glm::angleAxis(-ray_rotate, camera_ori * glm::vec3(0.0, 1.0, 0.0));
    }
    if(keys[GLFW_KEY_L]) {
        ray.direction = ray.direction * glm::angleAxis(ray_rotate,  camera_ori * glm::vec3(0.0, 1.0, 0.0));
    }
    
    if(keys[GLFW_KEY_E]) {
        view.ToggleRenderMode();
        keys[GLFW_KEY_E] = false;
    }
    if(keys[GLFW_KEY_C]) {
        draw_edges = (draw_edges + 1) % 2;
        keys[GLFW_KEY_C] = false;
    }
    if(keys[GLFW_KEY_F]) {
        ray.origin = camera.transform.GetPosition() + camera_ori * glm::vec3(0.0, -0.25, -0.5);
        ray.direction = glm::normalize(-ray.origin);
        // keys[GLFW_KEY_F] = false;
    }
    if(keys[GLFW_KEY_T]) {
        tree_depth++;
        kdtrees[RAYTREE]->Build(tree_depth, max_elements);
        kdtrees[COLTREE1]->Build(tree_depth+3, max_elements);
        keys[GLFW_KEY_T] = false;
    }
    if(keys[GLFW_KEY_R]) {
        tree_depth--;
        tree_depth = std::max(0, tree_depth);
        kdtrees[RAYTREE]->Build(tree_depth, max_elements);
        kdtrees[COLTREE1]->Build(tree_depth+3, max_elements);
        keys[GLFW_KEY_R] = false;
    }
}

void Application::MouseControls(Camera& camera, MouseMap& buttons,  Mouse& mouse) {
    float mouse_sens = -0.003f;
	glm::vec2 look = mouse.move * mouse_sens;
    if(buttons[GLFW_MOUSE_BUTTON_1]) {
        camera.OrbitYaw(-look.x);
        camera.OrbitPitch(-look.y);
    }
}

void Application::ScrollControls(Camera& camera, double xoffset, double yoffset) {
    float delta = -yoffset * 0.1f;
    camera.distance += delta;
}
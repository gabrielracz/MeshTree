#include "application.h"
#include <functional>
#include "GLFW/glfw3.h"
#include "glm/gtc/random.hpp"
#include "glm/gtx/intersect.hpp"
#include <chrono>
#include <format>
#include <locale>
#include <thread>



const std::vector<float> line_verts = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

void Application::Init(const std::string& objfile1, const std::string& objfile2, const std::string& objfile3) {
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
    ray_mesh = Mesh(objfile1);
    ray_triangles = GetMeshTriangles(ray_mesh);
    kdtrees[RAYTREE] = new KDTree(ray_triangles);
    kdtrees[RAYTREE]->Build(3, 1);

    col_mesh1 = Mesh(objfile2);
    col_transform1.SetPosition({-2.775, -0.5275, 0.37});
    // col_transform1.SetOrientation(glm::angleAxis(-PI/4.0f, glm::vec3(0.0, 1.0, 0.0)));
    col_triangles = GetMeshTriangles(col_mesh1, col_transform1);

    col_mesh2 = Mesh(objfile3);
    col_triangles2 = GetMeshTriangles(col_mesh2, col_transform2);

    kdtrees[COLTREE1] = new KDTree(col_triangles);
    kdtrees[COLTREE1]->Build(tree_depth, 1);

    kdtrees[COLTREE2] = new KDTree(col_triangles2);
    kdtrees[COLTREE2]->Build(tree_depth, 1);
    // kdtree_col
    ray.tmax = 10.0f;

    std::cout << "CONTROLS:\n" <<
        "1 - raycast demo\n" <<
        "2 - collision demo\n" <<
        "3 - raycast benchmark (shift toggles randomzied vs. ordered rays)\n" <<
        "4 - collision benchmark\n" <<
        "5 - naive collision benchmark\n\n" <<

        "WASD   - move camera\n" <<
        "ZX     - increase/decrease camera distance\n" <<
        "MOUSE  - click and drag to rotate camera orbit\n" <<
        "F      - fire ray\n" <<
        "ARROWS - rotate camera orbit\n" <<
        "IJKL   - angle ray\n" <<
        "TR     - increase/decrease tree depth\n" <<
        "Q      - toggle object rendering\n" <<
        "E      - toggle wireframe/fill render mode\n" <<
        "C      - toggle edge rendering on tree" << std::endl;
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
        case Scene::COLLISIONNAIVE:
            UpdateCollisionNaive();
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
    KDTree::TreeIntersect(*kdtrees[COLTREE2], *kdtrees[COLTREE1], &nodeinter);

    if(render_obj) {
        view.RenderObj(col_transform2, col_mesh2, mesh_shader, light, nodeinter.triangle_id1);
        view.RenderObj(col_transform1, col_mesh1, mesh_shader, light, nodeinter.triangle_id2);
    }

    RenderKDTree(&kdtrees[COLTREE2]->GetTree(), nodeinter.node_id1);
    RenderKDTree(&kdtrees[COLTREE1]->GetTree(), nodeinter.node_id2);
}

void Application::UpdateRaycastBenchmark() {

    view.RenderObj(ray_transform, ray_mesh, mesh_shader, light);
    view.Update();

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
        if(random_rays) {
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
    Transform test_transform;
    test_transform.SetPosition(glm::ballRand(1.5f));
    test_transform.SetOrientation(glm::angleAxis(glm::linearRand(-PI/2.0f, PI/2.0f), glm::ballRand(1.0f)));

    view.RenderObj(test_transform, col_mesh1, mesh_shader, light);
    view.RenderObj(col_transform2, col_mesh2, mesh_shader, light);
    view.Update();
    std::vector<Triangle> test_triangles = GetMeshTriangles(col_mesh1, test_transform);
    KDTree test_tree(test_triangles);
    test_tree.Build(tree_depth, 10);

    int tree_elapsed = 0;
    int naive_elapsed = 0;
    bool tree_hit = false;
    bool naive_hit = false;
    std::chrono::steady_clock::time_point start_time, end_time;


    std::cout.imbue(std::locale(""));
    std::cout << "Collision benchmark [depth " << tree_depth+3 << " vs. " << tree_depth << "] [triangles: " << col_triangles2.size() << " vs. " << test_triangles.size() << "]" << std::endl;    
    // accelerated intersection
    NodeIntersection nodeisect;
    start_time = std::chrono::steady_clock::now();
    tree_hit = KDTree::TreeIntersect(test_tree, *kdtrees[COLTREE2], &nodeisect);
    end_time = std::chrono::steady_clock::now();
    tree_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << std::boolalpha;
    std::cout << tree_elapsed << " ms hit " << std::to_string(tree_hit) << "\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void Application::UpdateCollisionNaive() {
    Transform test_transform;
    test_transform.SetPosition(glm::ballRand(1.5f));
    test_transform.SetOrientation(glm::angleAxis(glm::linearRand(-PI/2.0f, PI/2.0f), glm::ballRand(1.0f)));

    view.RenderObj(test_transform, col_mesh1, mesh_shader, light);
    view.RenderObj(col_transform2, col_mesh2, mesh_shader, light);
    view.Update();
    std::vector<Triangle> test_triangles = GetMeshTriangles(col_mesh1, test_transform);
    KDTree test_tree(test_triangles);
    test_tree.Build(tree_depth, 10);

    int elapsed = 0;
    bool hit = false;
    std::chrono::steady_clock::time_point start_time, end_time;
    std::cout << "Naive collision search [depth " << tree_depth+3 << " vs. " << tree_depth << "] [triangles: " << col_triangles2.size() << " vs. " << test_triangles.size() << "]" << std::endl;    
    start_time = std::chrono::steady_clock::now();
    for(Triangle& t1 : test_triangles) {
        for(Triangle& t2 : col_triangles2) {
            if(t1.Intersect(t2)) {
                hit = true;
                goto search_exit;
            }
        }
    }
search_exit:
    end_time = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << std::boolalpha;
    std::cout << elapsed << " ms hit " << std::to_string(hit) << "\n" << std::endl;
    active_scene = Scene::COLLISIONBENCHMARK;
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
        if(keys[GLFW_KEY_LEFT_SHIFT]) {
            random_rays = true;
        } else {
            random_rays = false;
        }
        active_scene = Scene::RAYCASTBENCHMARK;
        keys[GLFW_KEY_3] = false;
    }
    if(keys[GLFW_KEY_4]) {
        active_scene = Scene::COLLISIONBENCHMARK;
        keys[GLFW_KEY_4] = false;
    }
    if(keys[GLFW_KEY_5]) {
        active_scene = Scene::COLLISIONNAIVE;
        keys[GLFW_KEY_5] = false;
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
        kdtrees[COLTREE2]->Build(tree_depth+3, max_elements);
        keys[GLFW_KEY_T] = false;
    }
    if(keys[GLFW_KEY_R]) {
        tree_depth--;
        tree_depth = std::max(0, tree_depth);
        kdtrees[RAYTREE]->Build(tree_depth, max_elements);
        kdtrees[COLTREE1]->Build(tree_depth+3, max_elements);
        kdtrees[COLTREE2]->Build(tree_depth+3, max_elements);
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
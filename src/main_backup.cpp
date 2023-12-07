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
void ScrollControls(Camera& camera, double xoffset, double yoffset);
void RenderKDTree(View& view, Shader& shader, KDNode* tree);
void RenderRay(View& view, Mesh& line_mesh, Shader& line_shader, Ray& ray);
std::vector<Triangle> GetMeshTriangles(Mesh& mesh, Transform transform = {});

const std::vector<float> line_verts = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

KDTree* tree = nullptr;
KDTree* tree2 = nullptr;
int tree_depth = 6;
int max_elements = 10;
bool render_obj = true;
int hit_triangle = -1;
Ray ray;
int draw_edges = 1;
glm::ivec2 hit_nodes = {-1, -1};

template <typename T>
std::vector<T> concatenate(std::vector<T>& A, std::vector<T>& B) {
    std::vector<T> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
}

int main(void){
    // RENDERING
    View view;
    view.Init("[] MeshTree - Gabriel Racz (c)", 800, 800);

    Shader    mesh_shader(SHADER_DIRECTORY"/lit_vp.glsl", SHADER_DIRECTORY"/lit_fp.glsl");
    Shader box_shader(SHADER_DIRECTORY"/box_vp.glsl", SHADER_DIRECTORY"/box_fp.glsl");
    Shader line_shader(SHADER_DIRECTORY"/line_vp.glsl", SHADER_DIRECTORY"/line_fp.glsl");
    Mesh line_mesh(line_verts, {}, {{FLOAT3, "position"}});

    Light light({0.9, 0.9, 0.9, 1.0});
    light.transform.SetPosition({10.0, 10.0, 0.0});

    Camera& camera = view.GetCamera();
    Transform bunny_transform; // dummy transform at
    camera.Attach(&bunny_transform);
    camera.OrbitYaw(PI/2.0f);
    light.Attach(&camera.transform);

    MouseMap& mouse_buttons = view.GetMouseButtons();

    auto mouse_controls = [&camera, &mouse_buttons](Mouse& mouse) {
        MouseControls(camera, mouse_buttons, mouse);
    };
    view.SetMouseHandler(mouse_controls);
    auto scroll_controls = [&camera](double xoffset, double yoffset) {
        ScrollControls(camera, xoffset, yoffset);
    };
    view.SetScrollHandler(scroll_controls);


    // MESHTREE
    Mesh      bunny_mesh(RESOURCES_DIRECTORY"/bunny_full.obj");
    std::vector<Triangle> triangles = GetMeshTriangles(bunny_mesh);
    KDTree kdtree(triangles);
    kdtree.Build(3, 1);
    tree = &kdtree;

    Mesh      dragon_mesh(RESOURCES_DIRECTORY"/dragon.obj");
    Transform dragon_transform;
    dragon_transform.SetPosition({-3.2, 0.0, 0.7});
    std::vector<Triangle> dragon_triangles = GetMeshTriangles(dragon_mesh, dragon_transform);

    KDTree kdtree_col(dragon_triangles);
    kdtree_col.Build(tree_depth, 1);
    tree2 = &kdtree_col;
    // kdtree_col

    ray.tmax = 10.0f;

    while(!view.Closed()) {
        CheckControls(view.GetKeys(), view, camera);

        view.Clear();
        camera.Update();
        if(render_obj) {
            view.RenderObj(bunny_transform, bunny_mesh, mesh_shader, light, hit_triangle);
            view.RenderObj(dragon_transform, dragon_mesh, mesh_shader, light);
        }

        // Render ray-intersection
        Intersection intersection;
        if(kdtree.RayIntersect(ray, &intersection)) {
            glm::vec3 point = ray.direction * intersection.t0;
            view.RenderLine(line_mesh, line_shader, ray.origin, point, {1.0, 0.0, 0.0, 1.0});
            hit_triangle = intersection.triangle_id;
        } else {
            view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
            hit_triangle = -1;
        }

        //Render tree-intersections
        NodeIntersection nodeinter;
        if(KDTree::TreeIntersect(kdtree, kdtree_col, &nodeinter)) {
            hit_triangle = nodeinter.triangle_id1;
            hit_nodes.x = nodeinter.node_id1;
            hit_nodes.y = nodeinter.node_id2;
        }


        RenderKDTree(view, box_shader, &kdtree.GetTree());
        RenderKDTree(view, box_shader, &kdtree_col.GetTree());
        view.Update();
    }

    return 0;
}

void RenderKDTree(View& view, Shader& shader, KDNode* node) {
    if(node == nullptr) {
        return;
    }
    glm::vec4 color = {1.0, 1.0, 1.0, 0.05f/(tree_depth+1)};
    if(node->id == hit_nodes.x || node->id == hit_nodes.y) {
        color = {1.0, 0.0, 0.0, 0.25};
    }
    view.RenderBox(shader, node->aabb.min, node->aabb.max, color, draw_edges);
    RenderKDTree(view, shader, node->left_child);
    RenderKDTree(view, shader, node->right_child);
}

void RenderRay(View& view, Mesh& line_mesh, Shader& line_shader, Ray& ray) {
    view.RenderLine(line_mesh, line_shader, ray.origin, ray.direction * ray.tmax, {1.0, 0.0, 0.0, 1.0});
}   

std::vector<Triangle> GetMeshTriangles(Mesh& mesh, Transform transform) {
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
        tree->Build(tree_depth, max_elements);
        tree2->Build(tree_depth+3, max_elements);
        keys[GLFW_KEY_T] = false;
    }
    if(keys[GLFW_KEY_R]) {
        tree_depth--;
        tree_depth = std::max(0, tree_depth);
        tree->Build(tree_depth, max_elements);
        tree2->Build(tree_depth+3, max_elements);
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

void ScrollControls(Camera& camera, double xoffset, double yoffset) {
    float delta = -yoffset * 0.1f;
    camera.distance += delta;
}
#ifndef APPLICATION_H
#define APPLICATION_H

#include <iostream>

// rendering engine
#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

// MeshTree
#include "structures.h"
#include "kdtree.h"

class Application {
    enum class Scene {
        RAYSCASTDEMO,
        COLLISIONDEMO,
        RAYCASTBENCHMARK,
        COLLISIONBENCHMARK
    };

    enum Trees {
        RAYTREE = 0,
        COLTREE1,
        COLTREE2,
        NUM_TREES
    };

public:
    Application() = default;
    void Init();
    void Update();
    bool Closed() {return view.Closed();}

private:

    void UpdateRaycastDemo();
    void UpdateCollisionDemo();
    void UpdateRaycastBenchmark();
    void UpdateCollisionBenchmark();

    void CheckControls();
    void MouseControls(Camera& camera, MouseMap& mouse_buttons, Mouse& mouse);
    void ScrollControls(Camera& camera, double xoffset, double yoffset);
    void RenderKDTree(KDNode* tree, int hit_node = -1);
    std::vector<Triangle> GetMeshTriangles(Mesh& mesh, Transform transform = {});

    Scene active_scene = Scene::RAYSCASTDEMO;

    View view;
    Light light;

    Shader mesh_shader;
    Shader box_shader;
    Shader line_shader;

    Mesh line_mesh;
    Mesh ray_mesh;
    Mesh col_mesh1;
    Mesh col_mesh2;

    std::vector<Triangle> ray_triangles;
    std::vector<Triangle> col_triangles;
    std::vector<Triangle> col_triangles2;

    Transform ray_transform;
    Transform col_transform1;
    Transform col_transform2;

    Ray ray;
    KDTree* kdtrees[NUM_TREES] = {nullptr};

    int tree_depth = 6;
    int max_elements = 10;
    bool render_obj = true;
    int search_depth = KDTree::COMPLETE_SEARCH_DEPTH;
    int draw_edges = 1;
    bool closed = false;
};


#endif
#include <iostream>
#include <exception>
#include <glm/gtc/random.hpp>

// rendering engine
#include "path_config.h"
#include "view.h"
#include "mesh.h"
#include "shader.h"

// MeshTree
#include "structures.h"
#include "kdtree.h"

#define RANDV(limit) glm::vec3(static_cast<float>(rand()) / RAND_MAX * ((float)limit), \
                               static_cast<float>(rand()) / RAND_MAX * ((float)limit), \
                               static_cast<float>(rand()) / RAND_MAX * ((float)limit))
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
    View view;
    view.Init("MeshTree", 800, 800);
    Mesh      bunny_mesh(RESOURCES_DIRECTORY"/dragon.obj");

    // MESHTREE
    std::vector<Triangle>     triangles;
    std::vector<glm::vec3>    vertices = bunny_mesh.GetVertices();
    std::vector<unsigned int> indices = bunny_mesh.GetIndices();
    for(int i = 0; i < indices.size() - 2; i += 3) {
        triangles.emplace_back(Triangle({vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]]}));
    } 

    KDTree kdtree(triangles);
    srand(1337);
    kdtree.Build(30, 10);
    tree = &kdtree;

    int num_rays = 100'00;
    float ray_bound = 3.0f;
    std::vector<Ray> rays;
    for(int i = 0; i < num_rays; i++) {
        Ray ray;
        ray.origin = RANDV(ray_bound);
        ray.direction = RANDV(ray_bound);
        ray.tmax = 2*ray_bound;
        rays.push_back(ray);
    }
    std::cout << "generated" << std::endl;

    Intersection intersection;
    int hits = 0;
    bool accelerated = true;
    if(accelerated) {
        for(Ray& ray : rays) {
            if(kdtree.RayIntersect(ray, &intersection)) {
                hits++;
            }
        }
    } else {
        for(Ray& ray : rays) {
            float min_time = INF;
            for(Triangle& tri : triangles) {
                float t = 0;
                if(tri.Intersect(ray, &t)) {
                    if(t < min_time) {
                        min_time = t;
                    }
                }
            }
            if(min_time != INF) {
                hits++;
            }
        }
    }

    std::cout << hits << std::endl;
    return 0;
}

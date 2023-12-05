#ifndef KDTREE_H
#define KDTREE_H
#include <vector>
#include <unordered_map>
#include "structures.h"

const int INTERIOR_NODE = 0;

struct KDNode {
    int leaf_id = INTERIOR_NODE;
    AABB aabb;
    KDNode* right_child = nullptr;
    KDNode* left_child = nullptr;
};

class KDTree {

public:
    KDTree(std::vector<Triangle>& tris);
    void Build(int max_depth, int max_triangles);
    bool RayIntersect(Ray& r, Intersection* intersection);

    static bool TreeIntersect(KDTree& tree1, KDTree& tree2, NodeIntersection* intersection);
    KDNode& GetTree() {return nodes.front();}
    std::vector<int> GetLeafTriangleIndices(int leaf_id);

private:
    void BuildTree(KDNode& node, std::vector<Triangle> contained_tris, std::vector<int> tri_indices, int current_depth);
    bool RayTraverse(KDNode& node, Ray& ray, Intersection* intersection);
    static bool TreeTraverse(KDTree& tree1, KDTree& tree2, KDNode& node1, KDNode& node2, NodeIntersection* intersection);
    static int GetLeafID();
    static float SplitSurfaceAreaHeuristic(std::vector<Triangle>& tris, Axis* axis);


    // helpers
    static Axis GetLargestAxis(AABB& aabb);
    static glm::vec3 MinimumTriangleVertex(std::vector<Triangle>& tris, Axis axis);
    static glm::vec3 MaximumTriangleVertex(std::vector<Triangle>& tris, Axis axis);

    std::vector<Triangle>& triangles; // non-owning
    std::vector<KDNode> nodes;
    std::unordered_map<int, std::vector<int>> leaf_triangle_map; //maps leaf_ids to triangles contained within that leaf node

    int max_depth = 0;
    int max_elements = 0;
    static int leaf_id_counter;

};

#endif
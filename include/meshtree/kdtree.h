#ifndef KDTREE_H
#define KDTREE_H
#include <vector>
#include <unordered_map>
#include "structures.h"

struct KDNode {
    int leaf_id = 0;
    AABB aabb;
    KDNode* right_child = nullptr;
    KDNode* left_child = nullptr;
};

class KDTree {

public:
    KDTree(std::vector<Triangle>& tris);
    void Build(int max_depth, int max_triangles);
    Intersection RayIntersect(Ray& r);
    KDNode& GetTree() {return nodes.front();}

private:
    void BuildTree(KDNode& node, std::vector<Triangle> contained_tris, std::vector<int> tri_indices, int current_depth);
    int GetLeafID();
    static Axis GetLargestAxis(AABB& aabb);
    static float SplitSurfaceAreaHeuristic(std::vector<Triangle>& tris, Axis* axis);

    std::vector<Triangle>& triangles; // non-owning
    std::vector<KDNode> nodes;
    std::unordered_map<int, std::vector<int>> leaf_triangle_map; //maps leaf_ids to triangles contained within that leaf node

    int max_depth = 0;
    int max_elements = 0;
    int leaf_id_counter = 1;

};

#endif
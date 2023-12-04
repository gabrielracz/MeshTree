#ifndef KDTREE_H
#define KDTREE_H
#include <vector>
#include "structures.h"

struct KDNode {
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
    void BuildTree(KDNode& node, std::vector<Triangle> contained_tris, int current_depth);
    static Axis GetLargestAxis(AABB& aabb);
    static float SplitSurfaceAreaHeuristic(std::vector<Triangle>& tris, Axis* axis);

    std::vector<Triangle>& triangles; // non-owning
    std::vector<KDNode> nodes;

    int max_depth = 0;
    int max_elements = 0;

};

#endif
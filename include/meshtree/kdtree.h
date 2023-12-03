#ifndef KDTREE_H
#define KDTREE_H
#include <vector>
#include "structures.h"

struct KDNode {
    AABB bounds;
    KDNode* child1 = nullptr;
    KDNode* child2 = nullptr;
};

class KDTree {
public:
    KDTree(std::vector<Triangle>& tris) : triangles(tris) {}
    void Build(int max_depth, int max_triangles);
    Intersection RayIntersect(Ray& r);
    KDNode& GetTree() {return root;}

private:
    std::vector<Triangle>& triangles; // non-owning
    KDNode root;

};

#endif
#ifndef KDTREE_H
#define KDTREE_H
#include <vector>
#include "structures.h"

class KDTree {
public:
    KDTree(std::vector<Triangle>& tris) : triangles(tris) {}
    void Build(int max_depth, int max_triangles);
    Intersection RayIntersect(Ray& r);

private:
    std::vector<Triangle>& triangles; // non-owning
};

#endif
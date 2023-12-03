#include <iostream>
#include <glm/gtx/string_cast.hpp>

#include "kdtree.h"


void KDTree::Build(int max_depth, int max_triangles) {
    // Create a bounding box that fully captures the triangles
    glm::vec3 min_extent( INF);
    glm::vec3 max_extent(-INF);
    for(const Triangle& tri : triangles) {
        for(const glm::vec3& v : tri.vertices) {
            min_extent.x = std::min(min_extent.x, v.x);
            min_extent.y = std::min(min_extent.y, v.y);
            min_extent.z = std::min(min_extent.z, v.z);

            max_extent.x = std::max(max_extent.x, v.x);
            max_extent.y = std::max(max_extent.y, v.y);
            max_extent.z = std::max(max_extent.z, v.z);
        }
    }
    AABB root_box(min_extent, max_extent);
    root.bounds = root_box;

    return;
}
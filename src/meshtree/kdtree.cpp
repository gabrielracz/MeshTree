#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <algorithm>

#include "kdtree.h"

KDTree::KDTree(std::vector<Triangle>& tris) : triangles(tris) {}


// Depth 0 - single AABB
// Depth n - subdivide n times
void KDTree::Build(int depth, int max_triangles) {
    max_depth = depth;
    max_elements = max_triangles;
    nodes.resize(2*max_depth + 1);

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
    int root_index = 0;
    AABB root_box(min_extent, max_extent);
    nodes[root_index].aabb = root_box;

    BuildTree(nodes[root_index], triangles, 0);

    return;
}


void KDTree::BuildTree(KDNode& node, std::vector<Triangle> contained_tris, int current_depth) {
    if(current_depth > max_depth || contained_tris.size() < max_elements) {
        return;
    }


    // select a plane to split
    Axis split_plane = GetLargestAxis(node.aabb);
    std::sort(contained_tris.begin(), contained_tris.end(), [split_plane](Triangle a, Triangle b) {
        return a.centroid[split_plane] < b.centroid[split_plane];
    });
    //find split

    // Axis split_plane = Axis::XAXIS;
    float split_point = SplitSurfaceAreaHeuristic(contained_tris, &split_plane);
    std::cout << split_point << " " << split_plane << std::endl;
    std::vector<Triangle> left_child_tris;
    std::vector<Triangle> right_child_tris;
    for(Triangle& t : contained_tris) {
        bool left = false; // true if to the left, right otherwise
        bool right = false;
        for(const glm::vec3& v : t.vertices) {
            if(v[split_plane] < split_point) {
                left = true;
            } else {
                right = true;
            }
        }
        
        // TODO: make these triangle IDs and not real triangles
        // could be straddling, support adding to both
        if(left) {
            left_child_tris.push_back(t);
        }
        if(right) {
            right_child_tris.push_back(t);
        }
    }


    KDNode* right_child = new KDNode();
    KDNode* left_child = new KDNode();

    // configure child boxes
    right_child->aabb = node.aabb;
    right_child->aabb.min[split_plane] = split_point;

    left_child->aabb = node.aabb;
    left_child->aabb.max[split_plane] = split_point;

    node.right_child = right_child;
    node.left_child = left_child;

    // std::cout << "SPLIT: " << left_child_tris.size() << " " << right_child_tris.size() << std::endl;

    current_depth += 1;
    BuildTree(*right_child, right_child_tris, current_depth);
    BuildTree(*left_child, left_child_tris, current_depth);
}

// return split index along this axis that maximizes SAH
float KDTree::SplitSurfaceAreaHeuristic(std::vector<Triangle>& tris, Axis* optimal_axis) {

    float traversal_cost = 1;
    float intersect_cost = 80;

    float min_cost = INF;
    float best_split = 0;



    // try every axis to find the best split
    Axis axis = *optimal_axis;
    // for(int axis = 0; axis < 3; axis++) {
        // threshold, not indices

        float start = tris.front().vertices[0][axis];
        float end = tris.back().vertices[0][axis];
        float num_steps = 64;
        float step_size = (glm::abs(start) + glm::abs(end))/num_steps;

        // for(float split : possible_splits) {
        // for(float split : possible_splits) {
        for(float split = start; split < end; split += step_size) {
            int Acount = 0;
            float ASA    = 0;
            int Bcount = 0;
            float BSA    = 0;
            // calculate partioned surface areas
            for(Triangle& t : tris) {
                if(t.centroid[axis] < split) { // left of split
                    Acount++;
                    ASA += t.SurfaceArea();
                    // ASA += t.Bounds(axis);
                } else {                       // right of split
                    Bcount++;
                    BSA += t.SurfaceArea();
                    // BSA += t.Bounds(axis);
                }
            }

            // float empty_bonus = (Acount == 0 || Bcount == 0) ? 0.9 : 0;
            float empty_bonus = 0.0;

            float SA = ASA + BSA; // total surface area
            float cost = traversal_cost + (1-empty_bonus) * intersect_cost * (Acount * ASA + Bcount*BSA) / SA;

            if(cost < min_cost) {
                min_cost = cost;
                best_split = split;
                // *optimal_axis = static_cast<Axis>(axis);
            }
        }
    // }
    return best_split;
}

Axis KDTree::GetLargestAxis(AABB& aabb) {
    glm::vec3 diff = glm::abs(aabb.max - aabb.min);
    if(diff.x > diff.y && diff.x > diff.z) {
        return Axis::XAXIS;
    } else if (diff.y > diff.z) {
        return Axis::YAXIS;
    } else {
        return Axis::ZAXIS;
    }
}
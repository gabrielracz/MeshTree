#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <algorithm>
#include <numeric>

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

    std::vector<int> all_triangle_indices(triangles.size());
    std::iota(all_triangle_indices.begin(), all_triangle_indices.end(), 0);

    BuildTree(nodes[root_index], triangles, all_triangle_indices, 0);

    return;
}


void KDTree::BuildTree(KDNode& node, std::vector<Triangle> contained_tris, std::vector<int> tri_indices, int current_depth) {
    if(current_depth > max_depth || contained_tris.size() < max_elements) {
        // make this a leaf
        node.leaf_id = GetLeafID();
        leaf_triangle_map.emplace(node.leaf_id, tri_indices);
        return;
    }


    // select a plane to split
    Axis split_plane = GetLargestAxis(node.aabb);
    float split_point = SplitSurfaceAreaHeuristic(contained_tris, &split_plane);
    node.aabb.split_axis = split_plane;
    node.aabb.split_point = split_point;

    std::vector<Triangle> left_child_tris;
    std::vector<int> left_child_indices;
    std::vector<Triangle> right_child_tris;
    std::vector<int> right_child_indices;

    int i = 0;
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
            left_child_indices.push_back(i);

        }
        if(right) {
            right_child_tris.push_back(t);
            right_child_indices.push_back(i);
        }
        i++;
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

    current_depth += 1;
    BuildTree(*right_child, right_child_tris, right_child_indices, current_depth);
    BuildTree(*left_child, left_child_tris, left_child_indices, current_depth);
}

// return split index along this axis that maximizes SAH
float KDTree::SplitSurfaceAreaHeuristic(std::vector<Triangle>& tris, Axis* optimal_axis) {

    float traversal_cost = 1.0;
    float intersect_cost = 70.0;

    float min_cost = INF;
    float best_split = 0;

    // try every axis to find the best split
    Axis axis = *optimal_axis;
    // for(Axis axis = XAXIS; axis < 3; axis = static_cast<Axis>(axis + 1)) {

        // float start = tris.front().vertices[0][axis];
        // float end = tris.back().vertices[0][axis];

        float start = MinimumTriangleVertex(tris, axis)[axis];
        float end = MaximumTriangleVertex(tris, axis)[axis];
        float num_steps = 32;
        float step_size = (glm::abs(start) + glm::abs(end))/num_steps;
        // float step_size = 0.03f; // estimate the min size of a triangle

        for(float split = start; split < end; split += step_size) {
            int Acount = 0;
            float ASA    = 0;
            int Bcount = 0;
            float BSA    = 0;
            // calculate partioned surface areas
            for(Triangle& t : tris) {
                if(t.centroid[axis] < split) { // left of split
                    Acount++;
                    // ASA += t.SurfaceArea();
                    ASA += t.Bounds(axis);
                } else {                       // right of split
                    Bcount++;
                    // BSA += t.SurfaceArea();
                    BSA += t.Bounds(axis);
                }
            }

            // reward making one of the partitions completely empty
            float empty_bonus = (Acount == 0 || Bcount == 0) ? 0.49 : 0.0;

            float SA = ASA + BSA; // total surface area
            float PA = ASA/SA;
            float PB = BSA/SA;
            float cost = traversal_cost + (1.0f-empty_bonus) * intersect_cost * (PA * Acount + PB * Bcount);

            if(cost < min_cost) {
                min_cost = cost;
                best_split = split;
                *optimal_axis = static_cast<Axis>(axis);
            }
        }
    // }
    return best_split;
}

bool KDTree::RayIntersect(Ray &ray, Intersection* intersect) {
    float t0, t1;
    KDNode& root = nodes[0];

    bool hit_root_box = root.aabb.Intersect(ray, &t0, &t1);
    if(!hit_root_box) {
        return false;
    }
    std::cout << "hit root" << std::endl;
    return RayTraverse(root, ray, intersect);
}

bool KDTree::RayTraverse(KDNode& node, Ray& ray, Intersection* intersect) {
    // Check leaf node 
    if(node.leaf_id != INTERIOR_NODE) {
        float time = 0.0;
        float min_time = INF;
        int earliest_triangle = 0;
        std::vector<int> contained_tri_indices = leaf_triangle_map[node.leaf_id];
        for(int i : contained_tri_indices) {
            if(triangles[i].Intersect(ray, &time) && time < min_time) {
                min_time = time;
                earliest_triangle = i;
            }
        }

        if(min_time == INF) {   // either contains no triangles or the ray hit none of them
            return false;
        }

        intersect->triangle_id = earliest_triangle;
        intersect->t0 = min_time;
        return true;
    }

    // Descend interior node children
    // Check against their bounding boxes
    float tleft0 = 0.0, tleft1, tright0, tright1 = 0.0;
    bool left_hit = false, right_hit = false;
    if(node.left_child != nullptr) {
        left_hit = node.left_child->aabb.Intersect(ray, &tleft0, &tleft1);
        std::cout << "hit left" << std::endl;
    }
    if(node.right_child != nullptr) {
        right_hit = node.right_child->aabb.Intersect(ray, &tright0, &tright1);
        std::cout << "hit right" << std::endl;
    }
    std::cout << std::endl;

    if(!left_hit && !right_hit) { // neither child was hit, nothing to do here.
        return false;
    }

    // Find the node that was hit first
    KDNode* first = nullptr;
    KDNode* second = nullptr;

    if(left_hit && right_hit) { // both children hit, traverse the smaller one first.
        if(tleft0 < tright0) {
            first = node.left_child;
            second = node.right_child;
        } else {
            first = node.right_child;
            second = node.left_child;
        }
    } else if (left_hit) {
        first = node.left_child;
    } else {                   // we made sure at least one was hit above
        first = node.right_child;
    }

    if(RayTraverse(*first, ray, intersect)) {
        return true;
    } else {
        if(second != nullptr) {
            return RayTraverse(*second, ray, intersect);
        }
    }
    // unreachable
    return false;
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

int KDTree::GetLeafID() {
    return leaf_id_counter++;
}

glm::vec3 KDTree::MinimumTriangleVertex(std::vector<Triangle>& tris, Axis axis) {
    glm::vec3 min = tris.front().vertices[0];
    for(Triangle& t : tris){ 
        glm::vec3 tmin = t.MinVertex(axis);
        if(tmin[axis] < min[axis]) {
            min = tmin;
        }
    }
    return min;
}

glm::vec3 KDTree::MaximumTriangleVertex(std::vector<Triangle>& tris, Axis axis) {
    glm::vec3 max = tris.front().vertices[0];
    for(Triangle& t : tris){ 
        glm::vec3 tmax = t.MaxVertex(axis);
        if(tmax[axis] > max[axis]) {
            max = tmax;
        }
    }
    return max;
}
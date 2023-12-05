#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <algorithm>
#include <numeric>

#include "kdtree.h"

int KDTree::node_id_counter = 1;

KDTree::KDTree(std::vector<Triangle>& tris) : triangles(tris) {}

// Depth 0 - single AABB
// Depth n - subdivide n times
void KDTree::Build(int depth, int max_triangles) {
    nodes.clear();
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
    nodes[root_index].id = CreateNodeID();

    std::vector<int> all_triangle_indices(triangles.size());
    std::iota(all_triangle_indices.begin(), all_triangle_indices.end(), 0);

    BuildTree(nodes[root_index], triangles, all_triangle_indices, 1);

    return;
}


void KDTree::BuildTree(KDNode& node, std::vector<Triangle> contained_tris, std::vector<int> tri_indices, int current_depth) {
    if(current_depth > max_depth || contained_tris.size() < max_elements) {
        // make this a leaf
        node.isleaf = true;
        leaf_triangle_map.emplace(node.id, tri_indices);
        return;
    };

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
            left_child_indices.push_back(tri_indices[i]);

        }
        if(right) {
            right_child_tris.push_back(t);
            right_child_indices.push_back(tri_indices[i]);
        }
        i++;
    }

    KDNode* right_child = new KDNode();
    KDNode* left_child = new KDNode();
    right_child->id = CreateNodeID();
    left_child->id = CreateNodeID();

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

    float traversal_cost = 2.0;
    float intersect_cost = 80.0;

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
        if(tris.size() < 32) {num_steps = 2;}
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
            float empty_bonus = (Acount == 0 || Bcount == 0) ? 0.40 : 0.0;

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
    return RayTraverse(root, ray, intersect);
}


bool KDTree::RayTraverse(KDNode& node, Ray& ray, Intersection* intersect) {
    // Check leaf node 
    if(node.isleaf) {
        float time = 0.0;
        float min_time = INF;
        int earliest_triangle = 0;
        std::vector<int> contained_tri_indices = leaf_triangle_map[node.id];
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
    }
    if(node.right_child != nullptr) {
        right_hit = node.right_child->aabb.Intersect(ray, &tright0, &tright1);
    }

    if(!left_hit && !right_hit) { // neither child was hit, nothing to do here.
        return false;
    }

    float tleft = std::min(tleft0, tleft1);
    float tright = std::min(tright0, tright1);

    // Find the node that was hit first
    KDNode* first = nullptr;
    KDNode* second = nullptr;

    if(left_hit && right_hit) { // both children hit, traverse the smaller one first.
        if(tleft < tright) {
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

bool KDTree::TreeIntersect(KDTree& tree1, KDTree& tree2, int search_depth, NodeIntersection* intersection) {
    KDNode& root1 = tree1.GetTree();
    KDNode& root2 = tree2.GetTree();

    bool root_boxes_overlap = root1.aabb.Intersect(root2.aabb);
    if(!root_boxes_overlap) {
        std::cout << "roots dont overlap" << std::endl;
        return false;
    }
    return TreeTraverse(tree1, tree2, root1, root2, search_depth, 0, intersection);
}

bool KDTree::TreeTraverse(KDTree& tree1, KDTree& tree2, KDNode& node1, KDNode& node2, int search_depth, int current_depth, NodeIntersection* intersection) {
    // traverse until we reach a leaf node on both trees
    if(node1.isleaf && node2.isleaf) {
        // check for overlapping triangles
        for(int i : tree1.GetLeafTriangleIndices(node1.id)) {
            for(int j : tree2.GetLeafTriangleIndices(node2.id)) {
                if(tree1.triangles[i].Intersect(tree2.triangles[j])) {
                    intersection->node_id1 = node1.id;
                    intersection->node_id2 = node2.id;
                    intersection->triangle_id1 = i;
                    intersection->triangle_id2 = j;
                    return true;
                }
            }
        }
        return false;
    }

    // interior node(s) but we reached the maximum requested search depth
    if(current_depth > search_depth) {
        if(node1.isleaf || node2.isleaf) {
            KDNode* leaf;
            KDTree* leaf_tree;
            KDNode* interior;
            if(node1.isleaf) {
                leaf = &node1;
                leaf_tree = &tree1;
                interior = &node2;
            } else {
                leaf = &node2;
                leaf_tree = &tree2;
                interior = &node2;
            }

            for(int i : leaf_tree->GetLeafTriangleIndices(leaf->id)) {
                if(interior->aabb.Intersect(leaf_tree->triangles[i])) {
                    // the leaf's triangle intersects the interior's aabb
                    intersection->triangle_id1 = i; 
                    intersection->node_id1 = leaf->id;
                    intersection->node_id2 = interior->id;
                    return true;
                }
            }
        }else {
            // these two nodes are guarenteed to be overlapping as we descended into them.
            intersection->node_id1 = node1.id;
            intersection->node_id2 = node2.id;
            return true;
        }

    }

    // TODO: add some heuristic that improves searching in the right direction
    current_depth++;
    // cover case where one is already a leaf node
    if(node1.isleaf || node1.isleaf) {
        KDNode& leaf     = node1.isleaf ? node1 : node2;
        KDNode& interior = node1.isleaf ? node2 : node1;
        if(interior.left_child && leaf.aabb.Intersect(interior.left_child->aabb)) {
            if(TreeTraverse(tree1, tree2, leaf, *interior.left_child, search_depth, current_depth, intersection)) { return true; }
        }
        if(interior.right_child && leaf.aabb.Intersect(interior.right_child->aabb)) {
            if(TreeTraverse(tree1, tree2, leaf, *interior.right_child, search_depth, current_depth, intersection)) { return true; }
        }
        // if neither of the interior branches resulted in an intersection, this leaf does not have any
        return false;
    }

    // both nodes are interior
    KDNode* children1[] = {node1.left_child, node1.right_child};
    KDNode* children2[] = {node2.left_child, node2.right_child};
    for(KDNode* child1 : children1) {
        for(KDNode* child2 : children2) {
            if(child1 && child2 && child1->aabb.Intersect(child2->aabb)) {
                if(TreeTraverse(tree1, tree2, *child1, *child2, search_depth, current_depth, intersection)) {
                    return true;
                }
            }
        }
    }
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

int KDTree::CreateNodeID() {
    // TODO: guard with mutex if multithreading
    return node_id_counter++;
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

std::vector<int> KDTree::GetLeafTriangleIndices(int leaf_id) {
    auto it = leaf_triangle_map.find(leaf_id);
    if(it != leaf_triangle_map.end()) {
        return it->second;
    } else {
        std::cout << "error, invalid leaf id " << leaf_id << std::endl;
        return {};
    }
}
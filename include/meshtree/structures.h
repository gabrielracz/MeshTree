#ifndef STRUCTURES_H
#define STRUCTURES_H
#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <initializer_list>
#include <array>
#include <vector>
#include <algorithm>

#include "triangle-triangle.h"

#define INF std::numeric_limits<float>::infinity()

enum Axis {
    XAXIS = 0,
    YAXIS = 1,
    ZAXIS = 2
};

struct Ray {
    glm::vec3 origin {};
    glm::vec3 direction {};
    float tmax = 0.0f; // maximum time along the direction
};

struct Triangle {
    std::array<glm::vec3, 3> vertices = {};
    glm::vec3 centroid = {};

    Triangle(std::array<glm::vec3, 3> const& verts) : vertices(verts), centroid((vertices[0] + vertices[1] + vertices[2])/3.0f) {}

    float SurfaceArea() {
        glm::vec3 side1 = vertices[1] - vertices[0];
        glm::vec3 side2 = vertices[2] - vertices[0];
        float area = 0.5f * glm::length(glm::cross(side1, side2));
        return area;
    }

    glm::vec3 MinVertex(Axis axis) {
        glm::vec3 min =  *std::min_element(vertices.begin(), vertices.end(), [axis](glm::vec3 a, glm::vec3 b) {
            return a[axis] < b[axis];
        });
        return min;
    }
    glm::vec3 MaxVertex(Axis axis) {
        glm::vec3 max =  *std::max_element(vertices.begin(), vertices.end(), [axis](glm::vec3 a, glm::vec3 b) {
            return a[axis] < b[axis];
        });
        return max;
    }

    float Bounds(Axis axis) {
        glm::vec3 min = MinVertex(axis);
        glm::vec3 max = MaxVertex(axis);

        return glm::abs(max[axis] - min[axis]);
    }

    bool Intersect(Ray& ray, float* t0) {
        glm::vec2 barycentricCoords;
        float t = 0;
        
        bool intersects = glm::intersectRayTriangle(ray.origin, ray.direction, vertices[0], vertices[1], vertices[2], barycentricCoords, t);

        bool within_triangle = intersects && (barycentricCoords.x >= 0.0f) && (barycentricCoords.y >= 0.0f) &&
            ((barycentricCoords.x + barycentricCoords.y) <= 1.0f);
        
        if(within_triangle) {
            *t0 = t;
        }
        
        return within_triangle;
    }

    bool Intersect(Triangle& o){
        // glm::vec3 e1 = vertices[1] - vertices[0];
        // glm::vec3 e2 = vertices[2] - vertices[0];
        // glm::vec3 h = glm::cross(other.vertices[1] - other.vertices[0], other.vertices[2] - other.vertices[0]);
        // float a = glm::dot(e1, h);

        // // If a is too small, the triangles are coplanar
        // if (a > -FLT_EPSILON && a < FLT_EPSILON) {
        //     return false;
        // }

        // float f = 1.0f / a;
        // glm::vec3 s = other.vertices[0] - vertices[0];
        // float u = f * glm::dot(s, h);

        // if (u < 0.0f || u > 1.0f) {
        //     return false;
        // }

        // glm::vec3 q = glm::cross(s, e1);
        // float v = f * glm::dot(other.vertices[1] - vertices[0], q);

        // if (v < 0.0f || u + v > 1.0f) {
        //     return false;
        // }

        // // At this point, u and v are within [0, 1], and u + v <= 1, so the triangles intersect
        // return true;
        return tri_tri_overlap_test_3d(glm::value_ptr(vertices[0]), glm::value_ptr(vertices[1]), glm::value_ptr(vertices[2]),
                                glm::value_ptr(o.vertices[0]), glm::value_ptr(o.vertices[1]), glm::value_ptr(o.vertices[2]));
    }
};

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
    Axis split_axis = Axis::XAXIS;
    float split_point = 0.0;
    AABB(): min(-INF), max(INF) {}
    AABB(const glm::vec3& min, const glm::vec3& max) : min(min), max(max) {}
    // taken from https://github.com/mmp/pbrt-v3/blob/master/src/core/geometry.h
    bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) {
        float t0 = 0, t1 = ray.tmax;
        for (int i = 0; i < 3; ++i) {
            // Update interval for _i_th bounding box slab
            float invRayDir = 1 / ray.direction[i];
            float tNear = (min[i] - ray.origin[i]) * invRayDir;
            float tFar = (max[i] - ray.origin[i]) * invRayDir;

            // Update parametric interval from slab intersection $t$ values
            if (tNear > tFar) std::swap(tNear, tFar);

            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar < t1 ? tFar : t1;
            if (t0 > t1) return false;
        }
        if (hit_t0) *hit_t0 = t0;
        if (hit_t1) *hit_t1 = t1;
        return true;
    }

    bool Intersect(const AABB& other) const {
        // Check for overlap along each axis
        bool overlap_x = (max.x >= other.min.x) && (min.x <= other.max.x);
        bool overlap_y = (max.y >= other.min.y) && (min.y <= other.max.y);
        bool overlap_z = (max.z >= other.min.z) && (min.z <= other.max.z);

        // If there is overlap along all axes, the AABBs overlap
        return overlap_x && overlap_y && overlap_z;
    }
};

struct Intersection {
    int triangle_id = -1;
    float t0 = 0;
    float t1 = 0;
};

struct NodeIntersection {
    int leaf_id1     = -1;
    int leaf_id2     = -1;
    int triangle_id1 = -1;
    int triangle_id2 = -1;
};

#endif
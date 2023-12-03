#ifndef STRUCTURES_H
#define STRUCTURES_H
#include <glm/glm.hpp>
#include <initializer_list>
#include <array>
#include <algorithm>

#define INF std::numeric_limits<float>::infinity()

enum Axis {
    XAXIS = 0,
    YAXIS = 1,
    ZAXIS = 2
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

    float Bounds(Axis axis) {
        glm::vec3 min =  *std::min_element(vertices.begin(), vertices.end(), [axis](glm::vec3 a, glm::vec3 b) {
            return a[axis] < b[axis];
        });
        glm::vec3 max =  *std::max_element(vertices.begin(), vertices.end(), [axis](glm::vec3 a, glm::vec3 b) {
            return a[axis] < b[axis];
        });

        return glm::abs(max[axis] - min[axis]);
    }

};

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
    AABB(): min(-INF), max(INF) {}
    AABB(const glm::vec3& min, const glm::vec3& max) : min(min), max(max) {}
};

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

struct Intersection {
    int triangle_id;
};

#endif
#ifndef STRUCTURES_H
#define STRUCTURES_H
#include <glm/glm.hpp>
#include <initializer_list>
#include <array>

struct Triangle {
    std::array<glm::vec3, 3> vertices = {};
    glm::vec3 centroid = {};
    Triangle(std::array<glm::vec3, 3> const& verts) : vertices(verts) {}
};

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

struct Intersection {
    int triangle_id;
};

#endif
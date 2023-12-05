#ifndef CAMERA_H_
#define CAMERA_H_

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "transform.h"
#include "shader.h"

class Camera {

    public:
        enum class Projection {
            PERSPECTIVE,
            ORTHOGRAPHIC
        };

        Camera();
        ~Camera();

        void Update();

        // Set the view from camera parameters: initial position of camera,
        // point looking at, and up vector
        // Resets the current orientation and position of the camera
        void SetView(glm::vec3 position, glm::vec3 look_at, glm::vec3 up);
        // Set projection from frustum parameters: field-of-view,
        // near and far planes, and width and height of viewport
        void SetPerspective(GLfloat fov, GLfloat near, GLfloat far, GLfloat w, GLfloat h);
        void SetOrtho(GLfloat w, GLfloat h);
        void SetProjectionUniforms(Shader& shd, Projection = Projection::PERSPECTIVE);

        void Attach(Transform* parent_transform);
        void Detach();
        void Drop();
        void MoveTo(const glm::vec3 newpos);
        void Reset();
        bool IsAttached();
        void SetupViewMatrix(void);
        const glm::mat4& GetViewMatrix() {return view_matrix_;}

        void OrbitPitch(float pitch);
        void OrbitYaw(float yaw);

        float pitch_speed = 0.0f;
        float yaw_speed = 0.0f;
        float roll_speed = 0.0f;
        glm::vec3 angular_velocity {0.0f, 0.0f, 0.0f};
        glm::vec3 velocity {0.0f, 0.0f, 0.0f};
        bool locked = false;
        float distance = 1.0f;

        float orbit_yaw = 0.0;
        float orbit_pitch = 0.0;

        Transform transform; 
    private:
        Transform* parent_transform {nullptr};

        glm::mat4 view_matrix_; // View matrix

        glm::mat4 perspective_matrix; // Projection matrix
        glm::mat4 ortho_matrix; // Projection matrix

        glm::vec3 original_pos = {0.0, 0.0, 0.0};

        // Create view matrix from current camera parameters

}; // class Camera

#endif // CAMERA_H_

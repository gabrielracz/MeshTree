#ifndef VIEW_H
#define VIEW_H

#include <string>
#include <memory>
#include <unordered_map>
#include <functional>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "camera.h"
#include "light.h"
#include "defines.h"
#include "mesh.h"
#include "texture.h"
#include "shader.h"

class Application;

namespace config
{
    const float camera_near_clip_distance = 0.01;
    const float camera_far_clip_distance = 10000.0;
    const float camera_fov = 75.0; // Field-of-view of camera (degrees)
    const glm::vec3 viewport_background_color(0.0, 0.0, 0.0);
    const glm::vec3 camera_position(0.0, 0.0, 5.0);
    const glm::vec3 camera_look_at(0.0, 0.0, 0.0);
    const glm::vec3 camera_up(0.0, 1.0, 0.0);
};

class View
{

    enum RenderMode
    {
        FILL = 0,
        WIREFRAME,
        NUM_RENDERMODES
    };

public:

    typedef struct
    {
        GLFWwindow *ptr;
        std::string title;
        int width;
        int height;
    } Window;

    View() = default;
    ~View();
    void Init(const std::string &title, int width, int height);

    void Clear();
    void RenderObj(Transform& transform, Mesh& mesh, Shader& shader, Light& light, int highlighted_vertex = -1);
    void RenderBox(Shader& shader, const glm::vec3& min_extent, const glm::vec3& max_extent, const glm::vec4 color);
    void RenderLine(Mesh& line_mesh, Shader& shader, const glm::vec3& origin, const glm::vec3& direction, const glm::vec4& color);
    void Update();

    void ToggleMouseCapture();
    void SetMouseHandler(MouseHandler h) { mouse_handler = h; }
    void ToggleRenderMode();

    KeyMap &GetKeys() { return key_controls; }
    Mouse &GetMouse() { return mouse; }
    MouseMap& GetMouseButtons() {return mouse_buttons;}
    Camera &GetCamera() { return camera; }

    int GetWidth() { return win.width; }
    int GetHeight() { return win.height; }

    bool Closed();

    Window GetWindow() { return win; }

private:
    Window win;
    Camera camera;
    Mouse mouse;
    KeyMap key_controls;
    MouseMap mouse_buttons;


    int render_mode = RenderMode::FILL;

    void InitWindow(const std::string &title, int width, int height);
    void InitView();
    void InitEventHandlers();
    void InitControls();
    void InitCamera();

    static void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void ResizeCallback(GLFWwindow *window, int width, int height);
    static void MouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
    static void MouseButtonCallback(GLFWwindow *window, int key, int action, int mods);

    std::function<void(Mouse &mouse)> mouse_handler;
};

#endif
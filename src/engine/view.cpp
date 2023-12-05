#include "view.h"

#include <GLFW/glfw3.h>
#include <stdexcept>


View::~View() {
    glfwTerminate();
}

void View::Clear() {
    // DRAW
    const glm::vec4 background_color = {0.0f, 0.0f, 0.0f, 1.0f};
    glClearColor(background_color[0], 
                 background_color[1],
                 background_color[2], 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    switch(render_mode) {
        case RenderMode::FILL:
            glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
            break;
        case RenderMode::WIREFRAME:
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
            break;
    }
}

void View::Update() {
    glfwSwapBuffers(win.ptr);
    glfwPollEvents();
}

void View::Init(const std::string& title, int width, int height) {
    InitWindow(title, width, height);
    InitView();
    InitCamera();
    InitEventHandlers();
    InitControls();
}

void View::InitWindow(const std::string& title, int width, int height) {
    // Initialize the window management library (GLFW)
    if (!glfwInit()) {
        throw((std::runtime_error(std::string("Could not initialize the GLFW library"))));
    }

    // Create a window and its OpenGL context
    win.width = width;
    win.height = height;
    win.title = title;

    // win.ptr = glfwCreateWindow(win.width, win.height, win.title.c_str(), glfwGetPrimaryMonitor(), NULL);
    win.ptr = glfwCreateWindow(win.width, win.height, win.title.c_str(), NULL, NULL);

    if (!win.ptr) {
        glfwTerminate();
        throw(std::runtime_error(std::string("Could not create window")));
    }

    // Make the window's context the current 
    glfwMakeContextCurrent(win.ptr);

    // Initialize the GLEW library to access OpenGL extensions
    // Need to do it after initializing an OpenGL context
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK){
        throw(std::runtime_error(std::string("Could not initialize the GLEW library: ")+std::string((const char *) glewGetErrorString(err))));
    }
}

void View::InitView(){

    // Set up z-buffer
    glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_CULL_FACE);

	//Use this to disable vsync
	// glfwSwapInterval(0);
    glViewport(0, 0, win.width, win.height);
}

void View::InitCamera() {
    camera.SetView(config::camera_position, config::camera_look_at, config::camera_up);
    camera.SetPerspective(config::camera_fov, config::camera_near_clip_distance, config::camera_far_clip_distance, win.width, win.height);
    camera.SetOrtho(win.width, win.height);
}

void View::InitEventHandlers(void){

    // Set event callbacks
    glfwSetKeyCallback(win.ptr, KeyCallback);
    glfwSetFramebufferSizeCallback(win.ptr, ResizeCallback);
    glfwSetCursorPosCallback(win.ptr, MouseMoveCallback);
	glfwSetMouseButtonCallback(win.ptr, MouseButtonCallback);


    // Set pointer to game object, so that callbacks can access it
    glfwSetWindowUserPointer(win.ptr, (void *) this);
}

void View::InitControls() {
	mouse.prev = glm::vec2((float) win.width / 2, (float) win.height / 2);
    mouse.captured = true;
    mouse.first_captured = true;

	// glfwSetInputMode(win.ptr, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    for(int key = GLFW_KEY_SPACE; key < GLFW_KEY_LAST; key++) {
        key_controls.insert({key, false});
    }

    for(int button = GLFW_MOUSE_BUTTON_1; button < GLFW_MOUSE_BUTTON_LAST; button++) {
        key_controls.insert({button, false});
    }
}

void View::RenderObj(Transform &transform, Mesh &mesh, Shader &shader, Light& light, int highlighted_vertex) { 
    shader.Use();
    // camera
    camera.SetProjectionUniforms(shader);
    glm::mat4 view_matrix = camera.GetViewMatrix();

    // model 
    glm::mat4 world = transform.GetLocalMatrix();
    glm::mat4 normal_matrix = glm::transpose(glm::inverse(view_matrix * world));
    shader.SetUniform4m(world, "world_mat");
    shader.SetUniform4m(normal_matrix, "normal_mat");
    shader.SetUniform1i(highlighted_vertex, "highlighted");

    // light
    light.SetUniforms(shader);

    mesh.Draw();
}

void View::RenderBox(Shader& shader, const glm::vec3& min_extent, const glm::vec3& max_extent, const glm::vec4 color, int draw_edges) {
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const std::vector<float> cube_vertices = {
    // Vertices
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,  // Vertex 0
     0.5f, -0.5f, -0.5f,  1.0f, 0.0f,  // Vertex 1
     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,  // Vertex 2
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,  // Vertex 3
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,  // Vertex 4
     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,  // Vertex 5
     0.5f,  0.5f,  0.5f,  1.0f, 1.0f,  // Vertex 6
    -0.5f,  0.5f,  0.5f,  0.0f, 1.0f   // Vertex 7
    };

    const std::vector<unsigned int> cube_indices = {
        // Indices
        0, 1, 2,  // Front face
        2, 3, 0,
        1, 5, 6,  // Right face
        6, 2, 1,
        7, 6, 5,  // Top face
        5, 4, 7,
        4, 0, 3,  // Left face
        3, 7, 4,
        4, 5, 1,  // Bottom face
        1, 0, 4,
        3, 2, 6,  // Back face
        6, 7, 3
    };

    const std::vector<float> vertices = {
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
    };

    Layout layout({{FLOAT3, "position"}, {FLOAT2, "uv"}});
    const static Mesh cube(vertices, {}, layout);

    glm::vec3 box_position = (min_extent + max_extent)/2.0f;
    glm::vec3 box_scale =   abs(max_extent - min_extent);
    Transform box_transform;
    box_transform.SetPosition(box_position);
    box_transform.SetScale(box_scale);

    shader.Use();
    // camera
    camera.SetProjectionUniforms(shader);
    glm::mat4 view_matrix = camera.GetViewMatrix();

    // model 
    glm::mat4 world = box_transform.GetLocalMatrix();
    shader.SetUniform4m(world, "world_mat");

    shader.SetUniform4f(color, "color");
    shader.SetUniform1i(draw_edges, "draw_edges");

    cube.Draw();
    glDepthMask(GL_TRUE);
}

void View::RenderLine(Mesh& line_mesh, Shader& shader, const glm::vec3& origin, const glm::vec3& direction, const glm::vec4& color) {
    glBindVertexArray(line_mesh.VAO);
    glBindBuffer(GL_ARRAY_BUFFER, line_mesh.VBO);

    glm::vec3 line = origin + direction;

    float line_verts[] = {
            origin.x, origin.y, origin.z,
            line.x, line.y, line.z
    };

    glBufferSubData(GL_ARRAY_BUFFER, 0, 6*sizeof(float), line_verts);

    shader.Use();
    camera.SetProjectionUniforms(shader);
    shader.SetUniform4f(color, "color");

    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}



void View::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods){

    // Get user data with a pointer to the game class
    void* ptr = glfwGetWindowUserPointer(window);
    View *view = (View *) ptr;

	if(action == GLFW_PRESS) {
		view->key_controls[key] = true;
	}else if(action == GLFW_RELEASE) {
		view->key_controls[key] = false;
	}
}

void View::ResizeCallback(GLFWwindow* window, int width, int height){

    // Set up viewport and camera projection based on new window size
    glViewport(0, 0, width, height);
    void* ptr = glfwGetWindowUserPointer(window);
    View *view = (View *) ptr;
    view->win.width = width;
    view->win.height = height;
    view->camera.SetPerspective(config::camera_fov, config::camera_near_clip_distance, config::camera_far_clip_distance, width, height);
    view->camera.SetOrtho(width, height);
    view->mouse.first_captured = true;
}

void View::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    void* ptr = glfwGetWindowUserPointer(window);
    View *view = (View *) ptr;

    if(!view->mouse.captured) {
        return;
    }

	Mouse& mouse = view->mouse;
	if (mouse.first_captured) {
        mouse.prev = {xpos, ypos};
		mouse.first_captured = false;
	}

    mouse.move = glm::vec2(xpos, ypos) - mouse.prev;
    mouse.prev = {xpos, ypos};

    if(view->mouse_handler) {
        view->mouse_handler(mouse);
    }
}

void View::MouseButtonCallback(GLFWwindow* window, int key, int action, int mods) {

    void* ptr = glfwGetWindowUserPointer(window);
    View *view = (View *) ptr;

	if(action == GLFW_PRESS) {
		view->mouse_buttons[key] = true;
	}else if(action == GLFW_RELEASE) {
		view->mouse_buttons[key] = false;
	}
}


void View::ToggleMouseCapture() {
    mouse.captured = !mouse.captured;
    mouse.first_captured = true;
    glfwSetInputMode(win.ptr, GLFW_CURSOR, mouse.captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}


void View::ToggleRenderMode() {
    render_mode = (render_mode + 1) % RenderMode::NUM_RENDERMODES;
}

bool View::Closed() {
    return glfwWindowShouldClose(win.ptr);
}

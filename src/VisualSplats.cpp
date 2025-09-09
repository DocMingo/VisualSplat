/*
* 1. 高斯椭球的scale是对数存储的，需要在着色器中exp恢复
* 2. 在imgui中添加参数调节
*/
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Splats.h>
#include <utils.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include<pcl/common/common.h>
#include<pcl/io/pcd_io.h>
// #include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply_io.h>
#include<vector>
#include<spdlog/spdlog.h>
#include<learnopengl/camera_m.h>
// #include<point_cloud_render_camera.h>
#include<learnopengl/shader_m.h>
#include<format>
#include<filesystem>
// #include<dmyDependence/dmyTool.h>

#include "cudaGL.h"
#include <cuda_runtime.h>
#include<device_launch_parameters.h>
#include"../include/Header.cuh"

#define USE_IMGUI 1

#include<ImGuiFPSDisplay.hpp>

__global__ void helloworld_from_gpu(void) {
	printf("Hello world from GPU\n");
	return;
}

using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// settings
const unsigned int SCR_WIDTH = 1080;
const unsigned int SCR_HEIGHT = 680;
float fov = 50.0f;


Camera camera(glm::vec3(0.0f, 0.0f, 0.0f));
bool firstMouse = true;
bool leftMousePressed = false;

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// lighting
auto litime = glfwGetTime();
glm::vec3 lightPos(1.2f, 1.0f, 2.0f);

GLfloat quad_v[] = {
	-1.0f, 1.0f,
	1.0f, 1.0f,
	1.0f, -1.0f,
	-1.0f, -1.0f
};

GLuint quad_f[] = {
	0, 1, 2,
	0, 2, 3
};

template<class T>
GLuint setupSSBO(const GLuint& bindIdx, const std::vector<T>& bufferData) {
	GLuint ssbo;
	// Generate SSBO
	glGenBuffers(1, &ssbo);
	// Bind ssbo to GL_SHADER_STORAGE_BUFFER
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
	// Populate GL_SHADER_STORAGE_BUFFER with our data
	glBufferData(GL_SHADER_STORAGE_BUFFER, bufferData.size() * sizeof(std::decay_t<T>), bufferData.data(), GL_STATIC_DRAW);
	// Specify the index of the binding (bindIdx)
	// glBindBufferBase 不仅绑定缓冲区，还显式关联到 Shader 中的 binding = bindIdx。
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindIdx, ssbo); // 将ssbo绑定到 GL_SHADER_STORAGE_BUFFER 的第bindIdx号的 binding point上，后面在着色器程序中使用binding = 1即可使用
	// Unbind ssbo to GL_SHADER_STORAGE_BUFFER
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // 接触全局绑定是使用 如果目的是“解绑定”，应该绑定 0 而非 ssbo

	return ssbo;
};

template<typename T>
void updateSSBO(GLuint ssbo, const std::vector<T>& data) {
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, data.size() * sizeof(T), data.data());
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

// 修改后的主函数 - 集成CUDA排序shi
int main() {

    /*=== load config file ===*/
    std::map config_map = parseFileData(R"(D:\Work\VSProject\VisualSplat\src\resources\config.txt)");

    /* === add cuda support ===*/
    int deviceCount{};
    cudaError_t error = cudaGetDeviceCount(&deviceCount);

    if (error != cudaSuccess) {
        cerr << "CUDA error: " << cudaGetErrorString(error) << endl;
        exit(-1);
    }

    /*=== Typical implementation by glfw + glad ===*/
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CUDA-OpenGL Gaussian Renderer", NULL, NULL);
    if (!window) { std::cerr << "Failed to create GLFW window\n"; return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback); // 获取光标位置参数
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);// 默认设置，可以删除
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    /*初始化glad*/
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n"; return -1;
    }

    spdlog::info("=== imgui ===");
    // 使用ImGui方案
    ImGuiFPSDisplay fpsDisplay;
    if (!fpsDisplay.initialize(window)) {
        return -1;
    }


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    spdlog::info("初始化成功, 创建着色器 start");
    Shader this_shader("src/resources/shader/vertex_shader.glsl", "src/resources/shader/fragment_shader.glsl");

    spdlog::info("着色器创建成功，开始读取高斯数据:{}", config_map["GSpath"]);
    GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
    auto ret_value = pcl::io::loadPLYFile<GaussianData>(config_map["GSpath"], *Gaussian_cloud);
    int numInstances = Gaussian_cloud->points.size();
    if (!ret_value) {
        cout << "高斯初始化成功, 点数为" << numInstances << endl;
    }

	// pcl::PointXYZ minp, maxp;
	// pcl::getMinMax3D(*Gaussian_cloud, minp, maxp);

    // 初始化相机
    // camera.Position = glm::vec3(Gaussian_cloud->points[0].x, Gaussian_cloud->points[0].y, Gaussian_cloud->points[0].z);
	Eigen::Vector3f centroid;
    Eigen::Matrix3f matr_gs = Gaussian_cloud->getMatrixXfMap().block(0,0,3, numInstances);
    centroid = matr_gs.rowwise().mean(); // 1 * 3

    size_t middleIndex = numInstances >> 1;
	camera.Position = glm::vec3(0,0,0);
	camera.center = glm::vec3(0,0,0);
	spdlog::info("相机初始化成功,位置为:{},{},{}", camera.Position.x, camera.Position.y, camera.Position.z);
	// PointCloudCamera camera;
    // camera.fitToPointCloud();

    // 准备高斯数据
    std::vector<float> flat_gaussian_data;
    flat_gaussian_data.reserve(numInstances * 14);
    for (const auto& point : Gaussian_cloud->points) {
        // cout << Eigen::Vector4f(point.rot_0, point.rot_1, point.rot_2, point.rot_3).transpose() << endl;
		// cout << "point.scale" << point.scale_0 << "," << point.scale_1 << "," << point.scale_2 << endl;
        glm::vec4 normRot = glm::normalize(glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3));
        // glm::vec4 normRot = normalizeRotation(glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3));
        glm::vec3 RGB = SH2RGB(glm::vec3(point.f_dc_0, point.f_dc_1, point.f_dc_2));
        flat_gaussian_data.insert(flat_gaussian_data.end(), {
            point.x - centroid[0], point.y- centroid[1], point.z- centroid[2],
            normRot.x, normRot.y, normRot.z, normRot.w,
            glm::exp(1.0f*point.scale_0), glm::exp(1.0f * point.scale_1), glm::exp(1.0f * point.scale_2),
            sigmoid(point.opacity),
            RGB.x, RGB.y, RGB.z
            });
    }

    // OpenGL资源设置
    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_v), quad_v, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quad_f), quad_f, GL_STATIC_DRAW);
    GLint quad_position = glGetAttribLocation(this_shader.ID, "quadPosition");
    glVertexAttribPointer(quad_position, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(quad_position);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // 设置SSBO
    GLuint pointsBindIdx = 2;
    GLuint sortedBindIdx = 1;
    GLuint ssbo1 = setupSSBO(pointsBindIdx, flat_gaussian_data);

    // 为索引创建SSBO（初始为顺序索引）
    std::vector<int> initialIndices(numInstances);
    std::iota(initialIndices.begin(), initialIndices.end(), 0);
    GLuint ssbo2 = setupSSBO<int>(sortedBindIdx, initialIndices);

    // 初始化CUDA排序器
    CudaGaussianSorter cudaSorter;
    if (!cudaSorter.initialize(ssbo1, ssbo2, numInstances)) {
        std::cerr << "Failed to initialize CUDA sorter" << std::endl;
        return -1;
    }

    spdlog::info("CUDA-OpenGL协作排序器初始化成功");

    float htany = tan(glm::radians(fov) / 2);
    float htanx = htany * SCR_WIDTH / SCR_HEIGHT;
    float focal_z = SCR_HEIGHT / (2 * htany);
    glm::vec3 hfov_focal(htanx, htany, focal_z);

    // 性能计时器
    auto frameStart = std::chrono::high_resolution_clock::now();
    auto sortStart = frameStart;
    auto renderStart = frameStart;
    const double frameDuration = 1.0 / 120.0;
    while (!glfwWindowShouldClose(window)) {
        auto frameStart = std::chrono::high_resolution_clock::now();

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // 更新FPS计数器
        fpsDisplay.update();

        processInput(window);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 viewMat = camera.GetViewMatrix();

        // 使用CUDA进行排序，直接更新OpenGL SSBO
        sortStart = std::chrono::high_resolution_clock::now();
        cudaSorter.sortGaussians(ssbo1, glm::mat3(viewMat));
        auto sortEnd = std::chrono::high_resolution_clock::now();
        double sortTime = std::chrono::duration<double, std::milli>(sortEnd - sortStart).count();

        renderStart = std::chrono::high_resolution_clock::now();
        // 设置着色器参数
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom),
            (float)SCR_WIDTH / (float)SCR_HEIGHT,
            0.1f, 100000.f);

        this_shader.use();
        this_shader.setMat4("projection", projection);
        this_shader.setVec3("hfov_focal", hfov_focal);
        this_shader.setMat4("view", viewMat);
        this_shader.setVec3("camera_position", camera.Position);
        this_shader.setFloat("scaleMod", fpsDisplay.getEllipsoidParameters().uniformScale);

        // 绘制
        glBindVertexArray(VAO);
        glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)0, numInstances);
        glBindVertexArray(0);

        auto renderEnd = std::chrono::high_resolution_clock::now();
        double renderTime = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

        // 设置性能数据并渲染ImGui
        auto frameEnd = std::chrono::high_resolution_clock::now();
        double totalTime = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
        fpsDisplay.setPerformanceTimes(sortTime, renderTime, totalTime);
        fpsDisplay.render();


        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 清理资源
    cudaSorter.cleanup();

    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteBuffers(1, &ssbo1);
    glDeleteBuffers(1, &ssbo2);
    glDeleteVertexArrays(1, &VAO);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    /*
    glfwGetKey(window, key) 只检测键盘按键，比如 W, A, S, D。
    鼠标按钮（左、中、右键）需要用 glfwGetMouseButton(window, button)：
    */
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	// if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	// 	camera.ProcessKeyboard(FORWARD, deltaTime);
	// if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	// 	camera.ProcessKeyboard(BACKWARD, deltaTime);
	// if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	// 	camera.ProcessKeyboard(LEFT, deltaTime);
	// if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	// 	camera.ProcessKeyboard(RIGHT, deltaTime);
    // if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {};
        // 设置鼠标控制事件切换

}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
	float xpos = static_cast<float>(xposIn);
	float ypos = static_cast<float>(yposIn);
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}
    
	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    
	lastX = xpos;
	lastY = ypos;
    
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
        camera.orbitRotate(xoffset, yoffset);
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)) {
        camera.ProcessDirectMove(xoffset, yoffset);
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)) {
        camera.ProcessMouseScroll(yoffset);
    }
}

// 鼠标按键回调函数
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            leftMousePressed = true;
            // 获取当前鼠标位置，避免跳跃
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            lastX = static_cast<float>(xpos);
            lastY = static_cast<float>(ypos);
        }
        else if (action == GLFW_RELEASE)
        {
            leftMousePressed = false;
        }
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.zoom(static_cast<float>(yoffset));
}
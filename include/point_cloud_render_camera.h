#ifndef POINT_CLOUD_CAMERA_H
#define POINT_CLOUD_CAMERA_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <algorithm>

// 点云相机操作模式
enum PointCloudCameraMode {
    ORBIT,      // 轨道模式（围绕目标旋转）
    PAN,        // 平移模式
    ZOOM,       // 缩放模式
    FREE_LOOK   // 自由视角模式
};

// 鼠标操作类型
enum MouseOperation {
    MOUSE_ORBIT,
    MOUSE_PAN,
    MOUSE_ZOOM,
    MOUSE_NONE
};

// 点云渲染专用相机类
class PointCloudCamera {
public:
    // 相机状态
    glm::vec3 position;     // 相机位置
    glm::vec3 target;       // 观察目标点
    glm::vec3 up;           // 上方向向量

    // 轨道控制参数
    float distance;         // 到目标的距离
    float azimuth;         // 方位角（水平旋转）
    float elevation;       // 仰角（垂直旋转）

    // 控制参数
    float orbitSpeed;      // 轨道旋转速度
    float panSpeed;        // 平移速度
    float zoomSpeed;       // 缩放速度
    float wheelZoomSpeed;  // 滚轮缩放速度

    // 约束参数
    float minDistance;     // 最小距离
    float maxDistance;     // 最大距离
    float minElevation;    // 最小仰角
    float maxElevation;    // 最大仰角

    // 交互状态
    MouseOperation currentOperation;
    glm::vec2 lastMousePos;
    bool isDragging;

    // 构造函数
    PointCloudCamera(
        glm::vec3 initialTarget = glm::vec3(0.0f, 0.0f, 0.0f),
        float initialDistance = 10.0f,
        float initialAzimuth = 0.0f,
        float initialElevation = 0.0f
    ) :
        target(initialTarget),
        distance(initialDistance),
        azimuth(initialAzimuth),
        elevation(initialElevation),
        up(0.0f, 1.0f, 0.0f),
        orbitSpeed(1.0f),
        panSpeed(0.01f),
        zoomSpeed(0.1f),
        wheelZoomSpeed(0.5f),
        minDistance(0.1f),
        maxDistance(1000.0f),
        minElevation(-89.0f),
        maxElevation(89.0f),
        currentOperation(MOUSE_NONE),
        isDragging(false),
        lastMousePos(0.0f, 0.0f)
    {
        updateCameraPosition();
    }

    // 根据点云数据自动设置相机位置
    void fitToPointCloud(const glm::vec3& minBounds, const glm::vec3& maxBounds) {
        // 计算点云中心
        target = (minBounds + maxBounds) * 0.5f;

        // 计算点云大小
        glm::vec3 size = maxBounds - minBounds;
        float maxSize = std::max({ size.x, size.y, size.z });

        // 设置合适的距离
        distance = maxSize * 2.0f;

        // 设置距离约束
        minDistance = maxSize * 0.01f;
        maxDistance = maxSize * 10.0f;

        // 重置角度到一个好的初始视角
        azimuth = 45.0f;
        elevation = 30.0f;

        updateCameraPosition();

        std::cout << "Camera fitted to point cloud:" << std::endl;
        std::cout << "  Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
        std::cout << "  Distance: " << distance << std::endl;
        std::cout << "  Bounds: [" << maxSize << "]" << std::endl;
    }

    // 获取视图矩阵
    glm::mat4 getViewMatrix() const {
        return glm::lookAt(position, target, up);
    }

    // 获取投影矩阵
    glm::mat4 getProjectionMatrix(float aspect, float fov = 45.0f, float nearPlane = 0.01f, float farPlane = 1000.0f) const {
        // 动态调整近远平面
        float dynamicNear = std::max(nearPlane, distance * 0.001f);
        float dynamicFar = std::max(farPlane, distance * 100.0f);
        return glm::perspective(glm::radians(fov), aspect, dynamicNear, dynamicFar);
    }

    // 处理鼠标按下
    void onMousePress(double x, double y, int button) {
        lastMousePos = glm::vec2(x, y);
        isDragging = true;

        switch (button) {
        case 0: // 左键 - 轨道旋转
            currentOperation = MOUSE_ORBIT;
            break;
        case 1: // 右键 - 平移
            currentOperation = MOUSE_PAN;
            break;
        case 2: // 中键 - 缩放
            currentOperation = MOUSE_ZOOM;
            break;
        default:
            currentOperation = MOUSE_NONE;
            break;
        }
    }

    // 处理鼠标释放
    void onMouseRelease() {
        isDragging = false;
        currentOperation = MOUSE_NONE;
    }

    // 处理鼠标移动
    void onMouseMove(double x, double y) {
        if (!isDragging) return;

        glm::vec2 currentPos(x, y);
        glm::vec2 delta = currentPos - lastMousePos;

        switch (currentOperation) {
        case MOUSE_ORBIT:
            orbit(delta.x * orbitSpeed, delta.y * orbitSpeed);
            break;
        case MOUSE_PAN:
            pan(-delta.x * panSpeed * distance * 0.001f,
                delta.y * panSpeed * distance * 0.001f);
            break;
        case MOUSE_ZOOM:
            zoom(delta.y * zoomSpeed);
            break;
        }

        lastMousePos = currentPos;
    }

    // 处理滚轮缩放
    void onMouseScroll(double yoffset) {
        zoom(-yoffset * wheelZoomSpeed);
    }

    // 处理键盘输入（用于精确控制）
    void processKeyboard(int key, float deltaTime) {
        float keyPanSpeed = panSpeed * distance * 10.0f * deltaTime;
        float keyRotateSpeed = orbitSpeed * 50.0f * deltaTime;

        switch (key) {
        case GLFW_KEY_W: // 前进
            zoom(-zoomSpeed * 5.0f * deltaTime);
            break;
        case GLFW_KEY_S: // 后退
            zoom(zoomSpeed * 5.0f * deltaTime);
            break;
        case GLFW_KEY_A: // 左移
            pan(-keyPanSpeed, 0.0f);
            break;
        case GLFW_KEY_D: // 右移
            pan(keyPanSpeed, 0.0f);
            break;
        case GLFW_KEY_Q: // 上移
            pan(0.0f, keyPanSpeed);
            break;
        case GLFW_KEY_E: // 下移
            pan(0.0f, -keyPanSpeed);
            break;
        case GLFW_KEY_LEFT: // 左旋转
            orbit(-keyRotateSpeed, 0.0f);
            break;
        case GLFW_KEY_RIGHT: // 右旋转
            orbit(keyRotateSpeed, 0.0f);
            break;
        case GLFW_KEY_UP: // 上旋转
            orbit(0.0f, -keyRotateSpeed);
            break;
        case GLFW_KEY_DOWN: // 下旋转
            orbit(0.0f, keyRotateSpeed);
            break;
        }
    }

    // 重置到初始位置
    void reset() {
        azimuth = 0.0f;
        elevation = 0.0f;
        distance = 10.0f;
        target = glm::vec3(0.0f);
        updateCameraPosition();
    }

    // 设置预定义视角
    void setViewPreset(const std::string& preset) {
        if (preset == "front") {
            azimuth = 0.0f;
            elevation = 0.0f;
        }
        else if (preset == "back") {
            azimuth = 180.0f;
            elevation = 0.0f;
        }
        else if (preset == "left") {
            azimuth = -90.0f;
            elevation = 0.0f;
        }
        else if (preset == "right") {
            azimuth = 90.0f;
            elevation = 0.0f;
        }
        else if (preset == "top") {
            azimuth = 0.0f;
            elevation = 90.0f;
        }
        else if (preset == "bottom") {
            azimuth = 0.0f;
            elevation = -90.0f;
        }
        else if (preset == "isometric") {
            azimuth = 45.0f;
            elevation = 35.26f; // arcsin(tan(30°))
        }

        updateCameraPosition();
    }

    // 获取相机信息（用于调试）
    void printCameraInfo() const {
        std::cout << "Camera Info:" << std::endl;
        std::cout << "  Position: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
        std::cout << "  Target: (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
        std::cout << "  Distance: " << distance << std::endl;
        std::cout << "  Azimuth: " << azimuth << "°" << std::endl;
        std::cout << "  Elevation: " << elevation << "°" << std::endl;
    }

private:
    // 轨道旋转
    void orbit(float deltaAzimuth, float deltaElevation) {
        azimuth += deltaAzimuth;
        elevation += deltaElevation;

        // 约束仰角
        elevation = std::clamp(elevation, minElevation, maxElevation);

        // 规范化方位角
        while (azimuth > 360.0f) azimuth -= 360.0f;
        while (azimuth < 0.0f) azimuth += 360.0f;

        updateCameraPosition();
    }

    // 平移
    void pan(float deltaX, float deltaY) {
        // 计算相机的右向量和上向量
        glm::vec3 forward = glm::normalize(target - position);
        glm::vec3 right = glm::normalize(glm::cross(forward, up));
        glm::vec3 cameraUp = glm::normalize(glm::cross(right, forward));

        // 移动目标点和相机位置
        glm::vec3 offset = right * deltaX + cameraUp * deltaY;
        target += offset;
        position += offset;
    }

    // 缩放
    void zoom(float deltaDistance) {
        distance += deltaDistance;
        distance = std::clamp(distance, minDistance, maxDistance);
        updateCameraPosition();
    }

    // 更新相机位置
    void updateCameraPosition() {
        // 将球坐标转换为笛卡尔坐标
        float azimuthRad = glm::radians(azimuth);
        float elevationRad = glm::radians(elevation);

        position.x = target.x + distance * cos(elevationRad) * cos(azimuthRad);
        position.y = target.y + distance * sin(elevationRad);
        position.z = target.z + distance * cos(elevationRad) * sin(azimuthRad);
    }
};

// GLFW回调函数设置
class CameraController {
private:
    PointCloudCamera* camera;
    GLFWwindow* window;

public:
    CameraController(PointCloudCamera* cam, GLFWwindow* win) : camera(cam), window(win) {
        // 设置用户指针，用于回调函数访问
        glfwSetWindowUserPointer(window, this);

        // 设置回调函数
        glfwSetCursorPosCallback(window, mouseCallback);
        glfwSetMouseButtonCallback(window, mouseButtonCallback);
        glfwSetScrollCallback(window, scrollCallback);

        // 显示鼠标光标
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    void processInput(float deltaTime) {
        // 检查键盘输入
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_W, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_S, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_A, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_D, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_Q, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_E, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_RIGHT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
            camera->processKeyboard(GLFW_KEY_DOWN, deltaTime);

        // 预设视角快捷键
        static bool key1Pressed = false, key2Pressed = false, key3Pressed = false;
        static bool key4Pressed = false, key5Pressed = false, key6Pressed = false;
        static bool key7Pressed = false, key0Pressed = false;

        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS && !key1Pressed) {
            camera->setViewPreset("front");
            key1Pressed = true;
        }
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_RELEASE) key1Pressed = false;

        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS && !key3Pressed) {
            camera->setViewPreset("right");
            key3Pressed = true;
        }
        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_RELEASE) key3Pressed = false;

        if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS && !key7Pressed) {
            camera->setViewPreset("top");
            key7Pressed = true;
        }
        if (glfwGetKey(window, GLFW_KEY_7) == GLFW_RELEASE) key7Pressed = false;

        if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS && !key0Pressed) {
            camera->setViewPreset("isometric");
            key0Pressed = true;
        }
        if (glfwGetKey(window, GLFW_KEY_0) == GLFW_RELEASE) key0Pressed = false;
    }

private:
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
        CameraController* controller = static_cast<CameraController*>(glfwGetWindowUserPointer(window));
        controller->camera->onMouseMove(xpos, ypos);
    }

    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        CameraController* controller = static_cast<CameraController*>(glfwGetWindowUserPointer(window));

        if (action == GLFW_PRESS) {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            controller->camera->onMousePress(xpos, ypos, button);
        }
        else if (action == GLFW_RELEASE) {
            controller->camera->onMouseRelease();
        }
    }

    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
        CameraController* controller = static_cast<CameraController*>(glfwGetWindowUserPointer(window));
        controller->camera->onMouseScroll(yoffset);
    }
};

#endif // POINT_CLOUD_CAMERA_H
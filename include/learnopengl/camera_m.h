#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include<iostream>

using namespace std;

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    ROTATION_0,
    ROTATION_1
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 3.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;


// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
    // camera Attributes
    glm::vec3 center;
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // Orbit parameters
    float Distance;             // 中心点到相机平面的距离
    float MinDistance = 0.1f;   // 最小距离
    float MaxDistance = 1000.0f; // 最大距离

    // euler Angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // constructor with vectors
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
    {
        Position = position;
        WorldUp = up;
        center = glm::vec3(0.0f, 0.0f, 0.0f); // 初始化中心点
        Yaw = yaw;
        Pitch = pitch;
		Distance = glm::length(Position - center);
        Front = glm::normalize(center - Position);
        // Distance = glm::abs(glm::dot(Position - center, Front));
        updateCameraVectors();
    }
    // constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
    {
        Position = glm::vec3(posX, posY, posZ);
        WorldUp = glm::vec3(upX, upY, upZ);
        center = glm::vec3(0.0f, 0.0f, 0.0f); // 初始化中心点
        Yaw = yaw;
        Pitch = pitch;
        Front = glm::normalize(center - Position);
        Distance = glm::length(Position - center);
        // Distance = glm::abs(glm::dot(Position - center, Front));
        updateCameraVectors();
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix()
    {
        return glm::lookAt(Position, Position + Front, Up);
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        using namespace std;
        float velocity = MovementSpeed * deltaTime;

        cout << "MovementSpeed:" << MovementSpeed << endl;
        cout << "deltaTime:" << deltaTime << endl;
        cout << "velocity:" << velocity << endl;
        cout << "Position" << Position.x << "," << Position.y << "," << Position.z << endl;

        if (direction == FORWARD)
            Position += Front * velocity;
        if (direction == BACKWARD)
            Position -= Front * velocity;
        if (direction == LEFT)
            Position -= Right * velocity;
        if (direction == RIGHT)
            Position += Right * velocity;
        if (direction == ROTATION_0) {
            // Yaw += velocity;
            glm::mat3 rotationMat = glm::rotate(glm::mat4(1.0f), glm::radians(50.0f), glm::vec3(0, 1, 0));
            Front = glm::normalize(rotationMat * Front);
            // updateCameraVectors();
        }

        if (direction == ROTATION_1) {

        }
    }

    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        if (1) {
            // 轨道模式使用专用的旋转方法
            orbitRotate(xoffset, yoffset, constrainPitch);
        }
        else {
            // 自由模式的鼠标移动
            xoffset *= MouseSensitivity;
            yoffset *= MouseSensitivity;

            Yaw += xoffset;
            Pitch += yoffset;

            // make sure that when pitch is out of bounds, screen doesn't get flipped
            if (constrainPitch)
            {
                if (Pitch > 89.0f)
                    Pitch = 89.0f;
                if (Pitch < -89.0f)
                    Pitch = -89.0f;
            }

            // update Front, Right and Up Vectors using the updated Euler angles
            updateCameraVectors();
        }
    }

    void ProcessDirectMove(float xoffset, float yoffset) {
        Position -= Right * xoffset * MouseSensitivity;
        Position -= Up * yoffset * MouseSensitivity;
        updateCameraVectors();
    }
    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset)
    {
        Distance -= yoffset * MovementSpeed * 0.1f;
        Distance = glm::clamp(Distance, MinDistance, MaxDistance);
        updatePositionScroll(yoffset);
    }

	void updatePositionScroll(float speed) {

		Position += Front * speed;
	}
    // ===== 轨道相机专用方法 =====
    void updatePositionFromAngles() {

        Position.x = center.x + Distance * cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Position.y = center.y + Distance * sin(glm::radians(Pitch));
        Position.z = -center.z + Distance * sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));

        // 更新Front向量（始终指向中心）
        Front = glm::normalize(center - Position);
        updateCameraVectors();
    }

    // 设置旋转中心点
    void setCenter(const glm::vec3& newCenter) {
        center = newCenter;
        // 重新计算距离，保持相机到中心的相对位置
        Distance = glm::length(Position - center);
        updatePositionFromAngles();
    }

    // 设置相机到中心点的距离
    void setDistance(float distance) {
        Distance = glm::clamp(distance, MinDistance, MaxDistance);
        updatePositionFromAngles();
    }

    // 围绕中心点旋转（鼠标拖拽） - 这是你需要的主要方法
    void orbitRotate(float xoffset, float yoffset, bool constrainPitch = true) {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw += xoffset;
        Pitch -= yoffset;  // 注意这里是减号，让鼠标上移时向上看

        // 限制俯仰角避免翻转
        if (constrainPitch) {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        // 根据新的角度更新相机位置
        updatePositionFromAngles();
    }

    // 缩放（改变到中心点的距离）
    void zoom(float yoffset) {
        Distance -= yoffset * MovementSpeed * 0.1f;
        Distance = glm::clamp(Distance, MinDistance, MaxDistance);
        updatePositionFromAngles();
    }

private:
    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors()
    {

        // 重新计算Right和Up向量
        Right = glm::normalize(glm::cross(Front, WorldUp));
        Up = glm::normalize(glm::cross(Right, Front));
    }
};
#endif
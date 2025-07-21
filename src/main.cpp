#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Splats.h>
#include <utils.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include<pcl/io/pcd_io.h>
// #include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply_io.h>
#include<vector>
#include<spdlog/spdlog.h>
#include<learnopengl/camera.h>
#include<learnopengl/shader_m.h>
// #include<dmyDependence/dmyTool.h>

using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
float fov = 90.0f;
float znear = 0.01f;
float zfar = 100.f;

// camera
Camera camera(glm::vec3(9040.0f, 392.0f, 54.0f)); // 或稍远一点

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

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

GLuint setupSSBO(const GLuint& bindIdx, const std::vector<float>& bufferData) {
	GLuint ssbo;
	// Generate SSBO
	glGenBuffers(1, &ssbo);
	// Bind ssbo to GL_SHADER_STORAGE_BUFFER
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
	// Populate GL_SHADER_STORAGE_BUFFER with our data
	glBufferData(GL_SHADER_STORAGE_BUFFER, bufferData.size() * sizeof(int), bufferData.data(), GL_STATIC_DRAW);
	// Specify the index of the binding (bindIdx)
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindIdx, ssbo);
	// Unbind ssbo to GL_SHADER_STORAGE_BUFFER
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);

	return ssbo;
};


int main1() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Gaussian Renderer", NULL, NULL);
	if (!window) { std::cerr << "Failed to create GLFW window\n"; return -1; }
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cerr << "Failed to initialize GLAD\n"; return -1;
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	spdlog::info("初始化成功, 开始加载着色器");
	Shader this_shader("src/resources/shader/vertex_shader.glsl", "src/resources/shader/fragment_shader.glsl");

	GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
	pcl::io::loadPLYFile<GaussianData>("Z:/非结构化数据/高斯模型/输电/m77_绝缘子.ply", *Gaussian_cloud);
	int numInstances = Gaussian_cloud->points.size();
	cout << "高斯初始化成功, 点数为" << numInstances << endl;
	std::cout << "Sample point: " << Gaussian_cloud->points[0].x << ", " << Gaussian_cloud->points[0].y << Gaussian_cloud->points[0].z << std::endl;

	std::vector<float> flat_gaussian_data;
	flat_gaussian_data.reserve(numInstances * 14);
	for (const auto& point : Gaussian_cloud->points) {
		// glm::vec4 tempRots = glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3);
		glm::vec4 normRot = normalizeRotation(glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3));
		glm::vec3 RGB = SH2RGB(glm::vec3(point.f_dc_0, point.f_dc_1, point.f_dc_2));
		flat_gaussian_data.insert(flat_gaussian_data.end(), {
			point.x, point.y, point.z,
			normRot.x, normRot.y, normRot.z, normRot.w,
			glm::exp(point.scale_0), glm::exp(point.scale_1), glm::exp(point.scale_2),
			sigmoid(point.opacity),
			RGB.x, RGB.y, RGB.z
			});
	}

	unsigned int VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quad_v), quad_v, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quad_f), quad_f, GL_STATIC_DRAW);
	glVertexAttribPointer(glGetAttribLocation(this_shader.ID, "quadPosition"), 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	GLuint pointsBindIdx = 1;
	GLuint sortedBindIdx = 2;
	GLuint ssbo1 = setupSSBO(pointsBindIdx, flat_gaussian_data);
	GLuint ssbo2 = 0;

	float htany = tan(glm::radians(fov) / 2);
	float htanx = htany * SCR_WIDTH / SCR_HEIGHT;
	float focal_z = SCR_HEIGHT / (2 * htany);
	glm::vec3 hfov_focal(htanx, htany, focal_z);

	while (!glfwWindowShouldClose(window)) {
		processInput(window);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / SCR_HEIGHT, znear, zfar);
		glm::mat4 viewMat = camera.GetViewMatrix();

		glDeleteBuffers(1, &ssbo2);
		// std::vector<float> gausIdx = VS::sortGaussians(Gaussian_cloud, glm::mat3(viewMat));
		// ssbo2 = setupSSBO(sortedBindIdx, gausIdx);

		this_shader.use();
		glUniformMatrix4fv(glGetUniformLocation(this_shader.ID, "view"), 1, GL_FALSE, glm::value_ptr(viewMat));
		glUniformMatrix4fv(glGetUniformLocation(this_shader.ID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
		glUniform3f(glGetUniformLocation(this_shader.ID, "hfov_focal"), hfov_focal.x, hfov_focal.y, hfov_focal.z);

		glBindVertexArray(VAO);
		glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, numInstances);
		glBindVertexArray(0);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
	glDeleteBuffers(1, &ssbo1);
	glDeleteBuffers(1, &ssbo2);
	glDeleteVertexArrays(1, &VAO);

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

int main() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Gaussian Renderer", NULL, NULL);
	if (!window) { std::cerr << "Failed to create GLFW window\n"; return -1; }
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cerr << "Failed to initialize GLAD\n"; return -1;
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	spdlog::info("初始化成功, 开始加载着色器");
	Shader this_shader("src/resources/shader/vertex_shader.glsl", "src/resources/shader/fragment_shader.glsl");

	GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
	pcl::io::loadPLYFile<GaussianData>("Z:/非结构化数据/高斯模型/输电/m77_绝缘子.ply", *Gaussian_cloud);
	int numInstances = Gaussian_cloud->points.size();
	cout << "高斯初始化成功, 点数为" << numInstances << endl;
	std::cout << "Sample point: " << Gaussian_cloud->points[0].x << ", " << Gaussian_cloud->points[0].y << Gaussian_cloud->points[0].z << std::endl;

	std::vector<float> flat_gaussian_data;
	flat_gaussian_data.reserve(numInstances * 14);
	for (const auto& point : Gaussian_cloud->points) {
		// glm::vec4 tempRots = glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3);
		glm::vec4 normRot = normalizeRotation(glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3));
		glm::vec3 RGB = SH2RGB(glm::vec3(point.f_dc_0, point.f_dc_1, point.f_dc_2));
		flat_gaussian_data.insert(flat_gaussian_data.end(), {
			point.x, point.y, point.z,
			normRot.x, normRot.y, normRot.z, normRot.w,
			glm::exp(point.scale_0), glm::exp(point.scale_1), glm::exp(point.scale_2),
			sigmoid(point.opacity),
			RGB.x, RGB.y, RGB.z
			});
	}
	cout << "读取成功" << endl;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
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

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

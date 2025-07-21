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
Camera camera(glm::vec3(0.0f, 0.0f, 0.0f)); // 或稍远一点

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


int main() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Gaussian Renderer", NULL, NULL);
	if (!window) { std::cerr << "Failed to create GLFW window\n"; return -1; }
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); // 窗口大小修改回调
	glfwSetCursorPosCallback(window, mouse_callback); // 鼠标光标移动回调
	glfwSetScrollCallback(window, scroll_callback);  // 鼠标滚轮滚动事件的回调函数
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // TODO: 增加鼠标按下按钮回调

	/*初始化glad*/
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { 
		std::cerr << "Failed to initialize GLAD\n"; return -1;
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	spdlog::info("初始化成功, 创建着色器 start");
	Shader this_shader("src/resources/shader/vertex_shader.glsl", "src/resources/shader/fragment_shader.glsl");

	spdlog::info("着色器创建成功，开始读取高斯数据");
	GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
	pcl::io::loadPLYFile<GaussianData>("Z:/非结构化数据/高斯模型/输电/m77_绝缘子.ply", *Gaussian_cloud);
	int numInstances = Gaussian_cloud->points.size();
	cout << "高斯初始化成功, 点数为" << numInstances << endl;
	std::cout << "Sample point: " << Gaussian_cloud->points[0].x << ", " << Gaussian_cloud->points[0].y << Gaussian_cloud->points[0].z << std::endl;

	std::vector<float> flat_gaussian_data;
	flat_gaussian_data.reserve(numInstances * 14);
	for (const auto& point : Gaussian_cloud->points) {
		// glm::vec4 tempRots = glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3);
		// 对旋转值四元数进行归一化，以确保四元数的幅值为1
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
	GLint quad_position = glGetAttribLocation(this_shader.ID, "quadPosition");
	glVertexAttribPointer(quad_position, 2, GL_FLOAT, GL_FALSE, 0, (void*)0); // 从着色器程序中获取quadPosition的位置
	glEnableVertexAttribArray(quad_position);

	// Unbind VBO and EBO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	GLuint pointsBindIdx = 2;
	GLuint sortedBindIdx = 1;
	GLuint ssbo1 = setupSSBO(pointsBindIdx, flat_gaussian_data); // 将SSBO绑定到binding = 2
	GLuint ssbo2 = 0;

	float htany = tan(glm::radians(fov) / 2);
	float htanx = htany * SCR_WIDTH / SCR_HEIGHT;
	float focal_z = SCR_HEIGHT / (2 * htany);
	glm::vec3 hfov_focal(htanx, htany, focal_z);

	while (!glfwWindowShouldClose(window)) {
		processInput(window);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / SCR_HEIGHT, znear, zfar);
		glm::mat4 viewMat = camera.GetViewMatrix();

		glDeleteBuffers(1, &ssbo2);
		std::vector<int> gausIdx = m_sortGaussians(Gaussian_cloud, glm::mat3(viewMat));
		ssbo2 = setupSSBO<int>(sortedBindIdx, gausIdx);

		this_shader.use();
		this_shader.setMat4("projection", projection);
		this_shader.setVec3("hfov_focal", hfov_focal);
		this_shader.setMat4("view", viewMat);
		
		/* 绑定VAO并进行实例绘制*/
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

int main1() {
	{
		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	}

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

	glEnable(GL_BLEND); // 
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // 启用alpha混合

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

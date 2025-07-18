#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include"Splats.h"
#include<pcl/io/pcd_io.h>
// #include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply_io.h>
#include"utils.cpp"
#include<vector>

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
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
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


int main() {
	// glfw: initialize and configure
   // ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	
	// configure global opengl state
	// -----------------------------
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Shader this_shader(R"(src\resources\shader\vertex_shader.glsl)", R"(src\resources\shader\fragment_shader.glsl)");

    // 0. 读取Gaussian数据
    pcl::PointCloud<GaussianData>::Ptr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
    pcl::io::loadPLYFile<GaussianData>(R"(Z:\非结构化数据\高斯模型\输电\m76_点云同步.ply)", *Gaussian_cloud);
	int numInstances = Gaussian_cloud->points.size();
    vector<float> flat_gaussian_data;
	flat_gaussian_data.reserve(numInstances * 14); // 预分配内存
	// 将数据存储到一个扁平化的一维向量中
	for (const auto& point : Gaussian_cloud->points) {
		glm::vec4 tempRots = glm::vec4(point.rot_0, point.rot_1, point.rot_2, point.rot_3);
		glm::vec4 normTempRots = normalizeRotation(tempRots);

		flat_gaussian_data.push_back(point.x);
		flat_gaussian_data.push_back(point.y);
		flat_gaussian_data.push_back(point.z);

		flat_gaussian_data.push_back(normTempRots.x);
		flat_gaussian_data.push_back(normTempRots.y);
		flat_gaussian_data.push_back(normTempRots.z);
		flat_gaussian_data.push_back(normTempRots.w);

		flat_gaussian_data.push_back(glm::exp(point.scale_0));
		flat_gaussian_data.push_back(glm::exp(point.scale_1));
		flat_gaussian_data.push_back(glm::exp(point.scale_2));

		flat_gaussian_data.push_back(sigmoid(point.opacity));

		glm::vec3 RGB = SH2RGB(glm::vec3(point.f_dc_0, point.f_dc_1, point.f_dc_2));
		flat_gaussian_data.push_back(RGB.x);
		flat_gaussian_data.push_back(RGB.y);
		flat_gaussian_data.push_back(RGB.z);

	};

	// 1. 配置数据属性
	unsigned int VAO, VBO, EBO;
	// Generate VAO, VBO, & EBO
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

	GLuint pointsBindIdx = 1;
	GLuint ssbo1 = setupSSBO(pointsBindIdx, flat_gaussian_data);

	// Unbind VBO and EBO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	float htany = tan(glm::radians(fov) / 2);
	float htanx = htany / SCR_HEIGHT * SCR_WIDTH;
	float focal_z = SCR_HEIGHT / (2 * htany);
	auto hfov_focal = glm::vec3(htanx, htany, focal_z);


	while (!glfwWindowShouldClose(window)) {
		processInput(window);

		glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, znear, zfar);
		glm::mat4 viewMat = camera.GetViewMatrix();
		auto gausIdx = sortGaussians(Gaussian_cloud, glm::mat3(viewMat));

		this_shader.use();  // 一定先激活着色器

		unsigned int projLoc = glGetUniformLocation(this_shader.ID, "projection");
		if (projLoc != -1)
			glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

		unsigned int hfovLoc = glGetUniformLocation(this_shader.ID, "hfov_focal");
		if (hfovLoc != -1)
			glUniform3f(hfovLoc, hfov_focal.x, hfov_focal.y, hfov_focal.z);

		glBindVertexArray(VAO);
		glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr, numInstances);
		glBindVertexArray(0);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

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

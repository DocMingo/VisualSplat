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

// #include<dmyDependence/dmyTool.h>

using namespace std;

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

GLuint setupSSBO(const GLuint& bindIdx, const auto& bufferData) {
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
    // 读取Gaussian数据
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

	// 数据定义
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

	GLint quad_position = glGetAttribLocation(shaderProgram, "quadPosition");
	glVertexAttribPointer(quad_position, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(quad_position);

	// Unbind VBO and EBO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);





}

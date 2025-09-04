/*
* @author: Mingo
2025-9-4: reestablish: 重建,使复原,使复位
*/
#include <Splats.h>
#include <utils.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include<pcl/common/common.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>

// #include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply_io.h>
#include<vector>
#include<spdlog/spdlog.h>
#include<fmt/format.h>
#include<filesystem>
// #include<dmyDependence/dmyTool.h>

#include "cudaGL.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "../include/Header.cuh"
#include <Eigen/Core>

using namespace std;

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
    fmt::print("device count{}", deviceCount);

    spdlog::info("着色器创建成功，开始读取高斯数据:{}", config_map["GSpath"]);
    GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
    auto ret_value = pcl::io::loadPLYFile<GaussianData>(config_map["GSpath"], *Gaussian_cloud);
    int numInstances = Gaussian_cloud->points.size();
    if (!ret_value) {
        cout << "高斯初始化成功, 点数为" << numInstances << endl;
    }

    Eigen::MatrixXd eigenMat(3, numInstances);
    eigenMat = Gaussian_cloud->getMatrixXfMap().block(0, 0, 3, numInstances);

    // 构建孪生点云，并构建kd树
    pcl::PointCloud<pcl::PointXYZ>::Ptr TwinBornCloud { new pcl::PointCloud<pcl::PointXYZ> };
	TwinBornCloud->assign(numInstances, pcl::PointXYZ()); // 容器 TwinBornCloud 的大小变为 eigenMat.cols(), 每个元素都是默认的 pcl::PointXYZ(0, 0, 0)
    TwinBornCloud->getMatrixXfMap().block(0, 0, 3, numInstances) = eigenMat;
    
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addPointCloud(TwinBornCloud, "3D Viewer");
    while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
    // pcl::PointXYZ minp, maxp;
    // pcl::getMinMax3D(*Gaussian_cloud, minp, maxp);

    // 准备高斯数据
    std::vector<float> flat_gaussian_data;
    flat_gaussian_data.reserve(numInstances * 14);
    for (const auto& point : Gaussian_cloud->points) {
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

    spdlog::info("高斯数据处理完成, 实际点数为:{}", numInstances);

}
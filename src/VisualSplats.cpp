/*
* @author: Mingo
2025-9-4: reestablish: 重建,使复原,使复位
*/
#include <iostream>
#include <Splats.h>
#include <unsuck.hpp>
// #include <glm/glm.hpp>
// #include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtc/type_ptr.hpp>
#include<pcl/common/common.h> // 基础功能，比如 getMinMax3D
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/common/transforms.h>
#include <Eigen/Geometry>

// #include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply_io.h>
#include<vector>
// #include<spdlog/spdlog.h>
#include<fmt/format.h>
#include<filesystem>
// #include<dmyDependence/dmyTool.h>

#include "cudaGL.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "../include/Header.cuh"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

int main()
{
    int a = 1;
    int b = 9;
    int middle_1 = (a + b) / 2;
    int middle_2 = (a + b) >> 1;
    cout << middle_1 << endl;
    cout << middle_2 << endl;


}
// 修改后的主函数 - 集成CUDA排序
int main1() {
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    printfmt("目前的工作路径为{}\n", fs::current_path().string());
    std::map<std::string, std::string> config_map;
    try {
        config_map = parseFileData(R"(config.txt)");
    }
    catch (std::runtime_error& e) {
        printfmt("{}\n", e.what());
    }

    printfmt("读取高斯数据：{}", config_map["GSpath"]);
    // spdlog::info("读取高斯数据:{}", config_map["GSpath"]);
    GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
    auto ret_value = pcl::io::loadPLYFile<GaussianData>(config_map["GSpath"], *Gaussian_cloud);
    int numInstances = Gaussian_cloud->points.size();
    if (!ret_value) {
        cout << "高斯初始化成功, 点数为" << numInstances << endl;
    }
    // 生成高斯孪生点云
    Eigen::MatrixXf eigenMat(3, numInstances);
    eigenMat = Gaussian_cloud->getMatrixXfMap().block(0, 0, 3, numInstances);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TwinBornCloud{ new pcl::PointCloud<pcl::PointXYZ> };
    TwinBornCloud->resize(numInstances);
    // TwinBornCloud->assign(numInstances, pcl::PointXYZ()); // 容器 TwinBornCloud 的大小变为 eigenMat.cols(), 每个元素都是默认的 pcl::PointXYZ(0, 0, 0)
    TwinBornCloud->getMatrixXfMap().block(0, 0, 3, numInstances) = eigenMat;
    viewer.addPointCloud(TwinBornCloud, "TwinCloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr denseGSCloud{ new pcl::PointCloud<pcl::PointXYZ>() }; // 用于存储高斯架构点云
    viewer.addPointCloud(denseGSCloud, "densecloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "densecloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "densecloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "TwinCloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "TwinCloud");

    int num{ 10 };
    static int indices{ 0 };
    for (const auto& gi : Gaussian_cloud->points) {
        indices++;
        // 1. 对于每个高斯椭球，生成参数表
        auto this_GaussianAxisInfo = extractGaussianAxes(gi); 
        // 2. 根据参数表生成单个稠密高斯
        pcl::PointCloud<pcl::PointXYZ>::Ptr GSpoints{ new pcl::PointCloud<pcl::PointXYZ> };
        for (int i{-num/2}; i < num/2; ++i) {
			float scale =  static_cast<float>(i)/num;
            pcl::PointXYZ thisPoint{};
			thisPoint.getVector3fMap() = this_GaussianAxisInfo.center + this_GaussianAxisInfo.axis_directions[this_GaussianAxisInfo.longest_axis_idx] * this_GaussianAxisInfo.axis_lengths[this_GaussianAxisInfo.longest_axis_idx] * scale;
            GSpoints->points.push_back(thisPoint);
        }
        

        // 2. 将稠密高斯放入总稠密点云
        *denseGSCloud += *GSpoints; // ->point 返回 vector

        if (indices % 100 == 0) {
            viewer.updatePointCloud(denseGSCloud, "densecloud");

        }
        viewer.spinOnce();
    }
    printfmt("denseGSCloud 点云数量为{}", denseGSCloud->points.size());

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}
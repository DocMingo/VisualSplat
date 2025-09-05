
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
#include<spdlog/spdlog.h>
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

// 修正版：基于高斯椭球真实主轴方向的点云生成

GaussianAxisInfo extractGaussianAxes1(const GaussianData& gi) {
    GaussianAxisInfo info;

    // 高斯中心
    info.center = gi.getVector3fMap();

    // 构建四元数
    Eigen::Vector4f quaternions;
    quaternions << gi.rot_0, gi.rot_1, gi.rot_2, gi.rot_3;

    // 获取旋转矩阵 R
    // R的列向量就是椭球在世界坐标系中的三个主轴方向！
    Eigen::Matrix3f R = quaternionToMatrix(quaternions);

    // 激活缩放参数
    Eigen::Vector3f scales;
    scales << exp(gi.scale_0), exp(gi.scale_1), exp(gi.scale_2);

    // 提取三个主轴方向（旋转矩阵的列向量）
    info.axis_directions[0] = R.col(0); // 第一个主轴方向
    info.axis_directions[1] = R.col(1); // 第二个主轴方向  
    info.axis_directions[2] = R.col(2); // 第三个主轴方向

    // 对应的轴长（使用3σ原则）
    info.axis_lengths[0] = 3.0f * scales[0];
    info.axis_lengths[1] = 3.0f * scales[1];
    info.axis_lengths[2] = 3.0f * scales[2];

    // 找到最长轴
    info.longest_axis_idx = 0;
    for (int i = 1; i < 3; i++) {
        if (info.axis_lengths[i] > info.axis_lengths[info.longest_axis_idx]) {
            info.longest_axis_idx = i;
        }
    }

    return info;
}

// 3. 基于真实主轴方向生成点云
pcl::PointCloud<pcl::PointXYZ>::Ptr generateGaussianAxisCloud(const GaussianData& gi, int numPoints = 40) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 提取椭球主轴信息
    GaussianAxisInfo axisInfo = extractGaussianAxes1(gi);

    // 沿最长轴生成点云
    int longestIdx = axisInfo.longest_axis_idx;
    Eigen::Vector3f longestAxis = axisInfo.axis_directions[longestIdx];
    float longestLength = axisInfo.axis_lengths[longestIdx];

    // 在最长轴上生成点
    for (int i = -numPoints / 2; i <= numPoints / 2; i++) {
        float t = (float)i / (numPoints / 2); // 归一化参数 [-1, 1]

        // 沿真实主轴方向生成点
        Eigen::Vector3f point = axisInfo.center + t * longestLength * longestAxis;

        pcl::PointXYZ pclPoint;
        pclPoint.getVector3fMap() = point;
        cloud->push_back(pclPoint);
    }

    return cloud;
}

// 4. 生成所有三个主轴的点云（可选）
pcl::PointCloud<pcl::PointXYZ>::Ptr generateAllAxesCloud(const GaussianData& gi, int numPointsPerAxis = 20) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 提取椭球主轴信息
    GaussianAxisInfo axisInfo = extractGaussianAxes1(gi);

    // 为每个主轴生成点
    for (int axisIdx = 0; axisIdx < 3; axisIdx++) {
        Eigen::Vector3f axisDirection = axisInfo.axis_directions[axisIdx];
        float axisLength = axisInfo.axis_lengths[axisIdx];

        // 沿当前轴生成点
        for (int i = -numPointsPerAxis / 2; i <= numPointsPerAxis / 2; i++) {
            float t = (float)i / (numPointsPerAxis / 2);

            Eigen::Vector3f point = axisInfo.center + t * axisLength * axisDirection;

            pcl::PointXYZ pclPoint;
            pclPoint.getVector3fMap() = point;
            cloud->push_back(pclPoint);
        }
    }

    return cloud;
}

// 5. 主函数修正版本
int main() {
    pcl::visualization::PCLVisualizer viewer("3D Viewer");

    /*=== load config file ===*/
    std::map<std::string, std::string> config_map = parseFileData(R"(D:\Work\VSProject\VisualSplat\src\resources\config.txt)");
    spdlog::info("读取高斯数据:{}", config_map["GSpath"]);

    GScloudPtr Gaussian_cloud(new pcl::PointCloud<GaussianData>);
    auto ret_value = pcl::io::loadPLYFile<GaussianData>(config_map["GSpath"], *Gaussian_cloud);
    int numInstances = Gaussian_cloud->points.size();
    if (!ret_value) {
        cout << "高斯初始化成功, 点数为" << numInstances << endl;
    }

    // 原始孪生点云
    Eigen::MatrixXf eigenMat(3, numInstances);
    eigenMat = Gaussian_cloud->getMatrixXfMap().block(0, 0, 3, numInstances);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TwinBornCloud{ new pcl::PointCloud<pcl::PointXYZ> };
    TwinBornCloud->resize(numInstances);
    TwinBornCloud->getMatrixXfMap().block(0, 0, 3, numInstances) = eigenMat;
    viewer.addPointCloud(TwinBornCloud, "TwinCloud");

    // 密集点云（主轴点云）
    pcl::PointCloud<pcl::PointXYZ>::Ptr denseCloud{ new pcl::PointCloud<pcl::PointXYZ>() };
    viewer.addPointCloud(denseCloud, "densecloud");

    // 设置渲染属性
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "densecloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "densecloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "TwinCloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "TwinCloud");

    // 处理每个高斯点
    for (size_t i = 0; i < Gaussian_cloud->points.size(); ++i) {
        const auto& gi = Gaussian_cloud->points[i];

        // 方法1：只生成最长轴点云
        auto axisCloud = generateGaussianAxisCloud(gi, 40);
        *denseCloud += *axisCloud;

        // 方法2：生成所有三个轴的点云（可选，注释掉方法1使用这个）
        // auto allAxesCloud = generateAllAxesCloud(gi, 20);
        // *denseCloud += *allAxesCloud;

        // 可选：实时更新显示（会很慢）
        if (i % 100 == 0) {  // 每100个点更新一次显示
            viewer.updatePointCloud(denseCloud, "densecloud");
            viewer.spinOnce(1);
            std::cout << "处理进度: " << i << "/" << Gaussian_cloud->points.size() << std::endl;
        }
    }

    // 最终更新显示
    viewer.updatePointCloud(denseCloud, "densecloud");
    std::cout << "处理完成！生成的密集点数量: " << denseCloud->points.size() << std::endl;

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    // 基于点云密度构建杆塔框架

	// 基于点云法线构建杆塔框架

    return 0;
}

// 6. 调试函数：可视化椭球的三个主轴方向
void visualizeGaussianAxes(pcl::visualization::PCLVisualizer& viewer, const GaussianData& gi, const std::string& id) {
    GaussianAxisInfo axisInfo = extractGaussianAxes(gi);

    // 用不同颜色的线段显示三个主轴
    std::vector<std::array<float, 3>> colors = { {1,0,0}, {0,1,0}, {0,0,1} }; // RGB

    for (int i = 0; i < 3; i++) {
        pcl::PointXYZ start, end;

        Eigen::Vector3f startPos = axisInfo.center - 0.5f * axisInfo.axis_lengths[i] * axisInfo.axis_directions[i];
        Eigen::Vector3f endPos = axisInfo.center + 0.5f * axisInfo.axis_lengths[i] * axisInfo.axis_directions[i];

        start.getVector3fMap() = startPos;
        end.getVector3fMap() = endPos;

        std::string lineId = id + "_axis_" + std::to_string(i);
        viewer.addLine(start, end, colors[i][0], colors[i][1], colors[i][2], lineId);
    }
}
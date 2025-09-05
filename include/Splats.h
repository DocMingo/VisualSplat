#pragma once

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<string>
#include<Eigen/core>

struct GaussianData {
	PCL_ADD_POINT4D; // property float x y z; the mean of the splat

	float nx, ny, nz; // the normal of the splat, not sure what this is used for
	float f_dc_0, f_dc_1, f_dc_2; // the spherical harmonics of the splat with the diffuse colour
	float opacity; // the opacity of the splat
	float scale_0, scale_1, scale_2; // the scale of the splat
	float rot_0, rot_1, rot_2, rot_3; // the rotation of the splat

	PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
	GaussianData,
	(float, x, x) (float, y, y) (float, z, z)
	(float, nx, nx) (float, ny, ny) (float, nz, nz)
	(float, f_dc_0, f_dc_0) (float, f_dc_1, f_dc_1) (float, f_dc_2, f_dc_2)
	(float, opacity, opacity)
	(float, scale_0, scale_0) (float, scale_1, scale_1) (float, scale_2, scale_2)
	(float, rot_0, rot_0) (float, rot_1, rot_1) (float, rot_2, rot_2) (float, rot_3, rot_3)
)

using GScloud = pcl::PointCloud<GaussianData>;
using GScloudPtr = GScloud::Ptr;

struct GaussianAxisInfo {
	Eigen::Vector3f axis_directions[3];  // 三个主轴在世界坐标系中的方向
	float axis_lengths[3];               // 对应的轴长
	int longest_axis_idx;                // 最长轴的索引
	Eigen::Vector3f center;              // 椭球中心
};
// 提取高斯椭球的主轴信息
GaussianAxisInfo extractGaussianAxes(const GaussianData& thisGS);
// 四元数转旋转矩阵
Eigen::Matrix3f quaternionToMatrix(const Eigen::Vector4f& q);
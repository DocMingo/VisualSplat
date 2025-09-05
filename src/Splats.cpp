#include<Splats.h>
#include<Eigen/dense>
#include<iostream>

GaussianAxisInfo extractGaussianAxes(const GaussianData& thisGS) {
    GaussianAxisInfo thisGSAxisInfo;

	thisGSAxisInfo.center = thisGS.getVector3fMap();

    Eigen::Vector4f quaternions; // 四元数
	quaternions << thisGS.rot_0, thisGS.rot_1, thisGS.rot_2, thisGS.rot_3;
    Eigen::Matrix3f R = quaternionToMatrix(quaternions);

    std::cout << R;

	Eigen::Vector3f scales; // 缩放
	scales << exp(thisGS.scale_0), exp(thisGS.scale_1), exp(thisGS.scale_2);

	// 提取三个主轴方向（旋转矩阵的列向量）
	thisGSAxisInfo.axis_directions[0] = R.col(0); // 第一个主轴方向
	thisGSAxisInfo.axis_directions[1] = R.col(1); // 第二个主轴方向  
	thisGSAxisInfo.axis_directions[2] = R.col(2); // 第三个主轴方向

	// 对应的轴长（使用3σ原则）
	thisGSAxisInfo.axis_lengths[0] = 3.0f * scales[0];
	thisGSAxisInfo.axis_lengths[1] = 3.0f * scales[1];
	thisGSAxisInfo.axis_lengths[2] = 3.0f * scales[2];

	// 找到最长轴
	thisGSAxisInfo.longest_axis_idx = 0;
	for (int i = 1; i < 3; i++) {
		if (thisGSAxisInfo.axis_lengths[i] > thisGSAxisInfo.axis_lengths[thisGSAxisInfo.longest_axis_idx]) {
			thisGSAxisInfo.longest_axis_idx = i;
		}
	}

	return thisGSAxisInfo;

}



// 四元数转旋转矩阵
Eigen::Matrix3f quaternionToMatrix(const Eigen::Vector4f& q) {
    float r = q[0]; // w (实部)
    float x = q[1]; // x
    float y = q[2]; // y  
    float z = q[3]; // z

    // 四元数归一化
    float norm = sqrt(r * r + x * x + y * y + z * z);
    if (norm > 1e-7) {
        r /= norm; x /= norm; y /= norm; z /= norm;
    }

    // 构建旋转矩阵
    Eigen::Matrix3f R;
    R << 1.f - 2.f * (y * y + z * z), 2.f * (x * y - r * z), 2.f * (x * z + r * y),
        2.f * (x * y + r * z), 1.f - 2.f * (x * x + z * z), 2.f * (y * z - r * x),
        2.f * (x * z - r * y), 2.f * (y * z + r * x), 1.f - 2.f * (x * x + y * y);

    return R;
}
#pragma once

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>
#include <thrust/sort.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>
#include <glm/glm.hpp>

// CUDA核函数：计算每个Gaussian点在视图空间的深度
__global__ void computeDepthKernel(
    const float* gaussianData,    // 输入：Gaussian数据 (x,y,z,...)
    float* depths,                // 输出：深度值
    int* indices,                 // 输出：索引
    const float* viewMatrix,      // 视图矩阵 (3x3)
    int numPoints
);

class CudaGaussianSorter {
private:
    cudaGraphicsResource* vbo_cuda_resource;
    cudaGraphicsResource* ssbo_cuda_resource;

    float* d_depths;              // GPU深度数组
    int* d_indices;               // GPU索引数组
    float* d_viewMatrix;          // GPU视图矩阵

    thrust::device_vector<float> thrust_depths;
    thrust::device_vector<int> thrust_indices;

    int numPoints;
    bool initialized;

public:
    CudaGaussianSorter() :
        vbo_cuda_resource(nullptr),
        ssbo_cuda_resource(nullptr),
        d_depths(nullptr),
        d_indices(nullptr),
        d_viewMatrix(nullptr),
        numPoints(0),
        initialized(false) {}

    ~CudaGaussianSorter() {
        cleanup();
    }

    void cleanup();
};
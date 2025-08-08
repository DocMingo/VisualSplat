#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>
#include <thrust/sort.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>
#include "glm/glm.hpp"
#include"Header.cuh"

// CUDA核函数：计算每个Gaussian点在视图空间的深度
__global__ void computeDepthKernel(
    const float* gaussianData,    // 输入：Gaussian数据 (x,y,z,...)
    float* depths,                // 输出：深度值
    int* indices,                 // 输出：索引
    const float* viewMatrix,      // 视图矩阵 (3x3)
    int numPoints
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < numPoints) {
        // 从flat数据中提取位置 (假设前3个float是x,y,z)
        float x = gaussianData[idx * 14 + 0];
        float y = gaussianData[idx * 14 + 1];
        float z = gaussianData[idx * 14 + 2];

        // 应用视图矩阵变换 (只需要z分量)
        float viewZ = viewMatrix[6] * x + viewMatrix[7] * y + viewMatrix[8] * z;

        depths[idx] = viewZ;
        indices[idx] = idx;
    }
}


// 初始化CUDA资源和OpenGL互操作
bool CudaGaussianSorter::initialize(GLuint gaussianSSBO, GLuint indexSSBO, int pointCount) {
    numPoints = pointCount;

    // 注册OpenGL buffer到CUDA
    cudaError_t err = cudaGraphicsGLRegisterBuffer(
        &ssbo_cuda_resource,
        indexSSBO,
        cudaGraphicsMapFlagsWriteDiscard
    );
    if (err != cudaSuccess) {
        std::cerr << "Failed to register index SSBO: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    // 分配CUDA内存
    cudaMalloc(&d_depths, numPoints * sizeof(float));
    cudaMalloc(&d_indices, numPoints * sizeof(int));
    cudaMalloc(&d_viewMatrix, 9 * sizeof(float)); // 3x3矩阵

    // 初始化Thrust向量
    thrust_depths.resize(numPoints);
    thrust_indices.resize(numPoints);

    initialized = true;
    return true;
}

// 执行CUDA排序
void CudaGaussianSorter::sortGaussians(GLuint gaussianSSBO, const glm::mat3& viewMat) {
    if (!initialized) return;

    // 将视图矩阵拷贝到GPU
    cudaMemcpy(d_viewMatrix, &viewMat[0][0], 9 * sizeof(float), cudaMemcpyHostToDevice);

    // 映射OpenGL buffer到CUDA
    float* d_gaussianData;
    size_t gaussianDataSize;

    cudaGraphicsResource* gaussianResource;
    cudaGraphicsGLRegisterBuffer(&gaussianResource, gaussianSSBO, cudaGraphicsMapFlagsReadOnly);
    cudaGraphicsMapResources(1, &gaussianResource);
    cudaGraphicsResourceGetMappedPointer((void**)&d_gaussianData, &gaussianDataSize, gaussianResource);

    // 启动CUDA核函数计算深度
    int blockSize = 256;
    int gridSize = (numPoints + blockSize - 1) / blockSize;

    computeDepthKernel <<< gridSize, blockSize >>> (
        d_gaussianData, d_depths, d_indices, d_viewMatrix, numPoints
        );

    cudaDeviceSynchronize();

    // 解除Gaussian SSBO映射
    cudaGraphicsUnmapResources(1, &gaussianResource);
    cudaGraphicsUnregisterResource(gaussianResource);

    // 使用Thrust进行排序
    thrust::copy(thrust::device_pointer_cast(d_depths),
        thrust::device_pointer_cast(d_depths + numPoints),
        thrust_depths.begin());
    thrust::copy(thrust::device_pointer_cast(d_indices),
        thrust::device_pointer_cast(d_indices + numPoints),
        thrust_indices.begin());

    // 按深度排序索引
    thrust::sort_by_key(thrust_depths.begin(), thrust_depths.end(), thrust_indices.begin());

    // 将排序结果直接写入OpenGL SSBO
    int* d_sortedIndices;
    size_t indexDataSize;

    cudaGraphicsMapResources(1, &ssbo_cuda_resource);
    cudaGraphicsResourceGetMappedPointer((void**)&d_sortedIndices, &indexDataSize, ssbo_cuda_resource);

    // 拷贝排序后的索引到OpenGL buffer
    thrust::copy(thrust_indices.begin(), thrust_indices.end(),
        thrust::device_pointer_cast(d_sortedIndices));

    cudaGraphicsUnmapResources(1, &ssbo_cuda_resource);
}

void CudaGaussianSorter::cleanup() {
    if (initialized) {
        if (ssbo_cuda_resource) {
            cudaGraphicsUnregisterResource(ssbo_cuda_resource);
        }

        if (d_depths) cudaFree(d_depths);
        if (d_indices) cudaFree(d_indices);
        if (d_viewMatrix) cudaFree(d_viewMatrix);

        initialized = false;
    }
}
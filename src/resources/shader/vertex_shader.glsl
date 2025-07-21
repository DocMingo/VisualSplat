#version 430 core

// 输入的一个二维四边形的顶点坐标（如：[-1, -1], [1, -1], [-1, 1], [1, 1]）
layout(location = 0) in vec2 quadPosition;

// 定义高斯数据在数据向量中的偏移量
#define POS_IDX      0   // 中心位置
#define ROT_IDX      3   // 四元数旋转
#define SCALE_IDX    7   // 缩放因子
#define OPACITY_IDX 10   // 不透明度
#define SH_IDX      11   // 球谐系数（这里只取前3个）
#define SH_DIM       3   // 球谐维度

// 高斯索引排列缓存（排序后的渲染顺序）
// std430表示内存对其鬼册是按照4字节vec4对其规则排布
layout(std430, binding = 1) buffer gaussians_order { 
    int sortedGaussianIdx[];
};

// 高斯原始属性数据缓存
layout(std430, binding = 2) buffer gaussians_data {
    float gData[];
};

// uniform 参数
uniform mat4 view;
uniform mat4 projection;
uniform vec3 hfov_focal; // (fx, fy, f) 即 focal_x, focal_y, focal

// 传递给 fragment shader 的变量
out vec3 outColor;
out float opacity;
out vec2 coordxy;  // 当前 quad 中某像素相对于中心的坐标
out vec3 conic;    // conic 系数（定义椭圆形状）

// 辅助函数：从 gData 中获取向量
vec3 get_vec3(int offset) {
    return vec3(gData[offset], gData[offset + 1], gData[offset + 2]);
}
vec4 get_vec4(int offset) {
    return vec4(gData[offset], gData[offset + 1], gData[offset + 2], gData[offset + 3]);
}

// 构建旋转-缩放矩阵对应的 3D 协方差矩阵
mat3 computeCov3D(vec4 rots, vec3 scales) {
    float scaleMod = 1.0f;

    vec3 row0 = vec3(
        1.0 - 2.0 * (rots.z * rots.z + rots.w * rots.w),
        2.0 * (rots.y * rots.z - rots.x * rots.w),
        2.0 * (rots.y * rots.w + rots.x * rots.z)
    );

    vec3 row1 = vec3(
        2.0 * (rots.y * rots.z + rots.x * rots.w),
        1.0 - 2.0 * (rots.y * rots.y + rots.w * rots.w),
        2.0 * (rots.z * rots.w - rots.x * rots.y)
    );

    vec3 row2 = vec3(
        2.0 * (rots.y * rots.w - rots.x * rots.z),
        2.0 * (rots.z * rots.w + rots.x * rots.y),
        1.0 - 2.0 * (rots.y * rots.y + rots.z * rots.z)
    );

    mat3 scaleMat = mat3(
        scaleMod * scales.x, 0, 0,
        0, scaleMod * scales.y, 0,
        0, 0, scaleMod * scales.z
    );

    mat3 rotMat = mat3(row0, row1, row2);
    mat3 mMat = scaleMat * rotMat;

    return transpose(mMat) * mMat; // 协方差矩阵 sigma
}

void main() {
    // 1. 获取当前实例对应的高斯数据起始索引
    int quadId = sortedGaussianIdx[gl_InstanceID];
    int total_dim = 3 + 4 + 3 + 1 + SH_DIM;
    int start = quadId * total_dim;

    vec3 center = get_vec3(start + POS_IDX);
    vec4 rotation = get_vec4(start + ROT_IDX);
    vec3 scale = get_vec3(start + SCALE_IDX);
    vec3 colorVal = get_vec3(start + SH_IDX);

    // 2. 计算协方差矩阵
    mat3 cov3d = computeCov3D(rotation, scale);

    // 3. 应用视图变换和投影变换
    vec4 cam = view * vec4(center, 1.0);
    vec4 pos2d = projection * cam;
    pos2d.xyz /= pos2d.w;
    pos2d.w = 1.0;

    vec2 wh = 2 * hfov_focal.xy * hfov_focal.z;

    // Set limits to avoid extreme perspective distortion & contrain effects of outliers
    float limx = 1.3 * hfov_focal.x;
    float limy = 1.3 * hfov_focal.y;

    float txtz = cam.x / cam.z;
    float tytz = cam.y / cam.z;

    // Clamped versions of txtz and tytz 
    float tx = min(limx, max(-limx, txtz)) * cam.z;
    float ty = min(limy, max(-limy, tytz)) * cam.z;
    // 4. 设置近平面约束，裁剪太远或太偏的高斯点
    if (any(greaterThan(abs(pos2d.xyz), vec3(1.3)))) {
        gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        return;
    }

    // 5. 计算雅可比矩阵并近似投影变换

    mat3 J = mat3(
        hfov_focal.z / cam.z, 0.0, -(hfov_focal.z * tx) / (cam.z * cam.z),
        0.0, hfov_focal.z / cam.z, -(hfov_focal.z * ty) / (cam.z * cam.z),
        0.0, 0.0, 0.0
    );

    mat3 T = transpose(mat3(view)) * J;
    mat3 cov2dMat = transpose(T) * transpose(cov3d) * T;

    // 6. 添加数值稳定项
    cov2dMat[0][0] += 0.3;
    cov2dMat[1][1] += 0.3;

    float det = cov2dMat[0][0] * cov2dMat[1][1] - cov2dMat[0][1] * cov2dMat[1][0];
    if (det == 0.0) {
        gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // 7. 将协方差转换为椭圆形式的二次型 conic（xAx + 2Bxy + Cy²）
    float det_inv = 1.0 / det;
    conic = vec3(
        cov2dMat[1][1] * det_inv,
        -cov2dMat[0][1] * det_inv,
        cov2dMat[0][0] * det_inv
    );

    // 8. 四边形尺寸估计（椭圆的半轴 * 3）
    vec2 quadwh_scr = vec2(3.0 * sqrt(cov2dMat[0][0]), 3.0 * sqrt(cov2dMat[1][1]));
    vec2 quadwh_ndc = quadwh_scr / wh * 2.0;

    // 9. 更新该四边形顶点在 NDC 中的位置
    pos2d.xy += quadPosition * quadwh_ndc;
    gl_Position = pos2d;

    // 10. 传值给片段着色器
    coordxy = quadPosition * quadwh_scr;
    outColor = colorVal;
    // outColor = vec3(1.0f, 0.0f, 0.0f); // 强制输出红色
    opacity = gData[start + OPACITY_IDX];
}

#version 430 core

// 输入的一个二维四边形的顶点坐标（如：[-1, -1], [1, -1], [-1, 1], [1, 1]）
layout(location = 0) in vec2 quadPosition;

// 定义高斯数据在数据向量中的偏移量
#define POS_IDX      0   // 中心位置
#define ROT_IDX      3   // 四元数旋转
#define SCALE_IDX    7   // 缩放因子
#define OPACITY_IDX 10   // 不透明度
#define SH_IDX      11   // 球谐系数（这里只取前3个）
// #define SH_DIM       3   // 球谐维度
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
uniform float scaleMod; // 整体缩放因子

// 传递给 fragment shader 的变量
out vec3 outColor;
out float opacity;
out vec2 coordxy;  // 当前 quad 中某像素相对于中心的坐标
out vec3 conic;    // conic 系数（定义椭圆形状）

// float scaleMod = 1.01f; // 经验值，整体缩放因子

// 辅助函数：从 gData 中获取向量
vec3 get_vec3(int offset) {
    return vec3(gData[offset], gData[offset + 1], gData[offset + 2]);
}
vec4 get_vec4(int offset) {
    return vec4(gData[offset], gData[offset + 1], gData[offset + 2], gData[offset + 3]);
}

// 构建旋转-缩放矩阵对应的 3D 协方差矩阵 
// 保持某个固定方向的缩放，而旋转只是改变整体姿态。
mat3 computeCov3D(vec4 rots, vec3 scales) {


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

mat3 computeCov3D_rota(vec4 rots, vec3 scales) {
    // 归一化四元数
    rots = normalize(rots);

    mat3 R = mat3(
        1.0 - 2.0 * (rots.y * rots.y + rots.z * rots.z),
        2.0 * (rots.x * rots.y - rots.z * rots.w),      
        2.0 * (rots.x * rots.z + rots.y * rots.w),      

        2.0 * (rots.x * rots.y + rots.z * rots.w),      
        1.0 - 2.0 * (rots.x * rots.x + rots.z * rots.z),
        2.0 * (rots.y * rots.z - rots.x * rots.w),      

        2.0 * (rots.x * rots.z - rots.y * rots.w),      
        2.0 * (rots.y * rots.z + rots.x * rots.w),      
        1.0 - 2.0 * (rots.x * rots.x + rots.y * rots.y) 
    );

    // 旋转矩阵
    mat3 R1 = mat3(
        1.0 - 2.0 * (rots.z * rots.z + rots.w * rots.w),
        2.0 * (rots.y * rots.z + rots.x * rots.w),
        2.0 * (rots.y * rots.w - rots.x * rots.z),

        2.0 * (rots.y * rots.z - rots.x * rots.w),
        1.0 - 2.0 * (rots.y * rots.y + rots.w * rots.w),
        2.0 * (rots.z * rots.w + rots.x * rots.y),

        2.0 * (rots.y * rots.w + rots.x * rots.z),
        2.0 * (rots.z * rots.w - rots.x * rots.y),
        1.0 - 2.0 * (rots.y * rots.y + rots.z * rots.z)
    );

    // 缩放矩阵
    mat3 S = mat3(
        scales.x, 0.0, 0.0,
        0.0, scales.y, 0.0,
        0.0, 0.0, scales.z
    );

    mat3 M = R * S;
    return  M * transpose(M);
}

// ===== 深度到颜色映射函数 =====
// HSV到RGB转换
vec3 hsv2rgb(vec3 c) {
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}
// 方法1: 热力图颜色映射 (蓝->绿->黄->红)
vec3 mapDepthToColor_Heatmap(float depth, float near, float far) {
    // 标准化深度到[0,1]
    float normalizedDepth = clamp((depth - near) / (far - near), 0.0, 1.0);

    vec3 color;
    if (normalizedDepth < 0.25) {
        // 蓝到青
        float t = normalizedDepth / 0.25;
        color = mix(vec3(0.0, 0.0, 1.0), vec3(0.0, 1.0, 1.0), t);
    }
    else if (normalizedDepth < 0.5) {
        // 青到绿
        float t = (normalizedDepth - 0.25) / 0.25;
        color = mix(vec3(0.0, 1.0, 1.0), vec3(0.0, 1.0, 0.0), t);
    }
    else if (normalizedDepth < 0.75) {
        // 绿到黄
        float t = (normalizedDepth - 0.5) / 0.25;
        color = mix(vec3(0.0, 1.0, 0.0), vec3(1.0, 1.0, 0.0), t);
    }
    else {
        // 黄到红
        float t = (normalizedDepth - 0.75) / 0.25;
        color = mix(vec3(1.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), t);
    }

    return color;
}

// 方法2: 简单的线性颜色映射 (蓝->红)
vec3 mapDepthToColor_Linear(float depth, float near, float far) {
    float normalizedDepth = clamp((depth - near) / (far - near), 0.0, 1.0);
    return mix(vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), normalizedDepth);
}

// 方法3: 彩虹颜色映射
vec3 mapDepthToColor_Rainbow(float depth, float near, float far) {
    float normalizedDepth = clamp((depth - near) / (far - near), 0.0, 1.0);
    float hue = normalizedDepth * 300.0; // 0-300度的色相
    return hsv2rgb(vec3(hue / 360.0, 1.0, 1.0));
}



// 方法4: 分层颜色映射（等高线效果）
vec3 mapDepthToColor_Layers(float depth, float near, float far, int numLayers) {
    float normalizedDepth = clamp((depth - near) / (far - near), 0.0, 1.0);
    float layerSize = 1.0 / float(numLayers);
    int layer = int(normalizedDepth / layerSize);

    // 为每层分配不同颜色
    vec3 layerColors[8] = vec3[8](
        vec3(0.0, 0.0, 1.0),   // 蓝
        vec3(0.0, 0.5, 1.0),   // 浅蓝
        vec3(0.0, 1.0, 1.0),   // 青
        vec3(0.0, 1.0, 0.0),   // 绿
        vec3(0.5, 1.0, 0.0),   // 黄绿
        vec3(1.0, 1.0, 0.0),   // 黄
        vec3(1.0, 0.5, 0.0),   // 橙
        vec3(1.0, 0.0, 0.0)    // 红
        );

    return layerColors[min(layer, 7)];
}

// 方法5: 对数深度映射（适合大范围深度）
vec3 mapDepthToColor_Log(float depth, float near, float far) {
    float logDepth = log(depth / near) / log(far / near);
    float normalizedDepth = clamp(logDepth, 0.0, 1.0);
    return mapDepthToColor_Heatmap(normalizedDepth, 0.0, 1.0);
}

// ===== 通用的深度颜色映射函数 =====
vec3 mapDepthToColor(float depth, float near, float far) {
    // 可以通过uniform变量控制使用哪种映射方式
    // uniform int depthColorMode; 

    // 这里默认使用热力图映射
    return mapDepthToColor_Heatmap(depth, near, far);

    /* 其他选项:
    if (depthColorMode == 0) return mapDepthToColor_Linear(depth, near, far);
    else if (depthColorMode == 1) return mapDepthToColor_Heatmap(depth, near, far);
    else if (depthColorMode == 2) return mapDepthToColor_Rainbow(depth, near, far);
    else if (depthColorMode == 3) return mapDepthToColor_Layers(depth, near, far, 8);
    else return mapDepthToColor_Log(depth, near, far);
    */
}


void main() {
    // 1. 获取当前实例对应的高斯数据起始索引
    int quadId = sortedGaussianIdx[gl_InstanceID]; // GPU 在执行顶点着色器时，会引入gl_InstanceID变量，值是当前实例的编号，从0开始递增
    int total_dim = 3 + 4 + 3 + 1 + SH_DIM;
    int start = quadId * total_dim;

    vec3 center = get_vec3(start + POS_IDX);
    vec4 rotation = get_vec4(start + ROT_IDX);
    vec3 scale = get_vec3(start + SCALE_IDX);
    vec3 colorVal = get_vec3(start + SH_IDX);

    vec4 cam = view * vec4(center, 1.0);
    // 早期剔除：背面剔除, 距离相机为负距离
    if (cam.z > 0.0) {
        gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        return;
    }

    // 根据高斯深度改变颜色;
    
    // 2. 计算协方差矩阵
    mat3 cov3d = computeCov3D(rotation, scale);
    // mat3 cov3d = computeCov3D_rota(rotation, scale);

    // 3. 应用视图变换和投影变换
    vec4 pos2d = projection * cam; // cam 是相机空间的点坐标 (x, y, z)
    pos2d.xyz /= pos2d.w; // 除以 w 得到 标准化设备坐标 (NDC)，范围通常是 [-1,1]
    pos2d.w = 1.0; // pos2d 就是最终用来 gl_Position 的位置

    vec2 wh = 2 * hfov_focal.xy * hfov_focal.z; // hfov_focal 表示相机水平/垂直视野和焦距

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
        // gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        // return;
    }

    // 5. 计算雅可比矩阵并近似投影变换
    mat3 J = mat3(
        hfov_focal.z / cam.z, 0.0, -(hfov_focal.z * tx) / (cam.z * cam.z),
        0.0, hfov_focal.z / cam.z, -(hfov_focal.z * ty) / (cam.z * cam.z),
        0.0, 0.0, 0.0
    );

    mat3 T = transpose(mat3(view)) * J;
    mat3 cov2dMat = transpose(T) * transpose(cov3d) * T;

    // mat3 T = J * transpose(mat3(view));  // 顺序反过来
    // mat3 cov2dMat = T * cov3d * transpose(T);  // 不需要额外的transpose

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

    float depth = -cam.z;
    vec3 depthColor = mapDepthToColor_Rainbow(depth, 0, 100);

    // 10. 传值给片段着色器
    coordxy = quadPosition * quadwh_scr;
    outColor = colorVal;
    // outColor = vec3(1.0f, 0.0f, 0.0f); // 强制输出红色
    // outColor = depthColor;
    opacity = gData[start + OPACITY_IDX];
}

#version 430 core

layout(location = 0) in vec2 quadPosition;

// 数据布局定义
#define POS_IDX      0
#define ROT_IDX      3
#define SCALE_IDX    7
#define OPACITY_IDX 10
#define SH_IDX      11
#define SH_DIM       3
#define SH_COEFFS   16  // 完整的球谐系数数量

layout(std430, binding = 1) buffer gaussians_order { 
    int sortedGaussianIdx[];
};

layout(std430, binding = 2) buffer gaussians_data {
    float gData[];
};

// Uniforms
uniform mat4 view;
uniform mat4 projection;
uniform vec3 hfov_focal;
uniform vec3 camera_position;  // 相机位置用于球谐计算
uniform float gamma_correction = 2.2;
uniform vec3 color_scale = vec3(1.0);
uniform vec3 color_offset = vec3(0.0);
uniform float saturation = 1.0;

// 输出变量
out vec3 outColor;
out float opacity;
out vec2 coordxy;
out vec3 conic;

// 球谐基函数 (简化版本)
float SH_basis_0() { return 0.28209479177387814; }
float SH_basis_1(vec3 dir) { return 0.4886025119029199 * dir.y; }
float SH_basis_2(vec3 dir) { return 0.4886025119029199 * dir.z; }
float SH_basis_3(vec3 dir) { return 0.4886025119029199 * dir.x; }

// 辅助函数
vec3 get_vec3(int offset) {
    return vec3(gData[offset], gData[offset + 1], gData[offset + 2]);
}

vec4 get_vec4(int offset) {
    return vec4(gData[offset], gData[offset + 1], gData[offset + 2], gData[offset + 3]);
}

// 计算球谐光照 (前4个基函数)
vec3 evaluateSH(vec3 dir, int start_idx) {
    vec3 color = vec3(0.0);
    
    // DC component (L=0, M=0)
    color += get_vec3(start_idx) * SH_basis_0();
    
    // L=1 components
    if (SH_COEFFS > 3) {
        color += get_vec3(start_idx + 3) * SH_basis_1(dir);
        color += get_vec3(start_idx + 6) * SH_basis_2(dir); 
        color += get_vec3(start_idx + 9) * SH_basis_3(dir);
    }
    
    return color;
}

// 应用饱和度
vec3 applySaturation(vec3 color, float sat) {
    vec3 grey = vec3(dot(color, vec3(0.299, 0.587, 0.114)));
    return mix(grey, color, sat);
}

// 伽马校正
vec3 applyGammaCorrection(vec3 color, float gamma) {
    return pow(max(color, vec3(0.0)), vec3(1.0 / gamma));
}

// 计算3D协方差矩阵
mat3 computeCov3D(vec4 rots, vec3 scales) {
    // 归一化四元数
    rots = normalize(rots);
    
    // 旋转矩阵
    mat3 R = mat3(
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
    return transpose(M) * M;
}

void main() {
    // 获取高斯数据
    int quadId = sortedGaussianIdx[gl_InstanceID];
    int total_dim = 3 + 4 + 3 + 1 + SH_DIM;
    int start = quadId * total_dim;

    vec3 center = get_vec3(start + POS_IDX);
    vec4 rotation = get_vec4(start + ROT_IDX);
    vec3 scale = get_vec3(start + SCALE_IDX);
    float opacity_val = gData[start + OPACITY_IDX];

    // 视图变换
    vec4 cam = view * vec4(center, 1.0);
    
    // 早期剔除：背面剔除
    if (cam.z > 0.0) {
        gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        return;
    }

    // 投影变换
    vec4 pos2d = projection * cam;
    pos2d.xyz /= pos2d.w;

    // 视锥剔除
    if (any(greaterThan(abs(pos2d.xy), vec2(1.3)))) {
        gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        return;
    }

    // 计算协方差矩阵
    mat3 cov3d = computeCov3D(rotation, scale);

    // 投影雅可比矩阵 (改进的计算)
    float focal_x = hfov_focal.z;
    float focal_y = hfov_focal.z;
    
    mat3 J = mat3(
        focal_x / cam.z, 0.0, -(focal_x * cam.x) / (cam.z * cam.z),
        0.0, focal_y / cam.z, -(focal_y * cam.y) / (cam.z * cam.z),
        0.0, 0.0, 0.0
    );

    // 视图变换的旋转部分
    mat3 W = transpose(mat3(view));
    mat3 T = W * J;

    // 2D协方差矩阵
    mat3 cov2d = transpose(T) * cov3d * T;

    // 数值稳定性
    cov2d[0][0] += 0.3;
    cov2d[1][1] += 0.3;

    float det = cov2d[0][0] * cov2d[1][1] - cov2d[0][1] * cov2d[1][0];
    if (abs(det) < 1e-6) {
        gl_Position = vec4(-100.0, -100.0, -100.0, 1.0);
        return;
    }

    // 二次型系数
    float det_inv = 1.0 / det;
    conic = vec3(
        cov2d[1][1] * det_inv,
        -cov2d[0][1] * det_inv,
        cov2d[0][0] * det_inv
    );

    // 四边形大小计算 (更精确的椭圆半轴)
    float lambda1 = 0.5 * (cov2d[0][0] + cov2d[1][1] + sqrt((cov2d[0][0] - cov2d[1][1]) * (cov2d[0][0] - cov2d[1][1]) + 4.0 * cov2d[0][1] * cov2d[0][1]));
    float lambda2 = 0.5 * (cov2d[0][0] + cov2d[1][1] - sqrt((cov2d[0][0] - cov2d[1][1]) * (cov2d[0][0] - cov2d[1][1]) + 4.0 * cov2d[0][1] * cov2d[0][1]));
    
    vec2 quadwh_scr = 3.0 * vec2(sqrt(lambda1), sqrt(lambda2));
    vec2 wh = 2.0 * hfov_focal.xy * hfov_focal.z;
    vec2 quadwh_ndc = quadwh_scr / wh * 2.0;

    // 最终位置
    pos2d.xy += quadPosition * quadwh_ndc;
    gl_Position = pos2d;

    // 颜色计算
    vec3 view_dir = normalize(center - camera_position);
    
    // 球谐光照
    vec3 color;
    if (SH_COEFFS > 3) {
        color = evaluateSH(view_dir, start + SH_IDX);
    } else {
        color = get_vec3(start + SH_IDX);  // 只使用DC分量
    }

    // 颜色处理管道
    color = color * color_scale + color_offset;
    color = applySaturation(color, saturation);
    color = applyGammaCorrection(color, gamma_correction);

    // 传递给片段着色器
    coordxy = quadPosition * quadwh_scr;
    outColor = max(color, vec3(0.0));
    opacity = clamp(opacity_val, 0.0, 1.0);
    
}
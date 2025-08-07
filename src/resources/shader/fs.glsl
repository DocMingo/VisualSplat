#version 430 core

in vec3 outColor;
in float opacity;
in vec2 coordxy;
in vec3 conic;
in vec3 debugInfo;

out vec4 fragColor;

uniform bool debug_mode = true;
uniform bool show_wireframe = false;

void main() {
    // 计算椭球内部的衰减
    float A = dot(coordxy, coordxy);

    if (debug_mode) {
        // 调试模式1: 显示椭球边界
        if (show_wireframe) {
            float edge = abs(A - 1.0);
            if (edge < 0.1) {
                fragColor = vec4(1.0, 1.0, 0.0, 1.0); // 黄色边界
                return;
            }
        }

        // 调试模式2: 显示距离场
        if (A > 4.0) {
            discard; // 限制椭球大小
        }

        // 使用简单的衰减而不是高斯衰减
        float alpha = opacity * (1.0 - A * 0.25);
        alpha = max(alpha, 0.1); // 确保最小可见度

        fragColor = vec4(outColor, alpha);
    }
    else {
        // 正常模式：标准高斯衰减
        if (A > 1.0) {
            discard;
        }

        float alpha = exp(-A * 4.0) * opacity;

        if (alpha < 0.01) {
            discard;
        }

        fragColor = vec4(outColor * alpha, alpha);
    }
}
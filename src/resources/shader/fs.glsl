#version 430 core

in vec3 outColor;
in float opacity;
in vec2 coordxy;
in vec3 conic;

out vec4 fragColor;

void main() {
    // 计算椭球内部的衰减
    float A = dot(coordxy, coordxy);

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
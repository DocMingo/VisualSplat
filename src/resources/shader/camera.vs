// 顶点着色器
#version 330 core
layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec2 aTexCoord;

// out vec2 TexCoord; // 用于将数据传递给fs片段着色器 片段着色器无法直接访问顶点属性（如 aPos 和 aTexCoord），只能接收顶点着色器通过 out 显式传下来的插值结果。

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * model * vec4(aPos, 1.0f);
	// TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}
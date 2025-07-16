// 片段着色器
#version 330 core
out vec4 m_Color; 
// 片段着色器只需要一个输出变量，这个变量是一个4分量向量，它表示的是最终的输出颜色，我们应该自己将其计算出来。声明输出变量可以使用out关键字，这里我们命名为FragColor。下面，我们将一个Alpha值为1.0(1.0代表完全不透明)的橘黄色的vec4赋值给颜色输出。

in vec2 TexCoord; // 接受顶点着色器中的输出 片段着色器无法直接访问顶点属性（如 aPos 和 aTexCoord），只能接收顶点着色器通过 out 显式传下来的插值结果。

// texture samplers
uniform vec3 objectColor;
uniform vec3 lightColor;

void main()
{
	// linearly interpolate between both textures (80% container, 20% awesomeface)
	// FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0);
	// FragColor += ourColor;
	m_Color = vec4(lightColor * objectColor , 1.0);
}
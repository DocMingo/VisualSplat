// Ƭ����ɫ��
#version 330 core
out vec4 m_Color; 
// Ƭ����ɫ��ֻ��Ҫһ��������������������һ��4��������������ʾ�������յ������ɫ������Ӧ���Լ����������������������������ʹ��out�ؼ��֣�������������ΪFragColor�����棬���ǽ�һ��AlphaֵΪ1.0(1.0������ȫ��͸��)���ٻ�ɫ��vec4��ֵ����ɫ�����

in vec2 TexCoord; // ���ܶ�����ɫ���е���� Ƭ����ɫ���޷�ֱ�ӷ��ʶ������ԣ��� aPos �� aTexCoord����ֻ�ܽ��ն�����ɫ��ͨ�� out ��ʽ�������Ĳ�ֵ�����

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
// Ƭ����ɫ��
#version 330 core
out vec4 FragColor; 
// Ƭ����ɫ��ֻ��Ҫһ��������������������һ��4��������������ʾ�������յ������ɫ������Ӧ���Լ����������������������������ʹ��out�ؼ��֣�������������ΪFragColor�����棬���ǽ�һ��AlphaֵΪ1.0(1.0������ȫ��͸��)���ٻ�ɫ��vec4��ֵ����ɫ�����

in vec2 TexCoord; // ���ܶ�����ɫ���е���� Ƭ����ɫ���޷�ֱ�ӷ��ʶ������ԣ��� aPos �� aTexCoord����ֻ�ܽ��ն�����ɫ��ͨ�� out ��ʽ�������Ĳ�ֵ�����

// texture samplers
uniform sampler2D texture1;
uniform sampler2D texture2;
uniform vec4 ourColor;

void main()
{
	// linearly interpolate between both textures (80% container, 20% awesomeface)
	FragColor = mix(texture(texture1, TexCoord), texture(texture2, TexCoord), 0);
	// FragColor += ourColor;
	// FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
}
// ������ɫ��
#version 330 core
layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec2 aTexCoord;

// out vec2 TexCoord; // ���ڽ����ݴ��ݸ�fsƬ����ɫ�� Ƭ����ɫ���޷�ֱ�ӷ��ʶ������ԣ��� aPos �� aTexCoord����ֻ�ܽ��ն�����ɫ��ͨ�� out ��ʽ�������Ĳ�ֵ�����

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position = projection * view * model * vec4(aPos, 1.0f);
	// TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}
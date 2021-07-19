#version 330 core

uniform sampler2D image;
uniform vec3 textColor;

in vec2 uv;
out vec4 color;

void main()
{
	vec4 sampled = vec4(1.0,1.0,1.0,texture(image,uv).r);
	color = sampled * vec4(textColor,1.0);
}

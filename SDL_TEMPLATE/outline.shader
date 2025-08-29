#shader vertex
#version 430 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;
uniform float u_Outlining;

void main() {
	gl_Position = u_Projection * u_View * u_Model * vec4(aPos + aNormal * u_Outlining, 1.0);
}

#shader fragment
#version 430 core

struct Material {
	sampler2D texture_diffuse1;
	sampler2D texture_specular1;
	float shininess;
};

out vec4 FragColor;

uniform vec3 u_OutlineColor;
uniform Material material;

void main() {
	FragColor = vec4(u_OutlineColor, 1.0f);
}
#shader vertex
#version 430 core

layout (location = 0) in vec3 L_aPos;

out vec3 texCoords;

uniform mat4 u_Projection;
uniform mat4 u_View;

void main() {
	texCoords = L_aPos;
	gl_Position = u_Projection * u_View * vec4(L_aPos, 1.0f);
}

#shader fragment
#version 430 core
	
out vec4 FragColor;

uniform samplerCube skybox;

in vec3 texCoords;

void main() {
	FragColor = texture(skybox, texCoords);
}
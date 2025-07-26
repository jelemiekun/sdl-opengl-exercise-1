#shader vertex
#version 430 core
	
layout (location = 0) in vec3 L_coordinates;
layout (location = 1) in vec2 L_imgCoordinates;

out vec2 imgCoordinates;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

void main() {
	vec4 pos = vec4(L_coordinates, 1.0f);
	pos = u_Projection * u_View * u_Model * pos;
	gl_Position = pos;
	imgCoordinates = L_imgCoordinates;
}



#shader fragment
#version 430 core

uniform vec3 u_LightColor;

out vec4 FragColor;

void main(){
	FragColor = vec4(u_LightColor, 1.0f);
}
#shader vertex
#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 vColor;

uniform mat4 u_View;
uniform mat4 u_Projection;

void main() {
    vColor = aColor;
    gl_Position = u_Projection * u_View * vec4(aPos, 1.0);
}

#shader fragment
#version 430 core

in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
}

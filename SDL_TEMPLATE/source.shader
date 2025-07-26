#shader vertex
#version 430 core
	
layout (location = 0) in vec3 L_coordinates;
layout (location = 1) in vec3 L_normal;
layout (location = 2) in vec2 L_texCoords;

uniform mat3 u_NormalMatrix;
uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

void main() {
	vec4 pos = vec4(L_coordinates, 1.0f);
	pos = u_Projection * u_View * u_Model * pos;
	gl_Position = pos;

	Normal = u_NormalMatrix * L_normal;
	FragPos = vec3(u_Model * vec4(L_coordinates, 1.0f));

	TexCoords = L_texCoords;
}



#shader fragment
#version 430 core

 struct DirLight {
	 vec3 direction;
	 vec3 ambient;
	 vec3 diffuse;
	 vec3 specular;
 };

 struct PointLight {
	 vec3 position;
	 float constant;
	 float linear;
	 float quadratic;
	 vec3 ambient;
	 vec3 diffuse;
	 vec3 specular;
 };

struct SpotLight {
	vec3 position;
	vec3 direction;
	float innerCutoff;
	float outerCutoff;
	float constant;
	float linear;
	float quadratic;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct Material {
	sampler2D texture_diffuse1;
	sampler2D texture_specular1;
	float shininess;
};


in vec3 Normal;
in vec3 FragPos;
in vec2 TexCoords;

out vec4 FragColor;

uniform DirLight dirLight;
uniform Material material;
uniform vec3 u_CameraPos;

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main(){
	vec3 norm = normalize(Normal);
	vec3 viewDir = normalize(u_CameraPos - FragPos);
	
	vec3 result = CalcDirLight(dirLight, norm, viewDir);

	FragColor = vec4(result, 1.0f);
}

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir) {
	// Ambient Lighting
	vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
	 
	// Diffuse Lighting
	vec3 lightDir = normalize(-light.direction);
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords));
	
	// Specular Lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords));
	 
	return (ambient + diffuse + specular);
}

vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir) {
	// Attenuation
	float distance = length(light.position -fragPos);
	float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
	
	// Ambient Lighting
	vec3 ambient  = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
	
	// Diffuse Lighting
	vec3 lightDir = normalize(light.position -fragPos);
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuse  = light.diffuse  * diff * vec3(texture(material.texture_diffuse1, TexCoords)) * attenuation;
	
	// Specular Lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords)) * attenuation;
	
	return (ambient + diffuse + specular);
} 

vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir) {
	vec3 lightDir = normalize(light.position - fragPos);
	
	// Calculating Attenuation
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
	
	// Calculating Spotlight Intensity
	float theta = dot(lightDir, normalize(-light.direction));
	float epsilon = light.innerCutoff - light.outerCutoff;
	float intensity = clamp((theta - light.outerCutoff) / epsilon, 0.0, 1.0); // ensures intensity is between 0.0 and 1.0
	
	// Ambient Lighting
	vec3 ambient = light.ambient * vec3(texture(material.texture_diffuse1, TexCoords));
	
	// Diffuse Lighting
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuse = light.diffuse * diff * vec3(texture(material.texture_diffuse1, TexCoords)) * attenuation * intensity;
	
	// Specular Lighting
	vec3 reflectDir = reflect(-lightDir, normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.specular * spec * vec3(texture(material.texture_specular1, TexCoords)) * attenuation * intensity;
	
	return (ambient + diffuse + specular);
}
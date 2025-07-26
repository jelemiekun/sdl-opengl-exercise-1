#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>

#include "Shader.h"

struct Vertex {
	glm::vec3 Position;
	glm::vec3 Normal;
	glm::vec2 TexCoords;
	glm::vec3 Tangent;
	glm::vec3 Bitangent;
};

struct Texture {
	unsigned int id;
	std::string type;
	std::string path;
};

class Mesh {
public:
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	std::vector<Texture> textures;
	glm::mat4 transform;
	Mesh(std::vector<Vertex> verts, std::vector<unsigned int> inds, std::vector<Texture> texs);
	void Draw(Shader& shader, glm::mat4& model);

private:
	unsigned int vao, vbo, ebo;
	void setupMesh();
};


#pragma once

#include <glad/glad.h> 
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <map>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include "Mesh.h"
#include "Shader.h"

class Model {
public:
    std::vector<Texture> textures_loaded;
    std::vector<Mesh> meshes;
    std::string directory;
    std::string pointerName;

    Model();
    void init(std::string const& path, std::string const& r_PointerName);

private:
    void loadModel(std::string const& path);
    void processNode(aiNode* node, const aiScene* scene, const glm::mat4& parentTransform);
    Mesh processMesh(aiMesh* mesh, const aiScene* scene);
    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName, const aiScene* scene);
};
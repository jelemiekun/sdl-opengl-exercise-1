#pragma once
#include <glm/glm.hpp>
#include <memory>

class Shader;
class Model;
struct ObjectInfo;

struct ModelInstance {
    ModelInstance(Model* modelRef, bool r_Drawable = true, bool gamma = false);

    std::shared_ptr<ObjectInfo> info;
    bool drawable;

    Model* modelRef;
	glm::mat4 model = glm::mat4(1.0f);
    float scale = 1.0f;
    glm::vec3 translation = glm::vec3(0.0f);
    float radiansRotate = 0.0f;
    glm::vec3 rotateAxis = glm::vec3(0.0f, 1.0f, 0.0f);
    bool gammaCorrection;

    void updateModelMatrix();
    glm::mat3 getNormalMatrix();
    void draw(Shader& shader);
};
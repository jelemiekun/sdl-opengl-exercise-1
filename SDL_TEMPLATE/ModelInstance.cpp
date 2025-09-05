#include "ModelInstance.h"
#include "Shader.h"
#include "Model.h"

ModelInstance::ModelInstance(Model* modelRef, bool gamma) : modelRef(modelRef), gammaCorrection(gamma)  {}

void ModelInstance::updateModelMatrix() {
	model = glm::mat4(1.0f);
    model = glm::translate(model, translation);
    model = glm::scale(model, glm::vec3(scale));
    model = glm::rotate(model, glm::radians(radiansRotate), rotateAxis);
}

glm::mat3 ModelInstance::getNormalMatrix() {
	return glm::mat3();
}

void ModelInstance::draw(Shader& shader) {
    for (unsigned int i = 0; i < modelRef->meshes.size(); i++) {
        modelRef->meshes[i].Draw(shader, model);
        shader.setMat3("u_NormalMatrix", getNormalMatrix());
    }
}

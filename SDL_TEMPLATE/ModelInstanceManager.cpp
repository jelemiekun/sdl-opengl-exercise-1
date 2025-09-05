#include "ModelInstanceManager.h"
#include "ModelInstance.h"
#include "PhysicsManager.h"
#include <spdlog/spdlog.h>

std::unordered_map<std::string, std::vector<ModelInstance>> ModelInstanceManager::modelInstances;

void ModelInstanceManager::addModelType(std::string modelName) {
	spdlog::info("Adding model type to the map...");

	if (modelNameExists(modelName)) {
		spdlog::error("Failed to add. Model type {} already exists!", modelName);
		return;
	}

	modelInstances[modelName] = std::vector<ModelInstance>();
	spdlog::info("Model type {} successfully added to the map successfully.", modelName);
}

void ModelInstanceManager::addModelInstance(std::string modelName, ModelInstance& modelInstance) {
	spdlog::info("Adding instance of a model...");

	if (!modelNameExists(modelName)) {
		spdlog::error("Failed to proceed to adding instance of the model. Model {} does not exist on the map!", modelName);
		return;
	}

	modelInstances[modelName].push_back(modelInstance);
	spdlog::info("Instance of model {} added to the list successfully.", modelName);
}

void ModelInstanceManager::removeModelType(std::string modelName) {
	spdlog::info("Removing model type to the map...");

	auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to remove. Model type {} does not exist!", modelName);
        return;
    }

    spdlog::info("Removing all instances of the model type {}...", modelName);

    it->second.clear();

    spdlog::info("Successfully removed all instances of the model type. Removing model type from the map...");

    std::string refName = modelName;
    modelInstances.erase(it);

    spdlog::info("Model type {} successfully removed from the map.", refName);
}

void ModelInstanceManager::removeModelInstance(std::string modelName, ModelInstance& modelInstance) {
	spdlog::info("Removing an instance of {} from the list...", modelName);

	auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to proceed removing an instance of model type. Model type {} does not exist!", modelName);
        return;
    }

	auto& instances = it->second;

    /*instances.erase(
        std::remove(instances.begin(), instances.end(), modelInstance),
        instances.end()
    );*/

    spdlog::info("Removed an instance of the model type {}...", modelName);
}

void ModelInstanceManager::updateAllModelMatrices() {
	for (auto& modelTypes : modelInstances) {
		std::string model = modelTypes.first;
		auto& instances = modelTypes.second;

		for (auto& instance : instances) {
			PhysicsManager::updateModelMatrix(&instance, PhysicsManager::landscapeBody);
		}
	}
}

void ModelInstanceManager::drawAll(Shader& shader) {
	for (auto& modelTypes : modelInstances) {
		std::string model = modelTypes.first;
		auto& instances = modelTypes.second;

		for (auto& instance : instances) {
			instance.draw(shader);
		}
	}
}

void ModelInstanceManager::drawAll(Shader& shader, std::string modelName) {

    auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::warn("No instances found for model type {}", modelName);
        return;
    }

    auto& instances = it->second;
    for (auto& instance : instances) {
        instance.draw(shader);
    }
}


bool ModelInstanceManager::modelNameExists(std::string modelName) {
	return modelInstances.find(modelName) != modelInstances.end();
}

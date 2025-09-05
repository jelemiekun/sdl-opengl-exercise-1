#include "ModelInstanceManager.h"
#include "ModelInstance.h"
#include "PhysicsManager.h"
#include <spdlog/spdlog.h>

std::unordered_map<std::string, std::vector<std::shared_ptr<ModelInstance>>> ModelInstanceManager::modelInstances;

void ModelInstanceManager::addModelType(const std::string& modelName) {
	spdlog::info("Adding model type to the map...");

	if (modelNameExists(modelName)) {
		spdlog::error("Failed to add. Model type {} already exists!", modelName);
		return;
	}

	modelInstances[modelName] = std::vector<std::shared_ptr<ModelInstance>>();
	spdlog::info("Model type {} added to the map successfully.", modelName);
}

void ModelInstanceManager::addModelInstance(const std::string& modelName, std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Adding instance of a model...");

	if (!modelNameExists(modelName)) {
		spdlog::error("Failed to proceed to adding instance of the model. Model {} does not exist on the map!", modelName);
		return;
	}

	modelInstances[modelName].emplace_back(modelInstance);
	spdlog::info("Instance of model {} added to the list successfully.", modelName);
}

void ModelInstanceManager::removeModelType(const std::string& modelName) {
	spdlog::info("Removing model type to the map...");

	auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to remove. Model type {} does not exist!", modelName);
        return;
    }

    spdlog::info("Removing all instances of the model type {}...", modelName);

    it->second.clear();

    spdlog::info("Successfully removed all instances of the model type. Removing model type from the map...");

    modelInstances.erase(it);

    spdlog::info("Model type {} successfully removed from the map.", modelName);
}

void ModelInstanceManager::removeModelInstance(const std::string& modelName, std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Removing an instance of {} from the list...", modelName);

	int sizeBefore = 0;
	int sizeAfter = 0;

	auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to proceed removing an instance of model type. Model type {} does not exist!", modelName);
        return;
    }

	auto& instances = it->second;

	sizeBefore = instances.size();

    instances.erase(
		std::remove(instances.begin(), instances.end(), modelInstance),
		instances.end()
	);

	sizeAfter = instances.size();

	if (sizeBefore != sizeAfter)
		spdlog::info("Removed an instance of the model type {}...", modelName);
	else
		spdlog::info("Removed no instances of the model type {}...", modelName);
}

void ModelInstanceManager::updateAllModelMatrices() {
	for (auto& modelTypes : modelInstances) {
		auto& instances = modelTypes.second;

		for (auto& instance : instances) {
			PhysicsManager::updateModelMatrix(instance.get(), PhysicsManager::landscapeBody);
		}
	}
}

void ModelInstanceManager::drawAll(Shader& shader) {
	for (auto& modelTypes : modelInstances) {
		auto& instances = modelTypes.second;

		for (auto& instance : instances) {
			instance->draw(shader);
		}
	}
}

void ModelInstanceManager::drawAll(Shader& shader, const std::string& modelName) {

    auto it = modelInstances.find(modelName);
    if (it == modelInstances.end()) {
        spdlog::warn("No instances found for model type {}", modelName);
        return;
    }

    auto& instances = it->second;
    for (auto& instance : instances) {
        instance->draw(shader);
    }
}


bool ModelInstanceManager::modelNameExists(const std::string& modelName) {
	return modelInstances.find(modelName) != modelInstances.end();
}

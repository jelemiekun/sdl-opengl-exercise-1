#include "ModelInstanceManager.h"
#include "ModelInstance.h"
#include "PhysicsManager.h"
#include "ThrowableSphere.h"
#include "PhysicsConstants.h"
#include "ObjectInfo.h"
#include <spdlog/spdlog.h>

std::unordered_map<std::string, std::vector<std::shared_ptr<ModelInstance>>> ModelInstanceManager::modelInstances;

void ModelInstanceManager::addModelType(const std::string& pointerName) {
	spdlog::info("Adding model type to the map...");

	if (pointerNameExists(pointerName)) {
		spdlog::error("Failed to add. Model type {} already exists!", pointerName);
		return;
	}

	modelInstances[pointerName] = std::vector<std::shared_ptr<ModelInstance>>();
	spdlog::info("Model type {} added to the map successfully.", pointerName);
}

void ModelInstanceManager::addModelInstance(const std::string& pointerName, std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Adding instance of a model...");

	if (!pointerNameExists(pointerName)) {
		spdlog::error("Failed to proceed to adding instance of the model. Model {} does not exist on the map!", pointerName);
		return;
	}

	modelInstances[pointerName].emplace_back(modelInstance);
	spdlog::info("Instance of model {} added to the list successfully.", pointerName);
}

void ModelInstanceManager::removeModelType(const std::string& pointerName) {
	spdlog::info("Removing model type to the map...");

	auto it = modelInstances.find(pointerName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to remove. Model type {} does not exist on the map!", pointerName);
        return;
    }

    spdlog::info("Removing all instances of the model type {}...", pointerName);

    it->second.clear();

    spdlog::info("Successfully removed all instances of the model type. Removing model type from the map...");

    modelInstances.erase(it);

    spdlog::info("Model type {} successfully removed from the map.", pointerName);
}

void ModelInstanceManager::removeModelInstance(const std::string& pointerName, std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Removing an instance of {} from the list...", pointerName);

	int sizeBefore = 0;
	int sizeAfter = 0;

	auto it = modelInstances.find(pointerName);
    if (it == modelInstances.end()) {
        spdlog::error("Failed to proceed removing an instance of model type. Model type {} does not exist on the map!", pointerName);
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
		spdlog::info("Removed an instance of the model type {}...", pointerName);
	else
		spdlog::info("Removed no instances of the model type {}...", pointerName);
}

void ModelInstanceManager::updateAllModelMatrices() {
	spdlog::debug("Updating model matrices for {} model types...", modelInstances.size());

	for (auto& modelTypes : modelInstances) {
		const std::string& modelType = modelTypes.first;
		auto& instances = modelTypes.second;

		if (modelType == OBJECTS_POINTER_NAME::LANDSCAPE) {
			for (auto& instance : instances) {
				PhysicsManager::updateModelMatrix(instance.get(), PhysicsManager::landscapeBody);
			}
		} else if (modelType == OBJECTS_POINTER_NAME::THROWABLE_SPHERE) {
			for (auto& instance : instances) {
				auto it = ThrowableSphere::modelPhysicsMap.find(instance);
				if (it != ThrowableSphere::modelPhysicsMap.end()) {
					PhysicsManager::updateModelMatrix(instance.get(), it->second.body);
				}
			}
		}

		spdlog::trace("Updating matrices for model type {} ({} instances)", modelTypes.first, instances.size());
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

void ModelInstanceManager::drawAll(Shader& shader, const std::string& pointerName) {

    auto it = modelInstances.find(pointerName);
    if (it == modelInstances.end()) {
        spdlog::warn("No instances found for model type {}", pointerName);
        return;
	}

    auto& instances = it->second;

	spdlog::debug("Drawing {} instances of model type {}", instances.size(), pointerName);

    for (auto& instance : instances) {
        instance->draw(shader);
    }
}


bool ModelInstanceManager::pointerNameExists(const std::string& pointerName) {
	return modelInstances.find(pointerName) != modelInstances.end();
}

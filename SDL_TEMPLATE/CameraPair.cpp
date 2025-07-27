#include "CameraPair.h"
#include <spdlog/spdlog.h>

std::vector<std::pair<Camera*, Model*>> CameraPair::pairs;

void CameraPair::addPair(Camera* camera, Model* model) {
	pairs.emplace_back(camera, model);
}

void CameraPair::removePair(const int& index) {
	if (index >= 0 && index < static_cast<int>(pairs.size())) {
		pairs.erase(pairs.begin() + index);
	}
}

Camera* CameraPair::getCamera(const Model& model) {
	for (auto& pair : pairs) {
		if (pair.second == &model) {
			return pair.first;
		}
	}
	spdlog::error("Model not paired with any Camera");
	return nullptr;
}

Model* CameraPair::getModel(const Camera& camera) {
	for (auto& pair : pairs) {
		if (pair.first == &camera) {
			return pair.second;
		}
	}
	spdlog::error("Camera not paired with any Model");
	return nullptr;
}

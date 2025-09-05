#pragma once
#include <vector>
#include <unordered_map>
#include <string>

class ModelInstance;
class Shader;

struct ModelInstanceManager {
public:
	static std::unordered_map<std::string, std::vector<ModelInstance>> modelInstances;

	static void addModelType(std::string modelName);
	static void addModelInstance(std::string modelName, ModelInstance& modelInstance);
	static void removeModelType(std::string modelName);
	static void removeModelInstance(std::string modelName, ModelInstance& modelInstance);

	static void drawAll(Shader& shader);
	static void drawAll(Shader& shader, std::string type);

private:
	static bool modelNameExists(std::string modelName);
};


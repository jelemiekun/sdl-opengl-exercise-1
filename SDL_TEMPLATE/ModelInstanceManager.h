#pragma once
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

struct ModelInstance;
class Shader;

struct ModelInstanceManager {
public:
	static std::unordered_map<std::string, std::vector<std::shared_ptr<ModelInstance>>> modelInstances;

	static void addModelType(const std::string& pointerName);
	static void addModelInstance(const std::string& pointerName, std::shared_ptr<ModelInstance> modelInstance);
	static void removeModelType(const std::string& pointerName);
	static void removeModelInstance(const std::string& pointerName, std::shared_ptr<ModelInstance> modelInstance);

	static void updateAllModelMatrices();

	static void drawAll(Shader& shader);
	static void drawAll(Shader& shader, const std::string& type);

private:
	static bool pointerNameExists(const std::string& pointerName);
};
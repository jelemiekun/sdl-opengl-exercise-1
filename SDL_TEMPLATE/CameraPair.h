#pragma once
#include <vector>

class Camera;
class Model;

class CameraPair {
public:
    static std::vector<std::pair<Camera*, Model*>> pairs;

    static void addPair(Camera* camera, Model* model);
    static void removePair(const int& index);
    static Camera* getCamera(const Model& model);
    static Model* getModel(const Camera& camera);
};
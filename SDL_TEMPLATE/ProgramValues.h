#pragma once
#include <glm/gtc/matrix_transform.hpp>

class ImGuiWindow;
class Camera;
class Model;
class Shader;

namespace ProgramValues {
	struct DirLight {
		glm::vec3 direction;
		glm::vec3 ambient;
		glm::vec3 diffuse;
		glm::vec3 specular;
	};

	struct PointLight {
		glm::vec3 position;
		float constant;
		float linear;
		float quadratic;
		glm::vec3 ambient;
		glm::vec3 diffuse;
		glm::vec3 specular;
	};

	struct SpotLight {
		glm::vec3 position;
		glm::vec3 direction;
		float innerCutoff;
		float outerCutoff;
		float constant;
		float linear;
		float quadratic;
		glm::vec3 ambient;
		glm::vec3 diffuse;
		glm::vec3 specular;
	};

	namespace GameWindow {
		extern glm::mat4 projection;
		extern int width;
		extern int height;
		extern int FPS_LIMIT;
		extern int FPS;
		extern float deltaTime;
	}

	namespace Shaders {
		extern Shader shaderObject;
		extern Shader shaderLight;
	}

	namespace Cameras {
		// TODO: use camera reference
		extern Camera freeFly;

	}

	namespace GameObjects {
		extern Model cube;
	}

	namespace CameraKeyEvents {
		extern bool isLockedIn;
		extern bool isLockedInPressed;
		extern bool moveForwardPressed;
		extern bool moveBackwardPressed;
		extern bool moveLeftPressed;
		extern bool moveRightPressed;
		extern bool moveUpPressed;
		extern bool moveDownPressed;
		extern bool sprinting;
		extern bool fastZoom;
	}

	namespace Lights {
		extern DirLight sun;
	}
};
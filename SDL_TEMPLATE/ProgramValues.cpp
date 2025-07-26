#include <glm/gtc/matrix_transform.hpp>
#include "ProgramValues.h"
#include "Model.h"
#include "Shader.h"
#include "Camera.h"

namespace ProgramValues {
	namespace GameWindow {
		glm::mat4 projection = glm::perspective(
			glm::radians(0.0f),
			(float)1 / (float)1,
			0.0f,
			0.0f
		);

		int width = 1080;
		int height = 720;
		int FPS_LIMIT = 60;
		int FPS = 0;
		float deltaTime = 0.0f;
	}

	namespace Shaders {
		Shader shaderObject;
		Shader shaderLight;
	}

	namespace Cameras {
		Camera freeFly;
	}

	namespace GameObjects {
		Model cube;
	}

	namespace CameraKeyEvents {
		bool isLockedIn = true;
		bool isLockedInPressed = false;
		bool moveForwardPressed = false;
		bool moveBackwardPressed = false;
		bool moveLeftPressed = false;
		bool moveRightPressed = false;
		bool moveUpPressed = false;
		bool moveDownPressed = false;
		bool sprinting = false;
		bool fastZoom = false;
	}

	namespace Lights {
		DirLight sun = {
			glm::vec3(0.2f, -1.0f, 0.0f),
			glm::vec3(0.2f),
			glm::vec3(0.4f),
			glm::vec3(0.6f)
		};
	}
}
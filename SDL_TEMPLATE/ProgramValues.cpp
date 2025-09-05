#include <glm/gtc/matrix_transform.hpp>
#include "ProgramValues.h"
#include "Model.h"
#include "Shader.h"
#include "Camera.h"
#include "Texture2D.h"
#include "ModelInstance.h"

namespace ProgramValues {
	namespace Textures {
		Texture2D skybox;
	}

	namespace GameWindow {
		glm::mat4 projection = glm::perspective(
			glm::radians(0.0f),
			(float)1 / (float)1,
			0.0f,
			0.0f
		);

		int width = 720;
		int height = 480;
		int FPS_LIMIT = 60;
		int FPS = 0;
		float deltaTime = 0.0f;
		float RAMusedMB = 0;
	}

	namespace Shaders {
		Shader shaderObject;
		Shader shaderLight;
		Shader shaderSkybox;
	}

	namespace Cameras {
		Camera* cameraReference = &freeFly;
		Camera freeFly;
		Camera camera1;
	}

	namespace GameObjects {
		Model landscape;
		Model throwingBall;
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
		bool isOnJump = false;
	}

	namespace Lights {
		namespace References {
			DirLight* dirLightRef = &dr_sun;
			SpotLight* spotLightRef = &spt_pov;
		}

		DirLight dr_sun = {
			glm::vec3(0.2f, 1.0f, 0.0f),
			glm::vec3(0.2f),
			glm::vec3(0.4f),
			glm::vec3(0.6f)
		};

		SpotLight spt_pov = {
			ProgramValues::Cameras::freeFly.position,
			ProgramValues::Cameras::freeFly.front,
			glm::radians(12.5f),
			glm::radians(17.5f),
			1.0f,
			0.09f,
			0.032f,
			glm::vec3(0.0f),
			glm::vec3(1.0f),
			glm::vec3(1.0f),
		};
	}

	namespace VertexArray {
		unsigned int skyboxVAO;
	}

	namespace GameFlags {
		bool isFreeFlying = false;
		bool isFreeFlyingPressed = false;
	}

	namespace ThrowableSphereFlags {
		bool pressingDown = false;
		bool cooldownDone = true;
	}
}
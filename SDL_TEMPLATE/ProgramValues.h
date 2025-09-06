#pragma once
#include <glm/gtc/matrix_transform.hpp>

class ImGuiWindow;
class Camera;
class Model;
class Shader;
class Texture2D;

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

	namespace Textures {
		extern Texture2D skybox;
	}

	namespace GameWindow {
		extern glm::mat4 projection;
		extern int width;
		extern int height;
		extern int FPS_LIMIT;
		extern int FPS;
		extern float deltaTime;
		extern float RAMusedMB;
	}

	namespace Shaders {
		extern Shader shaderObject;
		extern Shader shaderLight;
		extern Shader shaderSkybox;
	}

	namespace Cameras {
		extern Camera* cameraReference;
		// TODO: use camera reference
		extern Camera freeFly;
		extern Camera camera1;
	}

	namespace GameObjects {
		extern Model landscape;
		extern Model throwingBall;
	}

	namespace ProxiesGameObjcts {
		extern Model PROXY_PHYSICS_PLAYER;
		extern Model PROXY_VOID_PLANE;
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
		extern bool isOnJump;
	}

	namespace Lights {
		namespace References {
			extern DirLight* dirLightRef;
			extern SpotLight* spotLightRef;
		}

		extern DirLight dr_sun;
		extern SpotLight spt_pov;
	}

	namespace VertexArray {
		extern unsigned int skyboxVAO;
	}

	namespace GameFlags {
		extern bool isFreeFlying;
		extern bool isFreeFlyingPressed;
	}

	namespace ThrowableSphereFlags {
		extern bool pressingDown;
		extern bool cooldownDone;
	}

	namespace modelsPreTransformScale {
		extern float landscape;
	}
};
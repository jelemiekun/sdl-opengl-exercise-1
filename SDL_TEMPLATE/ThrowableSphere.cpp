#include <spdlog/spdlog.h>
#include "ThrowableSphere.h"
#include "ProgramValues.h"
#include "Camera.h"
#include "ObjectInfo.h"
#include "Model.h"
#include "ModelInstance.h"
#include "ModelInstanceManager.h"
#include "PhysicsManager.h"
#include "PhysicsConstants.h"
#include <random>

PhysicsProperties::PhysicsProperties(btSphereShape* r_Shape, btRigidBody* r_Body) :
	shape(r_Shape), body(r_Body) {}

Uint32 ThrowableSphere::lastTime = SDL_GetTicks();

std::unordered_map<
		std::shared_ptr<ModelInstance>, 
		PhysicsProperties, 
		SharedPtrHash, 
		SharedPtrEqual
	> ThrowableSphere::modelPhysicsMap;

void ThrowableSphere::checkCooldownTimer() {
	if (ProgramValues::ThrowableSphereFlags::cooldownDone)
		return;

	Uint32 currentTime = SDL_GetTicks();

	ProgramValues::ThrowableSphereFlags::cooldownDone = currentTime - lastTime > COOLDOWN_TIME;
}

void ThrowableSphere::input(SDL_Event& event) {
	if (!ProgramValues::CameraKeyEvents::isLockedIn)
		return;

	if (event.type == SDL_MOUSEBUTTONDOWN) {
		ProgramValues::ThrowableSphereFlags::pressingDown = true;
	} else if (event.type == SDL_MOUSEBUTTONUP) {
		ProgramValues::ThrowableSphereFlags::pressingDown = false;
	}
}

void ThrowableSphere::update() {
	checkCooldownTimer();
	if (isReadyToThrow()) {
		createThrowableSphere();
		updateCooldownTimerAndFlag();
	}
}

bool ThrowableSphere::isReadyToThrow() {
	return ProgramValues::ThrowableSphereFlags::pressingDown 
		&& ProgramValues::ThrowableSphereFlags::cooldownDone;
}

void ThrowableSphere::createThrowableSphere() {
	spdlog::info("Creating throwable sphere...");

	auto sphere = std::make_shared<ModelInstance>(&ProgramValues::GameObjects::throwingBall);
	sphere->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::THROWABLE_SPHERE);

	addToModelPhysicsMap(sphere);
	addToModelTypeList(sphere);

	spdlog::info("Throwable sphere created successfully.");
}

void ThrowableSphere::updateCooldownTimerAndFlag() {
	ProgramValues::ThrowableSphereFlags::cooldownDone = false;
	lastTime = SDL_GetTicks();
}

void ThrowableSphere::addToModelTypeList(std::shared_ptr<ModelInstance> sphere) {
    ModelInstanceManager::addModelInstance(ProgramValues::GameObjects::throwingBall.modelName, sphere);
}

void ThrowableSphere::addToModelPhysicsMap(std::shared_ptr<ModelInstance> sphere) {
    modelPhysicsMap.emplace(sphere, generatePhysicsProperties(sphere));
}

PhysicsProperties ThrowableSphere::generatePhysicsProperties(std::shared_ptr<ModelInstance> sphere) {
	float radius = generateRandomRadius();

	sphere->scale = radius;
	sphere->updateModelMatrix();

	btSphereShape* shape = new btSphereShape(btScalar(radius));
	
	btScalar mass = 1.0f;
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);

	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(
		btQuaternion(0, 0, 0, 1),
		btVector3(
			ProgramValues::Cameras::cameraReference->position.x,
			ProgramValues::Cameras::cameraReference->position.y,
			ProgramValues::Cameras::cameraReference->position.z
		)
	));

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
	btRigidBody* rigidBody = new btRigidBody(rbInfo);

	PhysicsManager::getWorld()->addRigidBody(rigidBody,
			COLLISION_CATEGORIES::OBJECTS,
			COLLISION_CATEGORIES::ENVIRONMENT |
			COLLISION_CATEGORIES::VOID_PLANE |
			COLLISION_CATEGORIES::OBJECTS
		);

	rigidBody->setUserPointer(sphere.get());
	manipulateRigidBody(*rigidBody);

	PhysicsProperties physicsProperties(shape, rigidBody);

	return physicsProperties;
}

float ThrowableSphere::generateRandomRadius() {
    static std::random_device dev;
    static std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(MIN_RADIUS, MAX_RADIUS);
    return dist(rng);
}

void ThrowableSphere::manipulateRigidBody(btRigidBody& body) {
    Camera* camera = ProgramValues::Cameras::cameraReference;
    if (!camera) {
        spdlog::error("Camera reference is null! Cannot shoot sphere.");
        return;
    }

    glm::vec3 front = glm::normalize(camera->front);

    btVector3 impulse(front.x, front.y, front.z);

    float throwStrength = 25.0f;
    impulse *= throwStrength;

    body.applyCentralImpulse(impulse);

    spdlog::info("Applied impulse to throwable sphere: {}, {}, {}", impulse.x(), impulse.y(), impulse.z());
}


void ThrowableSphere::removeInstance(std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Removing instance of throwable sphere...");

	removeInstanceToModelPhysicsMap(modelInstance);
	removeInstanceToModelTypeList(modelInstance);

	spdlog::info("Removed instance successfully.");
}

void ThrowableSphere::removeInstanceToModelTypeList(std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Removing instance of throwable sphere in the model type map...");

	std::string modelName = ProgramValues::GameObjects::throwingBall.modelName;

	auto it = ModelInstanceManager::modelInstances.find(modelName);

	if (it == ModelInstanceManager::modelInstances.end()) {
		spdlog::error("Failed to proceed to removing instance of the model to the model type map. Model {} does not exist on the map!", modelName);
		return;
	}

	auto& vec = it->second;
	vec.erase(std::remove_if(vec.begin(), vec.end(),
		[&](const std::shared_ptr<ModelInstance>& inst) {
			return inst == modelInstance;
		}),
	vec.end());


	spdlog::info("Removed instance in the model type map successfully.");
}

void ThrowableSphere::removeInstanceToModelPhysicsMap(std::shared_ptr<ModelInstance> modelInstance) {
	spdlog::info("Removing instance of throwable sphere in the model physics map...");

	auto it = modelPhysicsMap.find(modelInstance);
    if (it == modelPhysicsMap.end()) {
        spdlog::error("ModelInstance not found in physics map!");
        return;
    }

	auto& physicsProperties = it->second;
	deleteModelInstancePhysicalProperties(physicsProperties);

	modelPhysicsMap.erase(it);

	spdlog::info("Removed instance of throwable sphere in the model physics map successfully.");
}

void ThrowableSphere::deleteModelInstancePhysicalProperties(PhysicsProperties& physicsProperties) {
	PhysicsManager::getWorld()->removeRigidBody(physicsProperties.body);
	delete physicsProperties.body->getMotionState();
	delete physicsProperties.body;
	delete physicsProperties.shape;
}

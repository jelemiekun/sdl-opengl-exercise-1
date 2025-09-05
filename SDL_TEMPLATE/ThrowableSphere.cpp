#include <spdlog/spdlog.h>
#include "ThrowableSphere.h"
#include "ProgramValues.h"
#include "Camera.h"
#include "Model.h"
#include "ModelInstance.h"
#include "ModelInstanceManager.h"
#include "PhysicsManager.h"
#include <random>

PhysicsProperties::PhysicsProperties(btSphereShape* r_Shape, btRigidBody* r_Body) :
	shape(r_Shape), body(r_Body) {}

Uint32 ThrowableSphere::lastTime = SDL_GetTicks();

std::unordered_map<ModelInstance*, PhysicsProperties> ThrowableSphere::modelPhysicsMap;

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
	if (isReadyToThrow()) createThrowableSphere();
}

bool ThrowableSphere::isReadyToThrow() {
	return ProgramValues::ThrowableSphereFlags::pressingDown 
		&& ProgramValues::ThrowableSphereFlags::cooldownDone;
}

void ThrowableSphere::createThrowableSphere() {
	spdlog::info("Creating throwable sphere...");

	ModelInstance sphere(&ProgramValues::GameObjects::throwingBall);
	addToModelTypeList(&sphere);
	addToModelPhysicsMap(&sphere);

	spdlog::info("Throwable sphere created successfully.");
}

void ThrowableSphere::addToModelTypeList(ModelInstance* sphere) {
	ModelInstanceManager::addModelInstance(ProgramValues::GameObjects::throwingBall.modelName, sphere);
}

void ThrowableSphere::addToModelPhysicsMap(ModelInstance* sphere) {
	modelPhysicsMap.insert({ sphere, generatePhysicsProperties() });
}

PhysicsProperties ThrowableSphere::generatePhysicsProperties() {
	btSphereShape* shape = new btSphereShape(btScalar(generateRandomRadius()));
	
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

	PhysicsManager::getWorld()->addRigidBody(rigidBody);
	manipulateRigidBody(rigidBody);

	PhysicsProperties physicsProperties(shape, rigidBody);

	return physicsProperties;
}

float ThrowableSphere::generateRandomRadius() {
	std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<> dist6(1, 6);
	std::uniform_real_distribution<std::mt19937::result_type> dist6(MIN_RADIUS, MAX_RADIUS);
	return dist6(rng);
}

void ThrowableSphere::manipulateRigidBody(btRigidBody* body) {
	// TODO impulse
}

void ThrowableSphere::removeInstance() {
}

void ThrowableSphere::removeInstanceToModelTypeList() {

}

void ThrowableSphere::removeInstanceToModelPhysicsMap() {

}

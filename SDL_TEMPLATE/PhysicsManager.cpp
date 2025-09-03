#include <spdlog/spdlog.h>
#include "PhysicsManager.h"
#include "ProgramValues.h"
#include "Model.h"
#include "Mesh.h"
#include "DebugDrawer.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

btDiscreteDynamicsWorld* PhysicsManager::dynamicsWorld = nullptr;

DebugDrawer* PhysicsManager::debugDrawer = nullptr;

btVector3 PhysicsManager::gravity = btVector3(0, -9.8f, 0);

void PhysicsManager::init() {
	initPhysicsWorld();
	initDebugger();
	initCollisionShapes();
	initRigidBodies();
}

void PhysicsManager::update(const float deltaTime) {
	dynamicsWorld->stepSimulation(deltaTime, 10);
}

void PhysicsManager::updateModelMatrix(Model* model, btRigidBody* body) {
    btTransform transRef = PhysicsManager::getTrans(body);

    // Position
    btVector3 pos = transRef.getOrigin();
    glm::vec3 cubePos(pos.x(), pos.y(), pos.z());

    // Rotation
    btQuaternion rot = transRef.getRotation();
    glm::quat glmQuat(rot.w(), rot.x(), rot.y(), rot.z()); // Bullet: (x,y,z,w), GLM: (w,x,y,z)

    // Build T * R * S
    model->model =
        glm::translate(glm::mat4(1.0f), cubePos) *
        glm::mat4_cast(glmQuat) *
        glm::scale(glm::mat4(1.0f), glm::vec3(model->scale));
}

void PhysicsManager::updateExperiment() {

}

btDiscreteDynamicsWorld* PhysicsManager::getWorld() {
	return dynamicsWorld;
}

btTransform PhysicsManager::getTrans(btRigidBody* RB) {
	btTransform trans;

	if (RB) {
		RB->getMotionState()->getWorldTransform(trans);
	} else {
		spdlog::error("Rigid body {} does not exist.", static_cast<void*>(RB));
	}

	return trans;
}

DebugDrawer* PhysicsManager::getDebugDrawer() {
	return debugDrawer;
}

void PhysicsManager::initPhysicsWorld() {
	spdlog::info("Initializing physics world...");

	// Bullet physics world setup
	btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

	if (dynamicsWorld) {
		spdlog::info("Initialized physics world successfully.");
	} else {
		spdlog::error("Failed to initialize physics world.");
	}

	// Set gravity
	dynamicsWorld->setGravity(gravity);
	spdlog::info("Initialized gravity with {}, {}, {}", gravity.x(), gravity.y(), gravity.z());
}

void PhysicsManager::initDebugger() {
	debugDrawer = new DebugDrawer;
	dynamicsWorld->setDebugDrawer(debugDrawer);
}
void PhysicsManager::initCollisionShapes() {
	spdlog::info("Initializing collision shapes...");
	
	spdlog::info("Initialized collision shapes successfully.");
}

void PhysicsManager::initRigidBodies() {
	spdlog::info("Initializing rigid bodies...");


	spdlog::info("Initialized rigid bodies successfully.");
}

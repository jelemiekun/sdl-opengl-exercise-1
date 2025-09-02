#include <spdlog/spdlog.h>
#include "PhysicsManager.h"
#include "Model.h"
#include "DebugDrawer.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

btDiscreteDynamicsWorld* PhysicsManager::dynamicsWorld = nullptr;

DebugDrawer* PhysicsManager::debugDrawer = nullptr;

btCollisionShape* PhysicsManager::cubeShape = nullptr;
btCollisionShape* PhysicsManager::planeShape = nullptr;

btRigidBody* PhysicsManager::cubeBody = nullptr;
btRigidBody* PhysicsManager::planeBody = nullptr;

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
	
	cubeShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
	planeShape = new btStaticPlaneShape(btVector3(0.0f, 1.0f, 0.0f), 0);

	spdlog::info("Initialized collision shapes successfully.");
}

void PhysicsManager::initRigidBodies() {
	spdlog::info("Initializing rigid bodies...");

	{
		btScalar mass = 1.0f;
		btVector3 inertia(0, 0, 0);

		cubeShape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState* cubeMotion = new btDefaultMotionState(btTransform(
			btQuaternion(0, 0, 0, 1), btVector3(0, 40, 0)
		));

		btRigidBody::btRigidBodyConstructionInfo cubeRBInfo(mass, cubeMotion, cubeShape, inertia);
		cubeBody = new btRigidBody(cubeRBInfo);
		cubeBody->setRestitution(0.5f);
		cubeBody->applyCentralImpulse(btVector3(10.0f, 0.0f, 0.0f));
		cubeBody->applyTorqueImpulse(btVector3(10.0f, 0.0f, 0.0f));
		cubeBody->setFriction(4.5f);

		dynamicsWorld->addRigidBody(cubeBody);
	}

	{
		btDefaultMotionState* planeMotion = new btDefaultMotionState(btTransform(
			btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)
		));

		btRigidBody::btRigidBodyConstructionInfo planeRBInfo(0, planeMotion, planeShape, btVector3(0, 0, 0));
		planeBody = new btRigidBody(planeRBInfo);
		planeBody->setRestitution(1.0f);
		planeBody->setFriction(2.0f);

		dynamicsWorld->addRigidBody(planeBody);
	}

	spdlog::info("Initialized rigid bodies successfully.");
}

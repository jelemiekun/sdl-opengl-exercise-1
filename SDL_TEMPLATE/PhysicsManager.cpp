#include <spdlog/spdlog.h>
#include "ProgramValues.h"
#include "PhysicsManager.h"
#include "Model.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

btDiscreteDynamicsWorld* PhysicsManager::dynamicsWorld = nullptr;

btCollisionShape* PhysicsManager::cubeShape = nullptr;
btCollisionShape* PhysicsManager::planeShape = nullptr;

btRigidBody* PhysicsManager::cubeBody = nullptr;
btRigidBody* PhysicsManager::planeBody = nullptr;


btVector3 PhysicsManager::gravity = btVector3(0, -9.8f, 0);

void PhysicsManager::init() {
	initPhysicsWorld();
	initCollisionShapes();
	initRigidBodies();
}

void PhysicsManager::update(const float deltaTime) {
	dynamicsWorld->stepSimulation(deltaTime, 10);
}

void PhysicsManager::updateModelMatrix(Model* model, btRigidBody* body) {
	btTransform transRef = PhysicsManager::getTrans(body);

	// Update position
    btVector3 pos = transRef.getOrigin();
    glm::vec3 cubePos(pos.x(), pos.y(), pos.z());
    model->translation = cubePos;

	// Update rotation
    btQuaternion rot = transRef.getRotation();
    glm::quat glmQuat(rot.w(), rot.x(), rot.y(), rot.z());

    // Extract axis + angle from quaternion
    float angle = 2.0f * acos(glmQuat.w);
    float s = sqrt(1.0f - glmQuat.w * glmQuat.w);

    glm::vec3 axis;
    if (s < 0.0001f) {
        // If quaternion is close to identity, choose default axis
        axis = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
        axis = glm::normalize(glm::vec3(glmQuat.x, glmQuat.y, glmQuat.z) / s);
    }

    model->radiansRotate = angle;
    model->rotateAxis = axis;

    // Build model matrix
    model->model =
        glm::translate(glm::mat4(1.0f), model->translation) *
        glm::mat4_cast(glmQuat) *
        glm::scale(glm::mat4(1.0f), glm::vec3(model->scale));

    model->updateModelMatrix();
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

		dynamicsWorld->addRigidBody(cubeBody);
	}

	{
		btDefaultMotionState* planeMotion = new btDefaultMotionState(btTransform(
			btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)
		));

		btRigidBody::btRigidBodyConstructionInfo planeRBInfo(0, planeMotion, planeShape, btVector3(0, 0, 0));
		planeBody = new btRigidBody(planeRBInfo);

		dynamicsWorld->addRigidBody(planeBody);
	}

	spdlog::info("Initialized rigid bodies successfully.");
}

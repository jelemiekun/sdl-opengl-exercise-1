#include <spdlog/spdlog.h>
#include "PhysicsManager.h"

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

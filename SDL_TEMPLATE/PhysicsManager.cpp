#include <spdlog/spdlog.h>
#include "PhysicsManager.h"
#include "ProgramValues.h"
#include "Model.h"
#include "Mesh.h"
#include "Camera.h"
#include "DebugDrawer.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

btDiscreteDynamicsWorld* PhysicsManager::dynamicsWorld = nullptr;

DebugDrawer* PhysicsManager::debugDrawer = nullptr;

btBoxShape* PhysicsManager::playerShape = nullptr;
btBvhTriangleMeshShape* PhysicsManager::landscapeShape = nullptr;

btRigidBody* PhysicsManager::playerGhostBody = nullptr;
btRigidBody* PhysicsManager::landscapeBody = nullptr;

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

void PhysicsManager::updateCamera() {
	btTransform transRef = PhysicsManager::getTrans(playerGhostBody);

	spdlog::info("IsFlying: {}", ProgramValues::GameFlags::isFreeFlying);
	if (ProgramValues::GameFlags::isFreeFlying) {
		playerGhostBody->setGravity(btVector3(0, 0, 0));

		
	} else {
		playerGhostBody->setGravity(gravity);


	}

	Camera* camera = &ProgramValues::Cameras::freeFly;
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(btVector3(
			camera->position.x,
			camera->position.y,
			camera->position.z
		));

		playerGhostBody->proceedToTransform(trans);
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

	{
		btTriangleMesh* triangleMesh = new btTriangleMesh;

		Model* model = &ProgramValues::GameObjects::landscape;
		for (auto& mesh : model->meshes) {
			for (size_t i = 0; i < mesh.indices.size(); i += 3) {
				glm::vec4 v0 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i]].Position, 1.0f);
				glm::vec4 v1 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i + 1]].Position, 1.0f);
				glm::vec4 v2 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i + 2]].Position, 1.0f);

				triangleMesh->addTriangle(
					btVector3(v0.x, v0.y, v0.z),
					btVector3(v1.x, v1.y, v1.z),
					btVector3(v2.x, v2.y, v2.z)
				);
			}
		}

		landscapeShape = new btBvhTriangleMeshShape(triangleMesh, true);

		float scale = 30.0f;

		ProgramValues::GameObjects::landscape.scale = scale;
		landscapeShape->setLocalScaling(btVector3(scale, scale, scale));
	}

	{
		playerShape = new btBoxShape(btVector3(0.4f, 0.8f, 0.4f));
	}
	
	spdlog::info("Initialized collision shapes successfully.");
}

void PhysicsManager::initRigidBodies() {
	spdlog::info("Initializing rigid bodies...");

	{
		btScalar mass(0.0f);
		btVector3 inertia(0, 0, 0);

		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());
		btRigidBody::btRigidBodyConstructionInfo RBInfo(mass, motionState, landscapeShape, inertia);

		landscapeBody = new btRigidBody(RBInfo);

		dynamicsWorld->addRigidBody(landscapeBody, 
			COLLISION_CATEGORIES::ENVIRONMENT,
			COLLISION_CATEGORIES::PLAYER | COLLISION_CATEGORIES::OBJECTS);
	}

	{
		btScalar mass = 60.0f;
		btVector3 inertia(0, 0, 0);
		playerShape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState* playerMotion = new btDefaultMotionState(btTransform(
			btQuaternion(0,0,0,1), btVector3(0, 10.0f, 0)
		));

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, playerMotion, playerShape, inertia);

		playerGhostBody = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(playerGhostBody,
			COLLISION_CATEGORIES::PLAYER,
			COLLISION_CATEGORIES::ENVIRONMENT
			);
	}

	spdlog::info("Initialized rigid bodies successfully.");
}

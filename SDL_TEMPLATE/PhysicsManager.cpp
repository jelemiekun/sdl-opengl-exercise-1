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

btCollisionShape* PhysicsManager::cubeShape = nullptr;
btConvexHullShape* PhysicsManager::convexShape = nullptr;
btCompoundShape* PhysicsManager::landscapeShape = nullptr;
btBvhTriangleMeshShape* PhysicsManager::landscapeConcaveShape = nullptr;

btRigidBody* PhysicsManager::cubeBody = nullptr;
btRigidBody* PhysicsManager::triangleBody = nullptr;
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

void PhysicsManager::updateExperiment() {
	static bool hasChanged = false;
	cubeBody->applyTorque(btVector3(4,8,12));
	triangleBody->applyTorque(btVector3(9, 18, 27));
	if (!hasChanged) {
		ProgramValues::GameObjects::triangle.scale = 5.0f;
		convexShape->setLocalScaling(btVector3(5.0f, 5.0f, 5.0f));

		btScalar mass(1.0f);
		btVector3 inertia(0, 0, 0);
		convexShape->calculateLocalInertia(mass, inertia);
		
		triangleBody->setMassProps(mass, inertia);
		triangleBody->updateInertiaTensor();

		hasChanged = true;
	}
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
	convexShape = new btConvexHullShape;

	{
		Model* triangle = &ProgramValues::GameObjects::triangle;

		for (auto& mesh : triangle->meshes) {
			for (auto& v : mesh.vertices) {
				convexShape->addPoint(btVector3(v.Position.x, v.Position.y, v.Position.z));
			}
		}

	}

	landscapeShape = new btCompoundShape;

	{
		btTriangleMesh* triangleMesh = new btTriangleMesh;

		Model* triangle = &ProgramValues::GameObjects::landscape;
		for (auto& mesh : triangle->meshes) {
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

		landscapeConcaveShape = new btBvhTriangleMeshShape(triangleMesh, true);
	}

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
		cubeBody->applyCentralImpulse(btVector3(8.0f, 0.0f, 0.0f));
		cubeBody->setFriction(4.5f);

		dynamicsWorld->addRigidBody(cubeBody, 
			COLLISION_CATEGORIES::CUBE, 
			COLLISION_CATEGORIES::PLANE |
			COLLISION_CATEGORIES::TRIANGLE);
	}

	{
		btScalar mass = 0.0f;
		btVector3 inertia(0.0f, 0.0f, 0.0f);

		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, landscapeConcaveShape, inertia);
		landscapeBody = new btRigidBody(rbInfo);

		// --- 4. Add rigid body to the world ---
		dynamicsWorld->addRigidBody(landscapeBody,
			COLLISION_CATEGORIES::PLANE, 
			COLLISION_CATEGORIES::TRIANGLE |
			COLLISION_CATEGORIES::CUBE);
	}

	{
		btScalar mass = 1.0f;
		btVector3 localInertia(0,0,0);
		convexShape->calculateLocalInertia(mass, localInertia);

		btDefaultMotionState* motionState = new btDefaultMotionState(
			btTransform(btQuaternion(0,0,0,1), btVector3(0,20,0))
		);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, convexShape, localInertia);
		triangleBody = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(triangleBody,
			COLLISION_CATEGORIES::TRIANGLE, 
			COLLISION_CATEGORIES::PLANE |
			COLLISION_CATEGORIES::CUBE);

	}

	spdlog::info("Initialized rigid bodies successfully.");
}

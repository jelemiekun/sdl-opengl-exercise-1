#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Model;
class DebugDrawer;

namespace COLLISION_CATEGORIES {
	const short CUBE = 1 << 0;
	const short TRIANGLE = 1 << 0;
	const short PLANE = 1 << 0;
};

class PhysicsManager {
public:
	static void init();
	static void update(const float deltaTime);
	static void updateModelMatrix(Model* model, btRigidBody* body);
	static void updateExperiment();

	static inline btVector3 getGravity() { return gravity; }
	static btDiscreteDynamicsWorld* getWorld();
	static btTransform getTrans(btRigidBody* RB);
	static DebugDrawer* getDebugDrawer();

private:
	static btVector3 gravity;
	static btDiscreteDynamicsWorld* dynamicsWorld;
	static DebugDrawer* debugDrawer;

	static void initPhysicsWorld();
	static void initDebugger();
	static void initCollisionShapes();
	static void initRigidBodies();

private:
	static btCollisionShape* cubeShape;
	static btConvexHullShape* convexShape;
	static btCompoundShape* landscapeShape;
	static btBvhTriangleMeshShape* landscapeConcaveShape;
	static btConvexShape* militaryBackpackShape;

public:
	static btRigidBody* cubeBody;
	static btRigidBody* landscapeBody;
	static btRigidBody* triangleBody;
};


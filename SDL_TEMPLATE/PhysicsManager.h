#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Model;
class DebugDrawer;

namespace COLLISION_CATEGORIES {
	const short ENVIRONMENT		= 1 << 0;
	const short PLAYER			= 1 << 1;
	const short OBJECTS			= 1 << 2;
	const short VOID_PLANE		= 1 << 3;
};

class PhysicsManager {
public:
	static void init();
	static void update(const float deltaTime);
	static void updateModelMatrix(Model* model, btRigidBody* body);

	static inline btVector3 getGravity() { return gravity; }
	static btDiscreteDynamicsWorld* getWorld();
	static btTransform getTrans(btRigidBody* RB);
	static DebugDrawer* getDebugDrawer();

private:
	static btVector3 gravity;
	static btVector3 playerStartingPosition;
	static btDiscreteDynamicsWorld* dynamicsWorld;
	static DebugDrawer* debugDrawer;

	static void initPhysicsWorld();
	static void initDebugger();
	static void initCollisionShapes();
	static void initRigidBodies();

	static void updateCamera();
	static void updateExperiment();
	static void updateCollisions();

private:
	static btCapsuleShape* playerShape;
	static btBvhTriangleMeshShape* landscapeShape;
	static btStaticPlaneShape* voidPlaneShape;

public:
	static btRigidBody* landscapeBody;
	static btRigidBody* playerGhostBody;
	static btRigidBody* voidPlaneBody;
};


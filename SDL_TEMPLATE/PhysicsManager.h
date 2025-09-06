#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Model;
class DebugDrawer;
struct ModelInstance;

class PhysicsManager {
public:
	static void init();
	static void update(const float deltaTime);
	static void updateModelMatrix(ModelInstance* model, btRigidBody* body);

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
	static void updateCollidedObjects(btCollisionObject* obj0, btCollisionObject* obj1, const bool& hasCollision);

private:
	static btCapsuleShape* playerShape;
	static btBvhTriangleMeshShape* landscapeShape;
	static btStaticPlaneShape* voidPlaneShape;

public:
	static btRigidBody* landscapeBody;
	static btRigidBody* playerGhostBody;
	static btRigidBody* voidPlaneBody;
};


#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Model;
class DebugDrawer;

namespace COLLISION_CATEGORIES {

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

public:
};


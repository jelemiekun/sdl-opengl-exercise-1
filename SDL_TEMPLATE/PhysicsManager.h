#pragma once
#include <bullet/btBulletDynamicsCommon.h>

class Model;

class PhysicsManager {
public:
	static void init();
	static void update(const float deltaTime);
	static void updateModelMatrix(Model* model, btRigidBody* body);

	static inline btVector3 getGravity() { return gravity; }
	static btTransform getTrans(btRigidBody* RB);

private:
	static btVector3 gravity;
	static btDiscreteDynamicsWorld* dynamicsWorld;

	static void initPhysicsWorld();
	static void initCollisionShapes();
	static void initRigidBodies();

private:
	static btCollisionShape* cubeShape;
	static btCollisionShape* planeShape;

public:
	static btRigidBody* cubeBody;
	static btRigidBody* planeBody;
};


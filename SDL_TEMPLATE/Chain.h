#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <map>
#include <memory>
#include "ModelInstance.h"

class Chain {
private:
	static constexpr btScalar SHAPE_RADIUS = 15.0f;
	static constexpr btScalar SHAPE_HEIGHT = 20.0f;
	static constexpr btScalar MASS = 10.0f;
	static constexpr btScalar V_INERTIA = 10.0f;
	static btVector3 INERTIA;

public:
	void init(btDiscreteDynamicsWorld* dynamicsWorld, const int& chainCount, btRigidBody* chainedBody, const btVector3& anchorPoint);

public:
	Chain(int chainNumber);
	~Chain();

public:
	int m_chainNumber;

	btCapsuleShape* m_shape;
	btRigidBody* m_rigidBody;
	std::shared_ptr<ModelInstance> m_model;
	btPoint2PointConstraint* m_p2pConstraint;

public:
	void initShapeAndBody(btDiscreteDynamicsWorld* dynamicsWorld, const int& totalChainSize);
	void initModel();
	void addModelInstanceToTypeList();
	void initConstraintToOuterPivot(btRigidBody* chainedBody, const btVector3& anchotPoanchorPointint);
	void initConstraintToOtherChain(btRigidBody* lastRigidBody);
	void addConstraintToWorld(btDiscreteDynamicsWorld* dynamicsWorld);
	void incrementCounter();
};
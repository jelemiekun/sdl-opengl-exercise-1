#pragma once
#include <vector>
#include <memory>
#include <bullet/btBulletDynamicsCommon.h>
#include "Chain.h"

class ChainSet {
public:
	static std::vector<std::unique_ptr<ChainSet>> chainSets;

private:
	int chainCounter;

public:
	std::vector<std::unique_ptr<Chain>> chainData;

	ChainSet();

	void init(btDiscreteDynamicsWorld* dynamicsWorld, const int& chainCount, btRigidBody* chainedBody, const btVector3& anchorPoint);
	void update();
	
	void incrementCounter();
};
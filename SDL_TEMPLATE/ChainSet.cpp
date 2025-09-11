#include <spdlog/spdlog.h>
#include "ChainSet.h"
#include "Chain.h"

std::vector<std::unique_ptr<ChainSet>> ChainSet::chainSets;

ChainSet::ChainSet() : chainCounter(0) {}

void ChainSet::init(btDiscreteDynamicsWorld* dynamicsWorld, const int& chainCount, btRigidBody* chainedBody, const btVector3& anchorPoint) {
	spdlog::info("[ChainSystem] Initializing chain with {} links", chainCount);

    btRigidBody* lastRigidBody = nullptr;

    for (int index = 0; index < chainCount; index++) {
        auto chain = std::make_unique<Chain>(chainCounter);

        chain->initShapeAndBody(dynamicsWorld, chainCounter);
        chain->initModel();
        chain->addModelInstanceToTypeList();

        auto* currentRigidBody = chain->m_rigidBody;

        if (index == 0) {
            chain->initConstraintToOuterPivot(chainedBody, anchorPoint);
        } else {
            chain->initConstraintToOtherChain(lastRigidBody);
        }

        chain->addConstraintToWorld(dynamicsWorld);

        lastRigidBody = currentRigidBody;

        chainData.push_back(std::move(chain));
        incrementCounter();
    }
}

void ChainSet::incrementCounter() {
    int oldValue = chainCounter;
    chainCounter++;
    spdlog::trace("[ChainSystem] Incremented counter {} -> {}", oldValue, chainCounter);
}
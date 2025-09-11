#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <spdlog/spdlog.h>
#include "Chain.h"
#include "Model.h"
#include "ProgramValues.h"
#include "ObjectInfo.h"
#include "PhysicsConstants.h"
#include "ModelInstanceManager.h"

btVector3 Chain::INERTIA(V_INERTIA, V_INERTIA, V_INERTIA);

Chain::Chain(int chainNumber) :
    m_chainNumber(chainNumber),
    m_shape(nullptr),
    m_rigidBody(nullptr),
    m_p2pConstraint(nullptr) {
    spdlog::debug("[ChainSystem] Constructed Chain link {}", m_chainNumber);
}

Chain::~Chain() {
    spdlog::debug("[ChainSystem] Destroying Chain link {}", m_chainNumber);

    if (m_rigidBody && m_rigidBody->getMotionState())
        delete m_rigidBody->getMotionState();

    delete m_rigidBody;
    delete m_shape;
    delete m_p2pConstraint;
}

void Chain::initShapeAndBody(btDiscreteDynamicsWorld* dynamicsWorld, const int& index) {
    btVector3 origin(0, 15.0f, 0);
    origin.setY(origin.y() - (index * SHAPE_HEIGHT));

    m_shape = new btCapsuleShape(SHAPE_RADIUS, SHAPE_HEIGHT);
    m_shape->calculateLocalInertia(MASS, INERTIA);

    btDefaultMotionState* motion = new btDefaultMotionState(btTransform(
        btQuaternion(0, 0, 0, 1), origin
    ));

    btRigidBody::btRigidBodyConstructionInfo rbInfo(MASS, motion, m_shape, INERTIA);
    m_rigidBody = new btRigidBody(rbInfo);

    dynamicsWorld->addRigidBody(m_rigidBody,
        COLLISION_CATEGORIES::OBJECTS,
		COLLISION_CATEGORIES::ENVIRONMENT |
		COLLISION_CATEGORIES::VOID_PLANE |
		COLLISION_CATEGORIES::OBJECTS
        );

    spdlog::debug("[ChainSystem] Initialized rigid body for link {} at position ({}, {}, {})",
                  m_chainNumber, origin.x(), origin.y(), origin.z());
}

void Chain::initModel() {
    m_model = std::make_shared<ModelInstance>(&ProgramValues::GameObjects::singleChain);
    m_model->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::SINGLE_CHAIN);
    spdlog::debug("[ChainSystem] Model instance assigned to link {}", m_chainNumber);

    m_rigidBody->setUserPointer(m_model.get());

    btCapsuleShape* capsule = static_cast<btCapsuleShape*>(m_shape);
    float height = capsule->getHalfHeight() * 2.0f;
    float radius = capsule->getRadius();
    m_model->scale = height;
    m_model->updateModelMatrix();
}

void Chain::addModelInstanceToTypeList() {
    ModelInstanceManager::addModelInstance(ProgramValues::GameObjects::singleChain.pointerName, m_model);
    spdlog::debug("[ChainSystem] Model instance of link {} registered to type list", m_chainNumber);
}

void Chain::initConstraintToOuterPivot(btRigidBody* chainedBody, const btVector3& anchorPoint) {
    btVector3 pivotInChain(0.0f, SHAPE_HEIGHT * 0.5f, 0.0f);

    if (chainedBody) {
        m_p2pConstraint = new btPoint2PointConstraint(
            *chainedBody, *m_rigidBody, anchorPoint, pivotInChain
        );
        spdlog::debug("[ChainSystem] Constraint created between link {} and chained body at anchor ({}, {}, {})",
                      m_chainNumber, anchorPoint.x(), anchorPoint.y(), anchorPoint.z());
    }
}

void Chain::initConstraintToOtherChain(btRigidBody* lastRigidBody) {
    btVector3 pivotInPrev(0.0f, -SHAPE_HEIGHT * 0.5f, 0.0f);
    btVector3 pivotInChain(0.0f, SHAPE_HEIGHT * 0.5f, 0.0f);

    m_p2pConstraint = new btPoint2PointConstraint(
        *lastRigidBody, *m_rigidBody, pivotInPrev, pivotInChain
    );

    spdlog::debug("[ChainSystem] Constraint created between link {} and previous link {}",
                  m_chainNumber, m_chainNumber - 1);
}

void Chain::addConstraintToWorld(btDiscreteDynamicsWorld* dynamicsWorld) {
    m_p2pConstraint->m_setting.m_damping = 0.2f;
    m_p2pConstraint->m_setting.m_impulseClamp = 1.0f;

    dynamicsWorld->addConstraint(m_p2pConstraint);
    spdlog::debug("[ChainSystem] Constraint of link {} added to world", m_chainNumber);
}
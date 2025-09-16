#include "PhysicsManager.h"
#include "ProgramValues.h"
#include "Model.h"
#include "Mesh.h"
#include "Camera.h"
#include "ModelInstance.h"
#include "ModelInstanceManager.h"
#include "DebugDrawer.h"
#include "PhysicsConstants.h"
#include "ObjectInfo.h"
#include "ThrowableSphere.h"
#include <spdlog/spdlog.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <BulletSoftBody/btSoftBodyHelpers.h>

btSoftRigidDynamicsWorld* PhysicsManager::dynamicsWorld = nullptr;
btSoftBodyWorldInfo* PhysicsManager::softBodyWorldInfo = nullptr;

DebugDrawer* PhysicsManager::debugDrawer = nullptr;

btCapsuleShape* PhysicsManager::playerShape = nullptr;
btBvhTriangleMeshShape* PhysicsManager::landscapeShape = nullptr;
btStaticPlaneShape* PhysicsManager::voidPlaneShape = nullptr;

btRigidBody* PhysicsManager::playerGhostBody = nullptr;
btRigidBody* PhysicsManager::landscapeBody = nullptr;
btRigidBody* PhysicsManager::voidPlaneBody = nullptr;

btSoftBody* PhysicsManager::clothBody = nullptr;

btVector3 PhysicsManager::gravity = btVector3(0, -9.8f, 0);
btVector3 PhysicsManager::playerStartingPosition = btVector3(10.0f, 13.0f, -8.5);

void PhysicsManager::init() {
	spdlog::info("Initializing physics manager...");

	initPhysicsWorld();
	initDebugger();
	initCollisionShapes();
	initRigidBodies();
	initSoftBodies();

	spdlog::info("Physics manager initialized successfully.");
}

void PhysicsManager::update(const float deltaTime) {
	dynamicsWorld->stepSimulation(deltaTime, 10);
	updateCamera();
	updateCollisions();
}

void PhysicsManager::updateModelMatrix(ModelInstance* model, btRigidBody* body) {
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

void PhysicsManager::updateModelMatrix(ModelInstance* model, btSoftBody* body) { 
    btVector3 com = body->m_pose.m_com;
    glm::vec3 pos(com.x(), com.y(), com.z());

    btMatrix3x3 rotMat = body->m_pose.m_rot;
    btQuaternion rot;
    rotMat.getRotation(rot);

    glm::quat glmQuat(rot.w(), rot.x(), rot.y(), rot.z());

    model->model =
        glm::translate(glm::mat4(1.0f), pos) *
        glm::mat4_cast(glmQuat) *
        glm::scale(glm::mat4(1.0f), glm::vec3(model->scale));

    // Each vertex has: pos(3) + normal(3) + texcoord(2) + tangent(3) + bitangent(3) = 14 floats
	const int stride = 14;


    for (int i = 0; i < body->m_nodes.size(); i++) {
        const btVector3& nodePos = body->m_nodes[i].m_x;
        const btVector3& nodeNormal = body->m_nodes[i].m_n;

        int base = i * stride;

		std::vector<float>& flatVertices = model->modelRef->flatVertices;

        flatVertices[base + 0] = nodePos.x();
        flatVertices[base + 1] = nodePos.y();
        flatVertices[base + 2] = nodePos.z();

        flatVertices[base + 3] = nodeNormal.x();
        flatVertices[base + 4] = nodeNormal.y();
        flatVertices[base + 5] = nodeNormal.z();
    }

	model->modelRef->syncSoftBodyVertices();
}


void PhysicsManager::updateCamera() {
    Camera* camera = &ProgramValues::Cameras::freeFly;

    if (!ProgramValues::CameraKeyEvents::isLockedIn)
        return;

    float modifiedSpeed = ProgramValues::CameraKeyEvents::sprinting
        ? camera->speed * camera->SPRINT_MULTIPLIER: camera->speed;

    if (ProgramValues::GameFlags::isFreeFlying) {
        playerGhostBody->setGravity(btVector3(0, 0, 0));
        playerGhostBody->clearForces();
        playerGhostBody->setLinearVelocity(btVector3(0,0,0));
        playerGhostBody->setAngularVelocity(btVector3(0,0,0));

        if (ProgramValues::CameraKeyEvents::moveForwardPressed)  camera->position += modifiedSpeed * camera->front;
        if (ProgramValues::CameraKeyEvents::moveLeftPressed)     camera->position -= glm::normalize(glm::cross(camera->front, camera->up)) * modifiedSpeed;
        if (ProgramValues::CameraKeyEvents::moveBackwardPressed) camera->position -= modifiedSpeed * camera->front;
        if (ProgramValues::CameraKeyEvents::moveRightPressed)    camera->position += glm::normalize(glm::cross(camera->front, camera->up)) * modifiedSpeed;
        if (ProgramValues::CameraKeyEvents::moveUpPressed)       camera->position += camera->speed * camera->up;
        if (ProgramValues::CameraKeyEvents::moveDownPressed)     camera->position -= camera->speed * camera->up;

        playerGhostBody->setWorldTransform(btTransform(
            btQuaternion(0, 0, 0, 1),
            btVector3(camera->position.x, camera->position.y, camera->position.z)
        ));
    } else {
        playerGhostBody->activate(true);
        playerGhostBody->setGravity(gravity);

        static float speed = 7.0f;
		static float jumpSpeed = 2.85f;

        btVector3 forward(camera->front.x, 0, camera->front.z);
        forward.normalize();

        btVector3 right = btVector3(camera->front.z, 0, -camera->front.x);
        right.normalize();

        btVector3 desiredVelocity(0, playerGhostBody->getLinearVelocity().y(), 0);

        if (ProgramValues::CameraKeyEvents::moveForwardPressed) {
            desiredVelocity += forward * speed;
        }
        if (ProgramValues::CameraKeyEvents::moveBackwardPressed) {
            desiredVelocity -= forward * speed;
        }
        if (ProgramValues::CameraKeyEvents::moveLeftPressed) {
            desiredVelocity += right * speed;
        }
        if (ProgramValues::CameraKeyEvents::moveRightPressed) {
            desiredVelocity -= right * speed;
        }

        playerGhostBody->setLinearVelocity(desiredVelocity);

		if (ProgramValues::CameraKeyEvents::moveUpPressed &&
			!ProgramValues::CameraKeyEvents::isOnJump) {
			btVector3 vel = playerGhostBody->getLinearVelocity();
			vel.setY(jumpSpeed);
			playerGhostBody->setLinearVelocity(vel);
		}

        btTransform transRef = PhysicsManager::getTrans(playerGhostBody);
        btVector3 pos = transRef.getOrigin();
        camera->position = glm::vec3(pos.x(), pos.y(), pos.z());
    }


    camera->updateCameraVectors();
}

void PhysicsManager::updateCollisions() {
	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		bool hasCollision = false;
		int numContacts = contactManifold->getNumContacts();

		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			if (pt.getDistance() < 0.0f) {
				hasCollision = true;
				break;
			}
		}

		auto* obj0 = const_cast<btCollisionObject*>(contactManifold->getBody0());
		auto* obj1 = const_cast<btCollisionObject*>(contactManifold->getBody1());

		if (!obj0 || !obj1) continue;

		auto* instance0 = static_cast<ModelInstance*>(obj0->getUserPointer());
		auto* instance1 = static_cast<ModelInstance*>(obj1->getUserPointer());

		if (!instance0 || !instance1) {
			spdlog::warn("Detected collision with unpointed model instance object(s). Set user pointers!");
			continue;
		}

		if (!instance0->info || !instance1->info) {
			spdlog::warn("Detected collision with ModelInstance(s) that have no ObjectInfo (info==null).");
			continue;
		}

		std::string name0 = instance0->info->name;
		std::string name1 = instance1->info->name;
		
		updateCollidedObjects(obj0, obj1, hasCollision);
	}
}

void PhysicsManager::updateCollidedObjects(btCollisionObject* obj0, btCollisionObject* obj1, const bool& hasCollision) {
	auto* instance0 = static_cast<ModelInstance*>(obj0->getUserPointer());
	auto* instance1 = static_cast<ModelInstance*>(obj1->getUserPointer());

	std::string name0 = instance0->info->name;
	std::string name1 = instance1->info->name;
	
	{ // Player and Landscape
		bool playerAndLandScape = (name0 == OBJECTS_POINTER_NAME::PLAYER && name1 == OBJECTS_POINTER_NAME::LANDSCAPE) ||
			(name0 == OBJECTS_POINTER_NAME::LANDSCAPE && name1 == OBJECTS_POINTER_NAME::PLAYER);

		if (playerAndLandScape) ProgramValues::CameraKeyEvents::isOnJump = !hasCollision;
	}

	{ // Player and void plane
		bool playerAndVoidPlane = (name0 == OBJECTS_POINTER_NAME::PLAYER && name1 == OBJECTS_POINTER_NAME::VOID_PLANE) ||
			(name0 == OBJECTS_POINTER_NAME::VOID_PLANE && name1 == OBJECTS_POINTER_NAME::PLAYER);

		if (playerAndVoidPlane && hasCollision) {
			btTransform trans;
			trans.setIdentity();
			trans.setOrigin(playerStartingPosition);
			playerGhostBody->setWorldTransform(trans);

			Camera* camera = &ProgramValues::Cameras::freeFly;
			btTransform transRef = PhysicsManager::getTrans(playerGhostBody);
			btVector3 pos = transRef.getOrigin();
			camera->position = glm::vec3(pos.x(), pos.y(), pos.z());
		}
	}

	{ // Throwable Sphere and void plane
		bool throwableSphereAndVoidPlane = (name0 == OBJECTS_POINTER_NAME::THROWABLE_SPHERE && name1 == OBJECTS_POINTER_NAME::VOID_PLANE) ||
			(name0 == OBJECTS_POINTER_NAME::VOID_PLANE && name1 == OBJECTS_POINTER_NAME::THROWABLE_SPHERE);

		if (throwableSphereAndVoidPlane && hasCollision) {
			void* userPtr = (name0 == OBJECTS_POINTER_NAME::THROWABLE_SPHERE)
				? obj0->getUserPointer()
				: obj1->getUserPointer();

			auto* rawInstance = static_cast<ModelInstance*>(userPtr);

			auto modelInstance = ThrowableSphere::findSharedPtr(rawInstance);

			if (modelInstance) {
				ThrowableSphere::queueRemoval(modelInstance);
			} else {
				spdlog::error("Failed to resolve shared_ptr for throwable sphere!");
			}
		}
	}
}

btSoftRigidDynamicsWorld* PhysicsManager::getWorld() {
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

    spdlog::trace("Creating collision configuration...");
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();

    spdlog::trace("Creating dispatcher...");
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);

    spdlog::trace("Creating broadphase...");
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    spdlog::trace("Creating solver...");
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    spdlog::trace("Creating dynamics world...");
	dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

	softBodyWorldInfo = new btSoftBodyWorldInfo;
	softBodyWorldInfo->m_broadphase = broadphase;
	softBodyWorldInfo->m_dispatcher = dispatcher;
	softBodyWorldInfo->m_gravity = dynamicsWorld->getGravity();
	softBodyWorldInfo->m_sparsesdf.Initialize();

    if (dynamicsWorld) {
        spdlog::info("Physics world created successfully.");
    } else {
        spdlog::error("Failed to create physics world.");
    }

    dynamicsWorld->setGravity(gravity);
    spdlog::debug("Gravity set to: {}, {}, {}", gravity.x(), gravity.y(), gravity.z());
}


void PhysicsManager::initDebugger() {
	debugDrawer = new DebugDrawer;
	dynamicsWorld->setDebugDrawer(debugDrawer);
}
void PhysicsManager::initCollisionShapes() {
    spdlog::info("Initializing collision shapes...");

    spdlog::trace("Building triangle mesh for landscape...");
    btTriangleMesh* triangleMesh = new btTriangleMesh;

    Model* model = &ProgramValues::GameObjects::landscape;
    size_t numTriangles = 0;

    for (auto& mesh : model->meshes) {
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            glm::vec4 v0 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i]].Position, 1.0f);
            glm::vec4 v1 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i + 1]].Position, 1.0f);
            glm::vec4 v2 = mesh.transform * glm::vec4(mesh.vertices[mesh.indices[i + 2]].Position, 1.0f);

            triangleMesh->addTriangle(
                btVector3(v0.x, v0.y, v0.z),
                btVector3(v1.x, v1.y, v1.z),
                btVector3(v2.x, v2.y, v2.z)
            );
            numTriangles++;
        }
    }
    spdlog::debug("Landscape triangle mesh built with {} triangles.", numTriangles);

    landscapeShape = new btBvhTriangleMeshShape(triangleMesh, true);

    const float& scale = ProgramValues::modelsPreTransformScale::landscape;

    landscapeShape->setLocalScaling(btVector3(scale, scale, scale));
    spdlog::info("Landscape collision shape created.");

    spdlog::trace("Creating player capsule shape...");
    playerShape = new btCapsuleShape(btScalar(0.2f), btScalar(0.8f));
    spdlog::info("Player capsule shape created with radius 0.2 and height 0.8.");

    spdlog::trace("Creating void plane shape...");
    voidPlaneShape = new btStaticPlaneShape(btVector3(0,1,0), btScalar(-100));
    spdlog::info("Void plane shape created at Y = -100.");

    spdlog::info("Collision shapes initialized successfully.");
}


void PhysicsManager::initRigidBodies() {
	spdlog::info("Initializing rigid bodies...");

	if (!dynamicsWorld) {
		spdlog::error("dynamicsWorld is null — cannot add rigid bodies!");
		return;
	}

	{
		auto landscapeInstance = std::make_shared<ModelInstance>(&ProgramValues::GameObjects::landscape);
		ModelInstanceManager::addModelInstance(
			ProgramValues::GameObjects::landscape.pointerName, landscapeInstance);
		landscapeInstance->scale = ProgramValues::modelsPreTransformScale::landscape;
		landscapeInstance->updateModelMatrix();
		landscapeInstance->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::LANDSCAPE);

		spdlog::info("Creating landscape rigid body...");
		if (!landscapeShape) {
			spdlog::error("landscapeShape is null!");
		}

		btScalar mass(0.0f);
		btVector3 inertia(0, 0, 0);

		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());
		if (!motionState) {
			spdlog::error("Failed to allocate motionState for landscape.");
		}

		btRigidBody::btRigidBodyConstructionInfo RBInfo(mass, motionState, landscapeShape, inertia);
		landscapeBody = new btRigidBody(RBInfo);
		if (!landscapeBody) {
			spdlog::error("Failed to create landscapeBody!");
		}

		dynamicsWorld->addRigidBody(landscapeBody, 
			COLLISION_CATEGORIES::ENVIRONMENT,
			COLLISION_CATEGORIES::PLAYER | COLLISION_CATEGORIES::OBJECTS);

		landscapeBody->setFriction(1.0f);

		landscapeBody->setUserPointer(landscapeInstance.get());


		spdlog::info("Landscape rigid body initialized.");
	}

	{
		auto playerGhostBodyInstance = std::make_shared<ModelInstance>(nullptr, false);
		ModelInstanceManager::addModelInstance(
			ProgramValues::ProxiesGameObjects::PROXY_PHYSICS_PLAYER.pointerName, playerGhostBodyInstance);
		playerGhostBodyInstance->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::PLAYER);

		spdlog::info("Creating player rigid body...");
		if (!playerShape) {
			spdlog::error("playerShape is null!");
			return;
		}

		btScalar mass = 60.0f;
		btVector3 inertia(0, 0, 0);
		playerShape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState* playerMotion = new btDefaultMotionState(btTransform(
			btQuaternion(0,0,0,1), playerStartingPosition
		));
		if (!playerMotion) {
			spdlog::error("Failed to allocate playerMotion.");
		}

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, playerMotion, playerShape, inertia);
		playerGhostBody = new btRigidBody(rbInfo);
		if (!playerGhostBody) {
			spdlog::error("Failed to create playerGhostBody!");
		}

		dynamicsWorld->addRigidBody(playerGhostBody,
			COLLISION_CATEGORIES::PLAYER,
			COLLISION_CATEGORIES::ENVIRONMENT |
			COLLISION_CATEGORIES::VOID_PLANE
		);

		playerGhostBody->setFriction(1.0f);
		playerGhostBody->setAngularFactor(btVector3(0,0,0));
		playerGhostBody->setRollingFriction(1.0f);
		playerGhostBody->setSpinningFriction(1.0f);
		playerGhostBody->setUserPointer(playerGhostBodyInstance.get());
		spdlog::info("Player rigid body initialized.");
	}

	{
		auto voidPlaneInstance = std::make_shared<ModelInstance>(nullptr, false);
		ModelInstanceManager::addModelInstance(
			ProgramValues::ProxiesGameObjects::PROXY_VOID_PLANE.pointerName, voidPlaneInstance);
		voidPlaneInstance->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::VOID_PLANE);

		spdlog::info("Creating void plane rigid body...");
		if (!voidPlaneShape) {
			spdlog::error("voidPlaneShape is null!");
		}

		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());
		if (!motionState) {
			spdlog::error("Failed to allocate motionState for void plane.");
		}

		btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, motionState, voidPlaneShape, btVector3(0, 0, 0));
		voidPlaneBody = new btRigidBody(rbInfo);
		if (!voidPlaneBody) {
			spdlog::error("Failed to create voidPlaneBody!");
		}

		dynamicsWorld->addRigidBody(voidPlaneBody, 
			COLLISION_CATEGORIES::VOID_PLANE,
			COLLISION_CATEGORIES::PLAYER | COLLISION_CATEGORIES::OBJECTS);

		voidPlaneBody->setUserPointer(voidPlaneInstance.get());
		spdlog::info("Void plane rigid body initialized.");
	}

	spdlog::info("Initialized rigid bodies successfully.");
}

void PhysicsManager::initSoftBodies() {
	{
		auto clothInstance = std::make_shared<ModelInstance>(&ProgramValues::GameObjects::cloth);
		ModelInstanceManager::addModelInstance(
			ProgramValues::GameObjects::cloth.pointerName, clothInstance);
		clothInstance->scale = ProgramValues::modelsPreTransformScale::cloth;
		clothInstance->updateModelMatrix();
		clothInstance->info = std::make_shared<ObjectInfo>(OBJECTS_POINTER_NAME::CLOTH);

		spdlog::info("Vertices: {}", clothInstance->modelRef->flatVertices.size()/3);
		spdlog::info("Indices : {}", clothInstance->modelRef->flatIndices.size());
		spdlog::info("Triangles: {}", clothInstance->modelRef->flatIndices.size()/3);

		const std::vector<float>& baseVerts = clothInstance->modelRef->flatVertices;
		const std::vector<int>& baseIndices = clothInstance->modelRef->flatIndices;

		clothBody = btSoftBodyHelpers::CreateFromTriMesh(
			*softBodyWorldInfo,
			baseVerts.data(),
			baseIndices.data(),
			baseIndices.size() / 3
		);

		btTransform move;
		move.setIdentity();
		move.setOrigin(btVector3(0, 35.0f, 0));

		clothBody->transform(move);

		btScalar scalar(ProgramValues::modelsPreTransformScale::cloth);
		clothBody->scale(btVector3(scalar, scalar, scalar));

		dynamicsWorld->addSoftBody(clothBody,
			COLLISION_CATEGORIES::OBJECTS,
			COLLISION_CATEGORIES::OBJECTS | COLLISION_CATEGORIES::ENVIRONMENT);
	}
}

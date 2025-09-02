#pragma once
#include <bullet/btBulletDynamicsCommon.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Model;
class DebugDrawer;

class PhysicsManager {
public:
	static void init();
	static void update(const float deltaTime);
	static void updateModelMatrix(Model* model, btRigidBody* body);
	static void renderDebugLines();

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
	static void initVertexObjects();
	static void initCollisionShapes();
	static void initRigidBodies();

private:
	static btCollisionShape* cubeShape;
	static btCollisionShape* planeShape;

public:
	struct LineVertex {
		glm::vec3 pos;
		glm::vec3 color;
	};

	static GLuint* gDebugVAO; 
	static GLuint* gDebugVBO;

	static std::vector<LineVertex> gDebugLines;

public:
	static btRigidBody* cubeBody;
	static btRigidBody* planeBody;
};


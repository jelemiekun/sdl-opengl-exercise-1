#pragma once
#include <SDL.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <unordered_map>

struct ModelInstance;

struct PhysicsProperties {
	PhysicsProperties(btSphereShape* r_Shape, btRigidBody* r_Body);

	btSphereShape* shape;
	btRigidBody* body;
};

class ThrowableSphere {
private:
	static std::unordered_map<ModelInstance*, PhysicsProperties> modelPhysicsMap;
	constexpr static Uint32 COOLDOWN_TIME = 1000;
	constexpr static float MIN_RADIUS = 0.1f;
	constexpr static float MAX_RADIUS = 1.5f;
	static Uint32 lastTime;

private:
	static void checkCooldownTimer();
	static bool isReadyToThrow();
	static void createThrowableSphere();
	static void addToModelTypeList(ModelInstance* sphere);
	static void addToModelPhysicsMap(ModelInstance* sphere);
	static PhysicsProperties generatePhysicsProperties();
	static float generateRandomRadius();
	static void manipulateRigidBody(btRigidBody* body);
	static void removeInstance(); // TODO
	static void removeInstanceToModelTypeList();
	static void removeInstanceToModelPhysicsMap();

public:
	static void input(SDL_Event& event);
	static void update();
};
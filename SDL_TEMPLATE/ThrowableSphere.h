#pragma once
#include <SDL.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <unordered_map>

struct ModelInstance;

struct PhysicsProperties {
	PhysicsProperties(btSphereShape* r_Shape, btRigidBody* r_Body);

	btSphereShape* shape;
	btRigidBody* body;

	~PhysicsProperties() = default;

    PhysicsProperties(const PhysicsProperties&) = delete;
    PhysicsProperties& operator=(const PhysicsProperties&) = delete;

    PhysicsProperties(PhysicsProperties&& o) noexcept : shape(o.shape), body(o.body) {
        o.shape = nullptr; o.body = nullptr;
    }
    PhysicsProperties& operator=(PhysicsProperties&& o) noexcept {
        if (this != &o) { shape = o.shape; body = o.body; o.shape = nullptr; o.body = nullptr; }
        return *this;
    }
};

class ThrowableSphere {
private:
	static std::unordered_map<std::shared_ptr<ModelInstance>, PhysicsProperties> modelPhysicsMap;
	constexpr static Uint32 COOLDOWN_TIME = 1000;
	constexpr static float MIN_RADIUS = 0.1f;
	constexpr static float MAX_RADIUS = 1.5f;
	static Uint32 lastTime;

private:
	static void checkCooldownTimer();
	static bool isReadyToThrow();
	static void createThrowableSphere();
	static void updateCooldownTimerAndFlag();
	static void addToModelTypeList(std::shared_ptr<ModelInstance> sphere);
	static void addToModelPhysicsMap(std::shared_ptr<ModelInstance> sphere);
	static PhysicsProperties generatePhysicsProperties(std::shared_ptr<ModelInstance> sphere);
	static float generateRandomRadius();
	static void manipulateRigidBody(btRigidBody& body);
	static void removeInstance(std::shared_ptr<ModelInstance> modelInstance); // TODO
	static void removeInstanceToModelTypeList(std::shared_ptr<ModelInstance> modelInstance);
	static void removeInstanceToModelPhysicsMap(std::shared_ptr<ModelInstance> modelInstance);
	static void deleteModelInstancePhysicalProperties(PhysicsProperties& physicsProperties);

public:
	static void input(SDL_Event& event);
	static void update();
};
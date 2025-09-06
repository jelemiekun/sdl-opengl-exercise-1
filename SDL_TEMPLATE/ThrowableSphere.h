#pragma once
#include <SDL.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <unordered_map>

struct ModelInstance;
struct ObjectInfo;

struct SharedPtrHash {
    size_t operator()(const std::shared_ptr<ModelInstance>& ptr) const noexcept {
        return std::hash<ModelInstance*>()(ptr.get());
    }
};

struct SharedPtrEqual {
    bool operator()(const std::shared_ptr<ModelInstance>& a,
                    const std::shared_ptr<ModelInstance>& b) const noexcept {
        return a.get() == b.get();
    }
};

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
public:
	static std::unordered_map<
		std::shared_ptr<ModelInstance>, 
		PhysicsProperties, 
		SharedPtrHash, 
		SharedPtrEqual
	> modelPhysicsMap;

private:
	constexpr static Uint32 COOLDOWN_TIME = 1000;
	constexpr static float MIN_RADIUS = 2.0f;
	constexpr static float MAX_RADIUS = 10.0f;
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
	static void removeInstance(std::shared_ptr<ModelInstance> modelInstance);
	static void removeInstanceToModelTypeList(std::shared_ptr<ModelInstance> modelInstance);
	static void removeInstanceToModelPhysicsMap(std::shared_ptr<ModelInstance> modelInstance);
	static void deleteModelInstancePhysicalProperties(PhysicsProperties& physicsProperties);

public:
	static void input(SDL_Event& event);
	static void update();
};
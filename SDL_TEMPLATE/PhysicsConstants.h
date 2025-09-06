#pragma once

namespace COLLISION_CATEGORIES {
	const short ENVIRONMENT		= 1 << 0;
	const short PLAYER			= 1 << 1;
	const short OBJECTS			= 1 << 2;
	const short VOID_PLANE		= 1 << 3;
};

namespace OBJECTS_POINTER_NAME {
	constexpr const char* PLAYER = "PLAYER";
	constexpr const char* LANDSCAPE = "LANDSCAPE";
	constexpr const char* VOID_PLANE = "VOID_PLANE";
	constexpr const char* THROWABLE_SPHERE = "THROWABLE_SPHERE";
};
#pragma once
#include <string>

namespace COLLISION_CATEGORIES {
	const short ENVIRONMENT		= 1 << 0;
	const short PLAYER			= 1 << 1;
	const short OBJECTS			= 1 << 2;
	const short VOID_PLANE		= 1 << 3;
};

namespace OBJECTS_POINTER_NAME {
	const std::string PLAYER = "PLAYER";
	const std::string LANDSCAPE = "LANDSCAPE";
	const std::string VOID_PLANE = "VOID_PLANE";
	const std::string THROWABLE_SPHERE = "THROWABLE_SPHERE";
	const std::string SINGLE_CHAIN = "SINGLE_CHAIN";
};
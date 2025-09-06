#include "ObjectInfo.h"

static int counter = 0;

ObjectInfo::ObjectInfo(const std::string& r_name) : name(r_name) , id(counter++) {}

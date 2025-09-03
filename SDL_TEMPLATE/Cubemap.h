#pragma once
#include <string>
#include <vector>
#include "glad/glad.h"

class Cubemap {
public:
    unsigned int rendererID;

    Cubemap();
    ~Cubemap();

    void init(const std::vector<std::string>& faces);
    void bind(unsigned int slot = 0) const;
    void unbind() const;

private:
    void loadCubemap(const std::vector<std::string>& faces);
};

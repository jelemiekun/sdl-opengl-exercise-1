#include "stb_image.h"
#include "Texture2D.h"
#include "glad/glad.h"
#include "spdlog/spdlog.h"

Texture2D::Texture2D(const std::string& path) :
	rendererID(0), filePath(path), localBuffer(nullptr), width(0), height(0), bpp(0) {
	stbi_set_flip_vertically_on_load(true);

	glGenTextures(1, &rendererID);
	glBindTexture(GL_TEXTURE_2D, rendererID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

	localBuffer = stbi_load(path.c_str(), &width, &height, &bpp, 4);

	if (localBuffer) {
		spdlog::info("Texture loaded successfully.");
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, localBuffer);
		glGenerateMipmap(GL_TEXTURE_2D);
		stbi_image_free(localBuffer);
	} else {
		spdlog::warn("Failed to load texture.");
	}

	glBindTexture(GL_TEXTURE_2D, 0);
}

Texture2D::~Texture2D() {
	glDeleteTextures(1, &rendererID);
}

void Texture2D::bind(unsigned int slot) const {
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, rendererID);
}

void Texture2D::unbind() const {
	glBindTexture(GL_TEXTURE_2D, 0);
}
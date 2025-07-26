#pragma once

#include <iostream>

class Texture2D {
private:
	unsigned int rendererID;
	std::string filePath;
	unsigned char* localBuffer;
	int width;
	int height;
	int bpp;

public:
	Texture2D(const std::string& path);
	~Texture2D();

	void bind(unsigned int slot = 0) const;
	void unbind() const;

	inline int getWidth() const { return width; }
	inline int getHeight() const { return height; }
};
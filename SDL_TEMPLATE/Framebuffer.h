#pragma once
#include <glad/glad.h>

class Framebuffer {
public:
    Framebuffer(unsigned int width, unsigned int height);
    ~Framebuffer();

    void bind() const;
    static void unbind();

    void resize(unsigned int newWidth, unsigned int newHeight);

    GLuint getColorTexture() const { return colorTex; }

private:
    GLuint fbo;
    GLuint colorTex;
    GLuint rbo;
    unsigned int width, height;

    void create();
    void cleanup();
};

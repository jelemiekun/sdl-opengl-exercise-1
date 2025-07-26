#pragma once
#include <SDL.h>

class GameWindow {
public:
    GameWindow();
    ~GameWindow();

    bool init();
    bool initOpenGLContext();

    void input(SDL_Event& e);
    void update();
    void render();

    void focus();
    void toggleFullscreen();
    
    int width();
    int height();
    bool hasMouseFocus();
    bool hasKeyboardFocus();
    bool isMinimized();
    bool isShown();
    SDL_Window* getWindow();
    SDL_GLContext& getGLContext();

    void free();
private:
    void handleWindowEvents(SDL_Event& e);
    void initVariables();
    void initOpenGlContextSetup();

private:
    SDL_Window* mWindow;        // Pointer to SDL window
    int mWindowID;              // Unique ID for this window
    int mWidth, mHeight;        // Dimensions

    bool mMouseFocus;           // Mouse focus flag
    bool mKeyboardFocus;        // Keyboard focus flag
    bool mFullScreen;           // Fullscreen state
    bool mMinimized;            // Minimized state
    bool mShown;                // Visibility state

    SDL_GLContext openGLContext;
};
#pragma once
#include <SDL.h>
#define SDL_MAIN_HANDLED
#include <memory>

class GameWindow;
class ImGuiWindow;

class Game {
private:
    Game();

public:
    Game(const Game&) = delete;
    Game& operator=(const Game&) = delete;
    Game(Game&&) = delete;
    Game& operator=(Game&&) = delete;

    static Game* getInstance();

private:
    bool running;
    GameWindow* gameWindow;
    SDL_Event event;

public:
    ImGuiWindow* imGuiWindow;

private:
    void initializeEverything();

    bool initSDL();
    bool initGameWindow();
    bool setOpenGLAttributes();
    bool initGLAD();
    bool initImGui();
    void initShaders();
    void initModels();
    void initCamera();
    void initFOVProjection();

    void gameLoop();

    void input();
    void update();
    void render();
public:
    void run();

    const bool& isRunning() const;
    void setRunning(bool value);

    void clean();
};


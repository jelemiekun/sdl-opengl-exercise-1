#include "Game.h"
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include "GameWindow.h"
#include "ImGuiWindow.h"
#include "FPSManager.h"
#include "Model.h"
#include "Shader.h"
#include "Camera.h"
#include "ProgramValues.h"
#include "imgui/imgui_impl_sdl2.h"

Game::Game() : running(false), gameWindow(nullptr), imGuiWindow(nullptr) {}

Game* Game::getInstance() {
    static Game instance;
    return &instance;
}

bool Game::setOpenGLAttributes() {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    return true;
}

bool Game::initSDL() {
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        spdlog::error("SDL Failed to initialize: {}", SDL_GetError());
        return false;
    } else {
        spdlog::info("SDL initialized successfully.");
    }

    return true;
}

bool Game::initGameWindow() {
    gameWindow = new GameWindow;

    if (gameWindow->init()) {
        spdlog::info("gameWindow initialized successfully.");
        return true;
    } else {
        spdlog::error("gameWindow failed to initialize.");
        return false;
    }
}

bool Game::initGLAD() {
    if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
        spdlog::error("Failed to initialize GLAD.");
        return false;
    }
    spdlog::info("GLAD initialized successfully.");
    return true;
}

bool Game::initImGui() {
    imGuiWindow = ImGuiWindow::getInstance();
    return imGuiWindow->init(gameWindow->getWindow(), gameWindow->getGLContext());
}

void Game::initShaders() {
    spdlog::info("Initializing shaders...");

    ProgramValues::Shaders::shaderObject.init("source.shader");
    ProgramValues::Shaders::shaderLight.init("light.shader");

    spdlog::info("Shaders initialized successfully.");
}

void Game::initModels() {
    spdlog::info("Initializing models...");

    ProgramValues::GameObjects::cube.init("assets/models/Geometry_Nodes.glb");

    spdlog::info("Models initialized successsfully.");
}

void Game::initCamera() {
    spdlog::info("Initializing cameras...");

    ProgramValues::Cameras::freeFly.init(
        glm::vec3(0.0f, 0.0f, 3.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        -90.0f,
        0.0f
    );

    spdlog::info("Cameras initialized successfully.");
}

void Game::initFOVProjection() {
    ProgramValues::GameWindow::projection = glm::perspective(
        glm::radians(ProgramValues::Cameras::freeFly.fov),
        (float)gameWindow->width() / (float)gameWindow->height(),
        1.0f,
        10000.0f
    );
}

void Game::initializeEverything() {
    spdlog::info("Initializing program...");

    bool initializationSuccess = initSDL() && 
        initGameWindow() && 
        gameWindow->initOpenGLContext() && 
        setOpenGLAttributes() &&
        initGLAD() &&
        initImGui();

    if (initializationSuccess) {
        spdlog::info("Program initialized successfully.");

        initShaders();
        initModels();
        initCamera();
        initFOVProjection();

        running = true;
    } else {
        spdlog::error("Program failed to initialize.");
    }
}

void Game::gameLoop() {
    int countFrame = 0;
    Uint32 startTime = SDL_GetTicks();
    Uint32 lastFrameTime = SDL_GetTicks();

    while (running) {
        Uint32 frameStart = SDL_GetTicks();

        // ProgramValues::GameWindow::deltaTime = (frameStart - lastFrameTime) / 1000.0f;
        lastFrameTime = frameStart;

        input();
        update();
        render();

        FPSManager::limitFPS(frameStart);
        FPSManager::calculateAverageFPS(countFrame, startTime);
    }

    clean();
}

void Game::input() {
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) running = false;
        ImGui_ImplSDL2_ProcessEvent(&event);
        gameWindow->input(event);
    }
}

void Game::update() {
    gameWindow->update();
}

void Game::render() {
    gameWindow->render();
}


void Game::run() {
    spdlog::info("Initializing program...");
    initializeEverything();

    if (running) {
        spdlog::info("Program initialized successfully.");
        gameLoop();
    } else {
        spdlog::info("Program failed to initialized.");
    }
}


const bool& Game::isRunning() const {
    return running;
}


void Game::setRunning(bool value) {
    running = value;
}

void Game::clean() {
    spdlog::info("Cleaning up ImGui...");
    imGuiWindow->clean();

    spdlog::info("Cleaning up custom GameWindow...");
    gameWindow->free();

    spdlog::info("Quitting SDL...");
    SDL_Quit();
}

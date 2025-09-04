#include "GameWindow.h"
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <memory>
#include "VertexArray.h"
#include "VertexBuffer.h"
#include "ElementBuffer.h"
#include "Shader.h"
#include "Texture2D.h"
#include "Camera.h"
#include "Game.h"
#include "ImGuiWindow.h"
#include "ProgramValues.h"
#include "Model.h"
#include "PhysicsManager.h"

static Game* game = Game::getInstance();

// Constructor
GameWindow::GameWindow()
    : mWindow(nullptr), openGLContext(nullptr), mWindowID(-1),
    mWidth(0), mHeight(0), mMouseFocus(false), mKeyboardFocus(false),
    mFullScreen(false), mMinimized(false), mShown(false) {}

// Destructor
GameWindow::~GameWindow() {
    free();
}

// Initializes the window
bool GameWindow::init() {
    mWindow = SDL_CreateWindow(
        "SDL Tutorial", 
        SDL_WINDOWPOS_UNDEFINED, 
        SDL_WINDOWPOS_UNDEFINED,
        ProgramValues::GameWindow::width, 
        ProgramValues::GameWindow::height, 
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL
    );

    if (!mWindow) {
        spdlog::error("Window creation failed: {}", SDL_GetError());
        return false;
    }

    initVariables();

    spdlog::info("Window initialized successfully");
    return true;
}

// Brings the window into focus
void GameWindow::focus() {
    if (!mShown) {
        SDL_ShowWindow(mWindow);
        spdlog::info("Window {} shown from focus()", mWindowID);
    }
    SDL_RaiseWindow(mWindow);
}

// Frees resources
void GameWindow::free() {
    if (mWindow) {
        spdlog::info("Destroying window {}", mWindowID);
        SDL_GL_DeleteContext(openGLContext);
        SDL_DestroyWindow(mWindow);
        mWindow = nullptr;
    }
    mWidth = mHeight = 0;
}

void GameWindow::toggleFullscreen() {
    if (mFullScreen) {
        SDL_SetWindowFullscreen(mWindow, 0);  // Remove fullscreen
        mFullScreen = false;
        spdlog::info("Fullscreen disabled for window {}", mWindowID);
    } else {
        SDL_SetWindowFullscreen(mWindow, SDL_WINDOW_FULLSCREEN_DESKTOP); // Set fullscreen
        mFullScreen = true;
        spdlog::info("Fullscreen enabled for window {}", mWindowID);
    }
}

void GameWindow::handleWindowEvents(SDL_Event& e) {
    if (e.type == SDL_WINDOWEVENT && e.window.windowID == mWindowID) {
        switch (e.window.event) {
            case SDL_WINDOWEVENT_SHOWN:
                mShown = true;
                spdlog::info("Window {} shown", mWindowID);
                break;

            case SDL_WINDOWEVENT_HIDDEN:
                mShown = false;
                spdlog::info("Window {} hidden", mWindowID);
                break;

            case SDL_WINDOWEVENT_SIZE_CHANGED:
                mWidth = e.window.data1;
                mHeight = e.window.data2;
                ProgramValues::GameWindow::width = mWidth;
                ProgramValues::GameWindow::height = mHeight;
                glViewport(0, 0, mWidth, mHeight);
                spdlog::info("Window {} resized to {}x{}", mWindowID, mWidth, mHeight);
                break;

            case SDL_WINDOWEVENT_RESTORED:
                ProgramValues::CameraKeyEvents::isLockedIn = true;
                spdlog::info("Window {} restored", mWindowID);
                break;

            case SDL_WINDOWEVENT_CLOSE:
                game->setRunning(false);
                SDL_HideWindow(mWindow);
                mShown = false;
                spdlog::info("Window {} closed and program stopping", mWindowID);
                break;
        }
    }

    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_F11) {
        toggleFullscreen();
        spdlog::info("Toggled fullscreen mode");
    }
}




// Handles events specific to this window
void GameWindow::input(SDL_Event& e) {
    handleWindowEvents(e);
    ProgramValues::Cameras::cameraReference->processInput(e, this);
}

void GameWindow::update() {
    if (ProgramValues::CameraKeyEvents::isLockedIn) {
        PhysicsManager::update(ProgramValues::GameWindow::deltaTime);
        PhysicsManager::updateModelMatrix(&ProgramValues::GameObjects::landscape, PhysicsManager::landscapeBody);
        
        // PhysicsManager::getWorld()->debugDrawWorld();
    }

    ProgramValues::Cameras::cameraReference->update();

    Shader* shaderObject = &ProgramValues::Shaders::shaderObject;

    shaderObject->bind();
    shaderObject->setVec3("u_CameraPos", ProgramValues::Cameras::cameraReference->position);
    shaderObject->setFloat("material.shininess", 32);

    { // Directional Light
        shaderObject->setVec3("dirLight.direction", -ProgramValues::Lights::References::dirLightRef->direction);
        shaderObject->setVec3("dirLight.ambient", ProgramValues::Lights::References::dirLightRef->ambient);
        shaderObject->setVec3("dirLight.diffuse", ProgramValues::Lights::References::dirLightRef->diffuse);
        shaderObject->setVec3("dirLight.specular", ProgramValues::Lights::References::dirLightRef->specular);
    }

    { // Spot Light
        shaderObject->setVec3("spotLight.position", ProgramValues::Cameras::cameraReference->position);
        shaderObject->setVec3("spotLight.direction", ProgramValues::Cameras::cameraReference->front);
        shaderObject->setFloat("spotLight.innerCutoff", ProgramValues::Lights::References::spotLightRef->innerCutoff);
        shaderObject->setFloat("spotLight.outerCutoff", ProgramValues::Lights::References::spotLightRef->outerCutoff);
        shaderObject->setFloat("spotLight.constant", ProgramValues::Lights::References::spotLightRef->constant);
        shaderObject->setFloat("spotLight.linear", ProgramValues::Lights::References::spotLightRef->linear);
        shaderObject->setFloat("spotLight.quadratic", ProgramValues::Lights::References::spotLightRef->quadratic);
        shaderObject->setVec3("spotLight.ambient", ProgramValues::Lights::References::spotLightRef->ambient);
        shaderObject->setVec3("spotLight.diffuse", ProgramValues::Lights::References::spotLightRef->diffuse);
        shaderObject->setVec3("spotLight.specular", ProgramValues::Lights::References::spotLightRef->specular);
    }
}

void GameWindow::render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.3f, 0.3f, 0.5f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    {
        Shader* shaderObject = &ProgramValues::Shaders::shaderObject;
        shaderObject->bind();
        shaderObject->setMat4("u_Projection", ProgramValues::GameWindow::projection);
        shaderObject->setMat4("u_View", ProgramValues::Cameras::cameraReference->getViewMatrix());
    
        auto drawModel = [this](Model* modelRef) -> void {
            Shader* shaderObject = &ProgramValues::Shaders::shaderObject;
            shaderObject->setMat3("u_NormalMatrix", modelRef->getNormalMatrix());
            shaderObject->setFloat("material.shininess", 1.0f);
        
            modelRef->Draw(*shaderObject);
        };

        drawModel(&ProgramValues::GameObjects::landscape);

        shaderObject->unbind();
    } 

    {
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);

        Shader* shaderSkybox = &ProgramValues::Shaders::shaderSkybox;
        shaderSkybox->bind();

        ProgramValues::Textures::skybox.bind();

        glm::mat4 view = glm::mat4(glm::mat3(ProgramValues::Cameras::cameraReference->getViewMatrix())); 

        shaderSkybox->setInt("skybox", 0);
        shaderSkybox->setMat4("u_Projection", ProgramValues::GameWindow::projection);
        shaderSkybox->setMat4("u_View", view);

        glBindVertexArray(ProgramValues::VertexArray::skyboxVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);

        shaderSkybox->unbind();
    }

    game->imGuiWindow->render();
    SDL_GL_SwapWindow(mWindow);
}




// Getters
int GameWindow::width() { return mWidth; }
int GameWindow::height() { return mHeight; }
bool GameWindow::hasMouseFocus() { return mMouseFocus; }
bool GameWindow::hasKeyboardFocus() { return mKeyboardFocus; }
bool GameWindow::isMinimized() { return mMinimized; }
bool GameWindow::isShown() { return mShown; }

SDL_Window* GameWindow::getWindow() {
    return mWindow ? mWindow : nullptr;
}

SDL_GLContext& GameWindow::getGLContext() {
    return openGLContext;
}

void GameWindow::initVariables() {
    mMouseFocus = mKeyboardFocus = true;
    mShown = true;
    mWidth = ProgramValues::GameWindow::width;
    mHeight = ProgramValues::GameWindow::height;
    mWindowID = SDL_GetWindowID(mWindow);
}

void GameWindow::initOpenGlContextSetup() {
    glViewport(0, 0, ProgramValues::GameWindow::width, ProgramValues::GameWindow::height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
}

bool GameWindow::initOpenGLContext() {
    openGLContext = SDL_GL_CreateContext(mWindow);
    if (!openGLContext) {
        spdlog::error("SDL_GL_CreateContext failed: {}", SDL_GetError());
        return false;
    }

    if (SDL_GL_MakeCurrent(mWindow, openGLContext) < 0) {
        spdlog::error("SDL_GL_MakeCurrent failed: {}", SDL_GetError());
        return false;
    }

    spdlog::info("SDL_GL_CreateContext on Window {} initialized successfully.", mWindowID);
    return true;
}

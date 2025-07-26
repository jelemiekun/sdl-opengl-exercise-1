#include "Camera.h"
#include <SDL.h>
#include "ProgramValues.h"
#include "GameWindow.h"
#include "imgui/imgui.h"

Camera::Camera()
    : front(glm::vec3(0.0f, 0.0f, -1.0f)), speed(0.05f), sensitivity(0.1f) {
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(position, position + front, up);
}

void Camera::init(glm::vec3 startPosition, glm::vec3 startUp,
    float startYaw, float startPitch) {
    position = startPosition;
    worldUp = startUp;
    yaw = startYaw;
    pitch = startPitch;
    updateCameraVectors();
}

void Camera::processInput(SDL_Event& event, GameWindow* window) {
    processKeyboard(event, window);
    processMouseMotion(event);
}

void Camera::processKeyboard(SDL_Event& event, GameWindow* window) {
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                if (!ProgramValues::CameraKeyEvents::isLockedInPressed) {
                    ProgramValues::CameraKeyEvents::isLockedInPressed = true;
                    ProgramValues::CameraKeyEvents::isLockedIn = !ProgramValues::CameraKeyEvents::isLockedIn;

                    if (!ProgramValues::CameraKeyEvents::isLockedIn) {
                        SDL_WarpMouseInWindow(window->getWindow(), window->width() / 2, window->height() / 2);
                    }
                }
                break;
            case SDLK_w:     ProgramValues::CameraKeyEvents::moveForwardPressed  = true; break;
            case SDLK_a:     ProgramValues::CameraKeyEvents::moveLeftPressed     = true; break;
            case SDLK_s:     ProgramValues::CameraKeyEvents::moveBackwardPressed = true; break;
            case SDLK_d:     ProgramValues::CameraKeyEvents::moveRightPressed    = true; break;
            case SDLK_SPACE: ProgramValues::CameraKeyEvents::moveUpPressed       = true; break;
            case SDLK_LCTRL: ProgramValues::CameraKeyEvents::moveDownPressed     = true; break;
            case SDLK_r:     ProgramValues::CameraKeyEvents::sprinting           = true; break;
            case SDLK_LSHIFT:ProgramValues::CameraKeyEvents::fastZoom            = true; break;
            default: break;
        }
    } else if (event.type == SDL_KEYUP) {
        switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                if (ProgramValues::CameraKeyEvents::isLockedInPressed) {
                    ProgramValues::CameraKeyEvents::isLockedInPressed = false;
                }
                break;
            case SDLK_w:     ProgramValues::CameraKeyEvents::moveForwardPressed  = false; break;
            case SDLK_a:     ProgramValues::CameraKeyEvents::moveLeftPressed     = false; break;
            case SDLK_s:     ProgramValues::CameraKeyEvents::moveBackwardPressed = false; break;
            case SDLK_d:     ProgramValues::CameraKeyEvents::moveRightPressed    = false; break;
            case SDLK_SPACE: ProgramValues::CameraKeyEvents::moveUpPressed       = false; break;
            case SDLK_LCTRL: ProgramValues::CameraKeyEvents::moveDownPressed     = false; break;
            case SDLK_r:     ProgramValues::CameraKeyEvents::sprinting           = false; break;
            case SDLK_LSHIFT:ProgramValues::CameraKeyEvents::fastZoom            = false; break;
            default: break;
        }
    }
}

void Camera::processMouseMotion(SDL_Event& event) {
    if (event.type == SDL_MOUSEMOTION && ProgramValues::CameraKeyEvents::isLockedIn) {
        float xoffset = static_cast<float>(event.motion.xrel);
        float yoffset = static_cast<float>(event.motion.yrel);

        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch -= yoffset;   

        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;
    } else if (event.type == SDL_MOUSEBUTTONDOWN) {
        ImGuiIO& io = ImGui::GetIO();
        if (event.button.button == SDL_BUTTON_LEFT && !ProgramValues::CameraKeyEvents::isLockedIn && !io.WantCaptureMouse)
            ProgramValues::CameraKeyEvents::isLockedIn = true;
    } else if (event.type == SDL_MOUSEWHEEL && ProgramValues::CameraKeyEvents::isLockedIn) {
        float localSensitivity =
            ProgramValues::CameraKeyEvents::fastZoom ? event.wheel.y * sensitivity * FOV_SPEED_MULTIPLIER : event.wheel.y * sensitivity;
        fov -= localSensitivity;

        if (fov < 1.0f) fov = 1.0f;
        if (fov > FOV_MAX) fov = FOV_MAX;
    }
}

void Camera::setViewToShader(GLuint shaderID, const std::string& uniformName) const {
    glm::mat4 view = getViewMatrix();
    glUniformMatrix4fv(glGetUniformLocation(shaderID, uniformName.c_str()), 1, GL_FALSE, &view[0][0]);
}

void Camera::update() {
    SDL_SetRelativeMouseMode(ProgramValues::CameraKeyEvents::isLockedIn ? SDL_TRUE : SDL_FALSE);

    if (ProgramValues::CameraKeyEvents::isLockedIn) {
        float modifiedSpeed = ProgramValues::CameraKeyEvents::sprinting ? speed * SPRINT_MULTIPLIER : speed;

        if (ProgramValues::CameraKeyEvents::moveForwardPressed)  position += modifiedSpeed * front;
        if (ProgramValues::CameraKeyEvents::moveLeftPressed)     position -= glm::normalize(glm::cross(front, up)) * modifiedSpeed;
        if (ProgramValues::CameraKeyEvents::moveBackwardPressed) position -= modifiedSpeed * front;
        if (ProgramValues::CameraKeyEvents::moveRightPressed)    position += glm::normalize(glm::cross(front, up)) * modifiedSpeed;
        if (ProgramValues::CameraKeyEvents::moveUpPressed)       position += speed * up;
        if (ProgramValues::CameraKeyEvents::moveDownPressed)     position -= speed * up;

        updateCameraVectors();
    }
}

void Camera::updateCameraVectors() {
    glm::vec3 dir;
    dir.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    dir.y = sin(glm::radians(pitch));
    dir.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(dir);

    right = glm::normalize(glm::cross(front, worldUp));
    up = glm::normalize(glm::cross(right, front));
}
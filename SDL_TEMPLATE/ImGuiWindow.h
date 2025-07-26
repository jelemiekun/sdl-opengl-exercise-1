#pragma once
#include <SDL.h>

class ImGuiWindow {
private:
    ImGuiWindow();

public:
    ImGuiWindow(const ImGuiWindow&) = delete;
    ImGuiWindow& operator=(const ImGuiWindow&) = delete;
    ImGuiWindow(ImGuiWindow&&) = delete;
    ImGuiWindow& operator=(ImGuiWindow&&) = delete;

    static ImGuiWindow* getInstance();

    bool init(SDL_Window* window, SDL_GLContext glContext) const;
    void render();
    void clean();
};
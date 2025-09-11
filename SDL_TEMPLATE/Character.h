#pragma once
#include <memory>
#include <SDL.h>
#include "CharacterState.h"

class Shader;

class Character {
private:
	std::unique_ptr<CharacterState> currentState;

public:
	Character();

	void setState(std::unique_ptr<CharacterState> newState);
	void input(SDL_Event& e);
	void update();
};
#pragma once
#include <SDL.h>

class Shader;

class CharacterState {
public:
	virtual void input(SDL_Event& e) = 0;
	virtual void update() = 0;
	virtual ~CharacterState() = default;
};

class FreeFly : public CharacterState {
public:
	void input(SDL_Event& e) override;
	void update() override;
};

class Drive : public CharacterState {
public:
	void input(SDL_Event& e) override;
	void update() override;
};
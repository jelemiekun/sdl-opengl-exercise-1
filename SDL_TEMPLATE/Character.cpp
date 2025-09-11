#include "CharacterState.h"
#include "Character.h"

Character::Character() : currentState(std::make_unique<FreeFly>()) {
}

void Character::setState(std::unique_ptr<CharacterState> newState) {
	currentState = std::move(newState);
}

void Character::input(SDL_Event& e) {
	currentState->input(e);
}

void Character::update() {
	currentState->update();
}
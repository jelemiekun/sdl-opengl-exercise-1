#include <SDL.h>
#include "Game.h"

int SDL_main(int argc, char* argv[]) {
    Game* game = Game::getInstance();
    game->run();

    return 0;
}
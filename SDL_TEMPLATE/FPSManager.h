#pragma once
#include <SDL.h>

class FPSManager {
public:
	static void calculateAverageFPS(int& countFrame, Uint32& startTime);
	static void limitFPS(const Uint32& frameStart);
};
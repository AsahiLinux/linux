// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

enum GameFlagsValues
{
	// To be continued

	/**
	 * While this bit is NOT set, the game will :
	 * - Disable manual movement of the character
	 * - Disable shooting bullets
	 * - Disable shooting Curly's nemesis
	 * - Disable changing weapons
	 * - Disable speeding up the display of text in TSC scripts
	 * - Disable damage of the character
	 * - Not display the HUD (Life, EXP, air, weapons)
	 * - Disable animation of the character
	 * - Disable movement of the inventory cursor
	 * - Disable getting out of the inventory while on the item section
	 * - Create a bullet if some other conditions are fullfilled while iterating over the stars in ActStar (If you have any idea of how that actually works, you may
	 * want to replace this line with a better explanation)
	 */
	GAME_FLAG_IS_CONTROL_ENABLED = 2

	// To be continued
};

extern int g_GameFlags;
extern int gCounter;

int Random(int min, int max);
void PutNumber4(int x, int y, int value, BOOL bZero);

BOOL Game(HWND hWnd);

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

enum
{
	BACKGROUND_TYPE_STATIONARY = 0,   // Doesn't move at all
	BACKGROUND_TYPE_MOVE_DISTANT = 1, // Moves at half the speed of the foreground
	BACKGROUND_TYPE_MOVE_NEAR = 2,    // Moves at the same speed as the foreground
	BACKGROUND_TYPE_WATER = 3,        // No background - draws a water foreground layer instead
	BACKGROUND_TYPE_BLACK = 4,        // No background - just black
	BACKGROUND_TYPE_AUTOSCROLL = 5,   // Constantly scrolls to the left (used by Ironhead)
	BACKGROUND_TYPE_CLOUDS_WINDY = 6, // Fancy parallax scrolling, items are blown to the left (used by bkMoon)
	BACKGROUND_TYPE_CLOUDS = 7        // Fancy parallax scrolling (used by bkFog)
};

typedef struct BACK
{
	BOOL flag;	// Unused - purpose unknown
	int partsW;
	int partsH;
	int numX;
	int numY;
	int type;
	int fx;
} BACK;

extern BACK gBack;
extern int gWaterY;

BOOL InitBack(const char *fName, int type);
void ActBack(void);
void PutBack(int fx, int fy);
void PutFront(int fx, int fy);

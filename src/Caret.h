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
	CARET_NULL = 0,
	CARET_BUBBLE = 1,
	CARET_PROJECTILE_DISSIPATION = 2,
	CARET_SHOOT = 3,
	CARET_SNAKE_AFTERIMAGE = 4,
	CARET_ZZZ = 5,
	CARET_SNAKE_AFTERIMAGE_DUPLICATE = 6,
	CARET_EXHAUST = 7,
	CARET_DROWNED_QUOTE = 8,
	CARET_QUESTION_MARK = 9,
	CARET_LEVEL_UP = 10,
	CARET_HURT_PARTICLES = 11,
	CARET_EXPLOSION = 12,
	CARET_TINY_PARTICLES = 13,
	CARET_UNKNOWN = 14,
	CARET_PROJECTILE_DISSIPATION_TINY = 15,
	CARET_EMPTY = 16,
	CARET_PUSH_JUMP_KEY = 17
};

void InitCaret(void);
void ActCaret(void);
void PutCaret(int fx, int fy);
void SetCaret(int x, int y, int code, int dir);

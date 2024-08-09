// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Caret.h"

#include <string.h>

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Draw.h"
#include "Game.h"
#include "Triangle.h"

#define CARET_MAX 0x40

struct CARET
{
	int cond;
	int code;
	int direct;
	int x;
	int y;
	int xm;
	int ym;
	int act_no;
	int act_wait;
	int ani_no;
	int ani_wait;
	int view_left;
	int view_top;
	RECT rect;
};

struct CARET_TABLE
{
	int view_left;
	int view_top;
};

CARET gCrt[CARET_MAX];

// Sprite offsets
CARET_TABLE gCaretTable[] = {
	{0, 0},                   // CARET_NULL
	{ 4 * 0x200,  4 * 0x200}, // CARET_BUBBLE
	{ 8 * 0x200,  8 * 0x200}, // CARET_PROJECTILE_DISSIPATION
	{ 8 * 0x200,  8 * 0x200}, // CARET_SHOOT
	{ 8 * 0x200,  8 * 0x200}, // CARET_SNAKE_AFTERIMAGE
	{ 4 * 0x200,  4 * 0x200}, // CARET_ZZZ
	{ 8 * 0x200,  8 * 0x200}, // CARET_SNAKE_AFTERIMAGE_DUPLICATE
	{ 4 * 0x200,  4 * 0x200}, // CARET_EXHAUST
	{ 8 * 0x200,  8 * 0x200}, // CARET_DROWNED_QUOTE
	{ 8 * 0x200,  8 * 0x200}, // CARET_QUESTION_MARK
	{28 * 0x200,  8 * 0x200}, // CARET_LEVEL_UP
	{ 4 * 0x200,  4 * 0x200}, // CARET_HURT_PARTICLES
	{16 * 0x200, 16 * 0x200}, // CARET_EXPLOSION
	{ 4 * 0x200,  4 * 0x200}, // CARET_TINY_PARTICLES
	{20 * 0x200, 20 * 0x200}, // CARET_UNKNOWN
	{ 4 * 0x200,  4 * 0x200}, // CARET_PROJECTILE_DISSIPATION_TINY
	{20 * 0x200,  4 * 0x200}, // CARET_EMPTY
	{52 * 0x200,  4 * 0x200}  // CARET_PUSH_JUMP_KEY
};

void InitCaret(void)
{
	memset(gCrt, 0, sizeof(gCrt));
}

// Null
void ActCaret00(CARET *crt)
{
	(void)crt;
}

// Bubble
void ActCaret01(CARET *crt)
{
	RECT rcLeft[4] = {
		{ 0, 64,  8, 72},
		{ 8, 64, 16, 72},
		{16, 64, 24, 72},
		{24, 64, 32, 72},
	};

	RECT rcRight[4] = {
		{64, 24, 72, 32},
		{72, 24, 80, 32},
		{80, 24, 88, 32},
		{88, 24, 96, 32},
	};

	if (crt->act_no == 0)
	{
		crt->act_no = 1;
		crt->xm = Random(-0x400, 0x400);
		crt->ym = Random(-0x400, 0);
	}

	crt->ym += 0x40;
	crt->x += crt->xm;
	crt->y += crt->ym;

	if (++crt->ani_wait > 5)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 3)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rcLeft' and 'rcRight', even though it's now too high
		#endif
		}
	}

	if (crt->direct == DIR_LEFT)
		crt->rect = rcLeft[crt->ani_no];
	else
		crt->rect = rcRight[crt->ani_no];
}

// Projectile dissipation
void ActCaret02(CARET *crt)
{
	RECT rect_left[4] = {
		{ 0, 32, 16, 48},
		{16, 32, 32, 48},
		{32, 32, 48, 48},
		{48, 32, 64, 48},
	};

	RECT rect_right[4] = {
		{176, 0, 192, 16},
		{192, 0, 208, 16},
		{208, 0, 224, 16},
		{224, 0, 240, 16},
	};

	RECT rect_up[3] = {
		{ 0, 32, 16, 48},
		{32, 32, 48, 48},
		{16, 32, 32, 48},
	};

	switch (crt->direct)
	{
		case DIR_LEFT:
			crt->ym -= 0x10;
			crt->y += crt->ym;

			if (++crt->ani_wait > 5)
			{
				crt->ani_wait = 0;
				++crt->ani_no;
			}

			if (crt->ani_no > 3)
			{
				crt->cond = 0;
			#ifdef FIX_MAJOR_BUGS
				return;	// The code below will use 'ani_no' to access 'rect_left', even though it's now too high
			#endif
			}

			crt->rect = rect_left[crt->ani_no];
			break;

		case DIR_RIGHT:
			if (++crt->ani_wait > 2)
			{
				crt->ani_wait = 0;
				++crt->ani_no;
			}

			if (crt->ani_no > 3)
			{
				crt->cond = 0;
			#ifdef FIX_MAJOR_BUGS
				return;	// The code below will use 'ani_no' to access 'rect_right', even though it's now too high
			#endif
			}

			crt->rect = rect_right[crt->ani_no];
			break;

		case DIR_UP:
			crt->rect = rect_up[++crt->ani_wait / 2 % 3];

			if (crt->ani_wait > 24)
				crt->cond = 0;

			break;
	}
}

// Shoot
void ActCaret03(CARET *crt)
{
	RECT rect[4] = {
		{ 0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
		{48, 48, 64, 64},
	};

	if (++crt->ani_wait > 2)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 3)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
		#endif
		}
	}

	crt->rect = rect[crt->ani_no];
}

// Snake after-image? This doesn't seem to be used.
void ActCaret04(CARET *crt)
{
	RECT rect[9] = {
		// Left
		{64, 32,  80, 48},
		{80, 32,  96, 48},
		{96, 32, 112, 48},
		// Up
		{64, 48,  80, 64},
		{80, 48,  96, 64},
		{96, 48, 112, 64},
		// Right
		{64, 64,  80, 80},
		{80, 64,  96, 80},
		{96, 64, 112, 80},
	};

	if (++crt->ani_wait > 1)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 2)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
		#endif
		}
	}

	crt->rect = rect[(crt->direct * 3) + crt->ani_no];
}

// 'Zzz' - snoring
void ActCaret05(CARET *crt)
{
	RECT rect[7] = {
		{32, 64, 40, 72},
		{32, 72, 40, 80},
		{40, 64, 48, 72},
		{40, 72, 48, 80},
		{40, 64, 48, 72},
		{40, 72, 48, 80},
		{40, 64, 48, 72},
	};

	if (++crt->ani_wait > 4)
	{
		crt->ani_wait = 0;
		++crt->ani_no;
	}

	if (crt->ani_no > 6)
	{
		crt->cond = 0;
	#ifdef FIX_MAJOR_BUGS
		return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
	#endif
	}

	crt->x += 0x80;
	crt->y -= 0x80;

	crt->rect = rect[crt->ani_no];
}

// No ActCaret06...

// Exhaust (used by the Booster and hoverbike)
void ActCaret07(CARET *crt)
{
	RECT rcLeft[7] = {
		{ 56, 0,  64, 8},
		{ 64, 0,  72, 8},
		{ 72, 0,  80, 8},
		{ 80, 0,  88, 8},
		{ 88, 0,  96, 8},
		{ 96, 0, 104, 8},
		{104, 0, 112, 8},
	};

	if (++crt->ani_wait > 1)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 6)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rcLeft', even though it's now too high
		#endif
		}
	}

	crt->rect = rcLeft[crt->ani_no];

	switch (crt->direct)
	{
		case DIR_LEFT:
			crt->x -= 2 * 0x200;
			break;
		case DIR_UP:
			crt->y -= 2 * 0x200;
			break;
		case DIR_RIGHT:
			crt->x += 2 * 0x200;
			break;
		case DIR_DOWN:
			crt->y += 2 * 0x200;
			break;
	}
}

// Drowned Quote
void ActCaret08(CARET *crt)
{
	RECT rcLeft = {16, 80, 32, 96};
	RECT rcRight = {32, 80, 48, 96};

	if (crt->direct == DIR_LEFT)
		crt->rect = rcLeft;
	else
		crt->rect = rcRight;
}

// The '?' that appears when you press the down key
void ActCaret09(CARET *crt)
{
	RECT rcLeft = {0, 80, 16, 96};
	RECT rcRight = {48, 64, 64, 80};

	if (++crt->ani_wait < 5)
		crt->y -= 0x800;

	if (crt->ani_wait == 32)
		crt->cond = 0;

	if (crt->direct == DIR_LEFT)
		crt->rect = rcLeft;
	else
		crt->rect = rcRight;
}

// 'Level Up!'
void ActCaret10(CARET *crt)
{
	RECT rcLeft[2] = {
		{0,  0, 56, 16},
		{0, 16, 56, 32},
	};

	RECT rcRight[2] = {
		{0,  96, 56, 112},
		{0, 112, 56, 128},
	};

	++crt->ani_wait;

	if (crt->direct == DIR_LEFT)
	{
		if (crt->ani_wait < 20)
			crt->y -= 2 * 0x200;

		if (crt->ani_wait == 80)
			crt->cond = 0;
	}
	else
	{
		if (crt->ani_wait < 20)
			crt->y -= 1 * 0x200;

		if (crt->ani_wait == 80)
			crt->cond = 0;
	}

	if (crt->direct == DIR_LEFT)
		crt->rect = rcLeft[crt->ani_wait / 2 % 2];
	else
		crt->rect = rcRight[crt->ani_wait / 2 % 2];
}

// Red hurt particles (used by bosses and invisible hidden pickups)
void ActCaret11(CARET *crt)
{
	unsigned char deg;

	if (crt->act_no == 0)
	{
		crt->act_no = 1;
		deg = Random(0, 0xFF);
		crt->xm = GetCos(deg) * 2;
		crt->ym = GetSin(deg) * 2;
	}

	crt->x += crt->xm;
	crt->y += crt->ym;

	RECT rcRight[7] = {
		{ 56, 8,  64, 16},
		{ 64, 8,  72, 16},
		{ 72, 8,  80, 16},
		{ 80, 8,  88, 16},
		{ 88, 8,  96, 16},
		{ 96, 8, 104, 16},
		{104, 8, 112, 16},
	};

	if (++crt->ani_wait > 2)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 6)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rcRight', even though it's now too high
		#endif
		}
	}

	crt->rect = rcRight[crt->ani_no];
}

// Missile Launcher explosion flash
void ActCaret12(CARET *crt)
{
	RECT rcLeft[2] = {
		{112, 0, 144, 32},
		{144, 0, 176, 32},
	};

	if (++crt->ani_wait > 2)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 1)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rcLeft', even though it's now too high
		#endif
		}
	}

	crt->rect = rcLeft[crt->ani_no];
}

// Particles used when Quote jumps into the ceiling, and also used by the Demon Crown and Ballos's puppy
void ActCaret13(CARET *crt)
{
	RECT rcLeft[2] = {
		{56, 24, 64, 32},
		{0, 0, 0, 0},
	};

	if (crt->act_no == 0)
	{
		crt->act_no = 1;

		switch (crt->direct)
		{
			case DIR_LEFT:
				crt->xm = Random(-0x600, 0x600);
				crt->ym = Random(-0x200, 0x200);
				break;

			case DIR_UP:
				crt->ym = -0x200 * Random(1, 3);
				break;
		}
	}

	switch (crt->direct)
	{
		case DIR_LEFT:
			crt->xm = (crt->xm * 4) / 5;
			crt->ym = (crt->ym * 4) / 5;
			break;
	}

	crt->x += crt->xm;
	crt->y += crt->ym;

	if (++crt->ani_wait > 20)
		crt->cond = 0;

	crt->rect = rcLeft[crt->ani_wait / 2 % 2];

	if (crt->direct == DIR_OTHER)
		crt->x -= 4 * 0x200;
}

// Broken (unknown and unused)
void ActCaret14(CARET *crt)
{
	// These rects are invalid.
	// However, notably, there are 5 unused 40x40 sprites at the bottom of Caret.pbm.
	// Perhaps those were originally at these coordinates.
	RECT rect[5] = {
		{0, 96, 40, 136},
		{40, 96, 80, 136},
		{80, 96, 120, 136},
		{120, 96, 160, 136},
		{160, 96, 200, 136},
	};

	if (++crt->ani_wait > 1)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 4)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
		#endif
		}
	}

	crt->rect = rect[crt->ani_no];
}

// Tiny version of the projectile dissipation effect
void ActCaret15(CARET *crt)
{
	RECT rcLeft[4] = {
		{ 0, 72,  8, 80},
		{ 8, 72, 16, 80},
		{16, 72, 24, 80},
		{24, 72, 32, 80},
	};

	if (++crt->ani_wait > 2)
	{
		crt->ani_wait = 0;

		if (++crt->ani_no > 3)
		{
			crt->cond = 0;
		#ifdef FIX_MAJOR_BUGS
			return;	// The code below will use 'ani_no' to access 'rcLeft', even though it's now too high
		#endif
		}
	}

	crt->rect = rcLeft[crt->ani_no];
}

// 'Empty!'
void ActCaret16(CARET *crt)
{
	RECT rcLeft[2] = {
		{104,  96, 144, 104},
		{104, 104, 144, 112},
	};

	if (++crt->ani_wait < 10)
		crt->y -= 2 * 0x200;

	if (crt->ani_wait == 40)
		crt->cond = 0;

	crt->rect = rcLeft[crt->ani_wait / 2 % 2];
}

// 'PUSH JUMP KEY!' (unused)
void ActCaret17(CARET *crt)
{
	RECT rcLeft[2] = {
		{0, 144, 144, 152},
		{0, 0, 0, 0},
	};

	if (++crt->ani_wait >= 40)
		crt->ani_wait = 0;

	if (crt->ani_wait < 30)
		crt->rect = rcLeft[0];
	else
		crt->rect = rcLeft[1];
}

typedef void (*CARETFUNCTION)(CARET*);
CARETFUNCTION gpCaretFuncTbl[] =
{
	ActCaret00,
	ActCaret01,
	ActCaret02,
	ActCaret03,
	ActCaret04,
	ActCaret05,
	ActCaret04, // Interestingly, this slot is a duplicate
	ActCaret07,
	ActCaret08,
	ActCaret09,
	ActCaret10,
	ActCaret11,
	ActCaret12,
	ActCaret13,
	ActCaret14,
	ActCaret15,
	ActCaret16,
	ActCaret17
};

void ActCaret(void)
{
	int i;
	int code;

	for (i = 0; i < CARET_MAX; ++i)
	{
		if (gCrt[i].cond & 0x80)
		{
			code = gCrt[i].code;
			gpCaretFuncTbl[code](&gCrt[i]);
		}
	}
}

void PutCaret(int fx, int fy)
{
	int i;

	for (i = 0; i < CARET_MAX; ++i)
	{
		if (gCrt[i].cond & 0x80)
		{
			PutBitmap3(
				&grcGame,
				((gCrt[i].x - gCrt[i].view_left) / 0x200) - (fx / 0x200),
				((gCrt[i].y - gCrt[i].view_top) / 0x200) - (fy / 0x200),
				&gCrt[i].rect,
				SURFACE_ID_CARET);
		}
	}
}

void SetCaret(int x, int y, int code, int dir)
{
	int c;
	for (c = 0; c < CARET_MAX; ++c)
		if (gCrt[c].cond == 0)
			break;

	if (c == CARET_MAX)
		return;

	memset(&gCrt[c], 0, sizeof(CARET));
	gCrt[c].cond = 0x80;
	gCrt[c].code = code;
	gCrt[c].x = x;
	gCrt[c].y = y;
	gCrt[c].view_left = gCaretTable[code].view_left;
	gCrt[c].view_top = gCaretTable[code].view_top;
	gCrt[c].direct = dir;
}

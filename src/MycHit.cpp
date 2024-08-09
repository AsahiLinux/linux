// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "MycHit.h"

#include "WindowsWrapper.h"

#include "Back.h"
#include "Boss.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Game.h"
#include "KeyControl.h"
#include "Map.h"
#include "MyChar.h"
#include "MycParam.h"
#include "NpChar.h"
#include "Sound.h"
#include "TextScr.h"

void ResetMyCharFlag(void)
{
	gMC.flag = 0;
}

static void PutlittleStar(void)
{
	if (!(gMC.cond & 2) && gMC.ym < -0x200)
	{
		PlaySoundObject(3, SOUND_MODE_PLAY);
		SetCaret(gMC.x, gMC.y - gMC.hit.top, CARET_TINY_PARTICLES, DIR_LEFT);
		SetCaret(gMC.x, gMC.y - gMC.hit.top, CARET_TINY_PARTICLES, DIR_LEFT);
	}
}

int JudgeHitMyCharBlock(int x, int y)
{
	int hit = 0;

	// Left wall
	if (gMC.y - gMC.hit.top < (y * 0x10 + 4) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 4) * 0x200
		&& gMC.x - gMC.hit.back < (x * 0x10 + 8) * 0x200
		&& gMC.x - gMC.hit.back > x * 0x10 * 0x200)
	{
		// Clip
		gMC.x = ((x * 0x10 + 8) * 0x200) + gMC.hit.back;

		// Halt momentum
		if (gMC.xm < -0x180)
			gMC.xm = -0x180;
		if (!(gKey & gKeyLeft) && gMC.xm < 0)
			gMC.xm = 0;

		// Set that a left wall was hit
		hit |= 1;
	}

	// Right wall
	if (gMC.y - gMC.hit.top < (y * 0x10 + 4) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 4) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 8) * 0x200
		&& gMC.x + gMC.hit.back < x * 0x10 * 0x200)
	{
		// Clip
		gMC.x = ((x * 0x10 - 8) * 0x200) - gMC.hit.back;

		// Halt momentum
		if (gMC.xm > 0x180)
			gMC.xm = 0x180;
		if (!(gKey & gKeyRight) && gMC.xm > 0)
			gMC.xm = 0;

		// Set that a right wall was hit
		hit |= 4;
	}

	// Ceiling
	if (gMC.x - gMC.hit.back < (x * 0x10 + 5) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 5) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 + 8) * 0x200
		&& gMC.y - gMC.hit.top > y * 0x10 * 0x200)
	{
		// Clip
		gMC.y = ((y * 0x10 + 8) * 0x200) + gMC.hit.top;

		// Halt momentum
		if (!(gMC.cond & 2) && gMC.ym < -0x200)
			PutlittleStar();
		if (gMC.ym < 0)
			gMC.ym = 0;

		// Set that a ceiling was hit
		hit |= 2;
	}

	// Floor
	if (gMC.x - gMC.hit.back < (x * 0x10 + 5) * 0x200
		&& gMC.x + gMC.hit.back > ((x * 0x10 - 5) * 0x200)
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 8) * 0x200
		&& gMC.y + gMC.hit.bottom < y * 0x10 * 0x200)
	{
		// Clip
		gMC.y = ((y * 0x10 - 8) * 0x200) - gMC.hit.bottom;

		// Halt momentum
		if (gMC.ym > 0x400)
			PlaySoundObject(23, SOUND_MODE_PLAY);
		if (gMC.ym > 0)
			gMC.ym = 0;

		// Set that a floor was hit
		hit |= 8;
	}

	return hit;
}

int JudgeHitMyCharTriangleA(int x, int y)
{
	int hit = 0;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 * 0x200) - (gMC.x - x * 0x10 * 0x200) / 2 + 0x800
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800 + gMC.hit.top;

		// Halt momentum
		if (!(gMC.cond & 2) && gMC.ym < -0x200)
			PutlittleStar();
		if (gMC.ym < 0)
			gMC.ym = 0;

		// Set that hit a ceiling
		hit |= 2;
	}

	return hit;
}

int JudgeHitMyCharTriangleB(int x, int y)
{
	int hit = 0;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800 + gMC.hit.top;

		// Halt momentum
		if (!(gMC.cond & 2) && gMC.ym < -0x200)
			PutlittleStar();
		if (gMC.ym < 0)
			gMC.ym = 0;

		// Set that hit a ceiling
		hit |= 2;
	}

	return hit;
}

int JudgeHitMyCharTriangleC(int x, int y)
{
	int hit = 0;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800 + gMC.hit.top;

		// Halt momentum
		if (!(gMC.cond & 2) && gMC.ym < -0x200)
			PutlittleStar();
		if (gMC.ym < 0)
			gMC.ym = 0;

		// Set that hit a ceiling
		hit |= 2;
	}

	return hit;
}

int JudgeHitMyCharTriangleD(int x, int y)
{
	int hit = 0;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800 + gMC.hit.top;

		// Halt momentum
		if (!(gMC.cond & 2) && gMC.ym < -0x200)
			PutlittleStar();
		if (gMC.ym < 0)
			gMC.ym = 0;

		// Set that hit a ceiling
		hit |= 2;
	}

	return hit;
}

int JudgeHitMyCharTriangleE(int x, int y)
{
	int hit = 0;

	hit |= 0x10000;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800
		&& gMC.y - gMC.hit.top < (y * 0x10 + 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800 - gMC.hit.bottom;

		// Halt momentum
		if (gMC.ym > 0x400)
			PlaySoundObject(23, SOUND_MODE_PLAY);
		if (gMC.ym > 0)
			gMC.ym = 0;

		// Set that hit this slope
		hit |= 0x28;
	}

	return hit;
}

int JudgeHitMyCharTriangleF(int x, int y)
{
	int hit = 0;

	hit |= 0x20000;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800
		&& gMC.y - gMC.hit.top < (y * 0x10 + 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) + ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800 - gMC.hit.bottom;

		// Halt momentum
		if (gMC.ym > 0x400)
			PlaySoundObject(23, SOUND_MODE_PLAY);
		if (gMC.ym > 0)
			gMC.ym = 0;

		// Set that hit this slope
		hit |= 0x28;
	}

	return hit;
}

int JudgeHitMyCharTriangleG(int x, int y)
{
	int hit = 0;

	hit |= 0x40000;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800
		&& gMC.y - gMC.hit.top < (y * 0x10 + 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) + 0x800 - gMC.hit.bottom;

		// Halt momentum
		if (gMC.ym > 0x400)
			PlaySoundObject(23, SOUND_MODE_PLAY);
		if (gMC.ym > 0)
			gMC.ym = 0;

		// Set that hit this slope
		hit |= 0x18;
	}

	return hit;
}

int JudgeHitMyCharTriangleH(int x, int y)
{
	int hit = 0;

	hit |= 0x80000;

	if (gMC.x < (x * 0x10 + 8) * 0x200
		&& gMC.x > (x * 0x10 - 8) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800
		&& gMC.y - gMC.hit.top < (y * 0x10 + 8) * 0x200)
	{
		// Clip
		gMC.y = (y * 0x10 * 0x200) - ((gMC.x - x * 0x10 * 0x200) / 2) - 0x800 - gMC.hit.bottom;

		// Halt momentum
		if (gMC.ym > 0x400)
			PlaySoundObject(23, SOUND_MODE_PLAY);
		if (gMC.ym > 0)
			gMC.ym = 0;

		// Set that hit this slope
		hit |= 0x18;
	}

	return hit;
}

int JudgeHitMyCharWater(int x, int y)
{
	int hit = 0;

	if (gMC.x - gMC.hit.back < (x * 0x10 + 5) * 0x200
		&& gMC.x + gMC.hit.back > ((x * 0x10 - 5) * 0x200)
		&& gMC.y - gMC.hit.top < ((y * 0x10 + 5) * 0x200)
		&& gMC.y + gMC.hit.bottom > y * 0x10 * 0x200)
		hit |= 0x100;

	return hit;
}

int JudgeHitMyCharDamage(int x, int y)
{
	int hit = 0;

	if (gMC.x - 0x800 < (x * 0x10 + 4) * 0x200
		&& gMC.x + 0x800 > (x * 0x10 - 4) * 0x200
		&& gMC.y - 0x800 < (y * 0x10 + 3) * 0x200
		&& gMC.y + 0x800 > (y * 0x10 - 3) * 0x200)
		hit |= 0x400;

	return hit;
}

int JudgeHitMyCharDamageW(int x, int y)
{
	int hit = 0;

	if (gMC.x - 0x800 < (x * 0x10 + 4) * 0x200
		&& gMC.x + 0x800 > (x * 0x10 - 4) * 0x200
		&& gMC.y - 0x800 < (y * 0x10 + 3) * 0x200
		&& gMC.y + 0x800 > (y * 0x10 - 3) * 0x200)
		hit |= 0xD00;

	return hit;
}

int JudgeHitMyCharVectLeft(int x, int y)
{
	int hit = 0;
	if (gMC.x - gMC.hit.back < (x * 0x10 + 6) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 6) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 + 6) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 6) * 0x200)
		hit |= 0x1000;

	return hit;
}

int JudgeHitMyCharVectUp(int x, int y)
{
	int hit = 0;
	if (gMC.x - gMC.hit.back < (x * 0x10 + 6) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 6) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 + 6) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 6) * 0x200)
		hit |= 0x2000;

	return hit;
}

int JudgeHitMyCharVectRight(int x, int y)
{
	int hit = 0;
	if (gMC.x - gMC.hit.back < (x * 0x10 + 6) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 6) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 + 6) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 6) * 0x200)
		hit |= 0x4000;

	return hit;
}

int JudgeHitMyCharVectDown(int x, int y)
{
	int hit = 0;
	if (gMC.x - gMC.hit.back < (x * 0x10 + 6) * 0x200
		&& gMC.x + gMC.hit.back > (x * 0x10 - 6) * 0x200
		&& gMC.y - gMC.hit.top < (y * 0x10 + 6) * 0x200
		&& gMC.y + gMC.hit.bottom > (y * 0x10 - 6) * 0x200)
		hit |= 0x8000;

	return hit;
}

void HitMyCharMap(void)
{
	int x, y;
	int i;
	unsigned char atrb[4];

	x = gMC.x / 0x10 / 0x200;
	y = gMC.y / 0x10 / 0x200;

	int offx[4];
	int offy[4];

	offx[0] = 0;
	offx[1] = 1;
	offx[2] = 0;
	offx[3] = 1;

	offy[0] = 0;
	offy[1] = 0;
	offy[2] = 1;
	offy[3] = 1;

	for (i = 0; i < 4; ++i)
	{
		atrb[i] = GetAttribute(x + offx[i], y + offy[i]);

		switch (atrb[i])
		{
			// Block
			case 0x05:
			case 0x41:
			case 0x43:
			case 0x46:
				gMC.flag |= JudgeHitMyCharBlock(x + offx[i], y + offy[i]);
				break;

			// Slopes
			case 0x50:
				gMC.flag |= JudgeHitMyCharTriangleA(x + offx[i], y + offy[i]);
				break;

			case 0x51:
				gMC.flag |= JudgeHitMyCharTriangleB(x + offx[i], y + offy[i]);
				break;

			case 0x52:
				gMC.flag |= JudgeHitMyCharTriangleC(x + offx[i], y + offy[i]);
				break;

			case 0x53:
				gMC.flag |= JudgeHitMyCharTriangleD(x + offx[i], y + offy[i]);
				break;

			case 0x54:
				gMC.flag |= JudgeHitMyCharTriangleE(x + offx[i], y + offy[i]);
				break;

			case 0x55:
				gMC.flag |= JudgeHitMyCharTriangleF(x + offx[i], y + offy[i]);
				break;

			case 0x56:
				gMC.flag |= JudgeHitMyCharTriangleG(x + offx[i], y + offy[i]);
				break;

			case 0x57:
				gMC.flag |= JudgeHitMyCharTriangleH(x + offx[i], y + offy[i]);
				break;

			// Spikes
			case 0x42:
				gMC.flag |= JudgeHitMyCharDamage(x + offx[i], y + offy[i]);
				break;

			// Water spikes
			case 0x62:
				gMC.flag |= JudgeHitMyCharDamageW(x + offx[i], y + offy[i]);
				break;

			// Wind
			case 0x80:
				gMC.flag |= JudgeHitMyCharVectLeft(x + offx[i], y + offy[i]);
				break;

			case 0x81:
				gMC.flag |= JudgeHitMyCharVectUp(x + offx[i], y + offy[i]);
				break;

			case 0x82:
				gMC.flag |= JudgeHitMyCharVectRight(x + offx[i], y + offy[i]);
				break;

			case 0x83:
				gMC.flag |= JudgeHitMyCharVectDown(x + offx[i], y + offy[i]);
				break;

			// Water
			case 0x02:
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			// Water and water blocks (same as the previous case)
			case 0x60:
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x61:
				gMC.flag |= JudgeHitMyCharBlock(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			// Water slopes
			case 0x70:
				gMC.flag |= JudgeHitMyCharTriangleA(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x71:
				gMC.flag |= JudgeHitMyCharTriangleB(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x72:
				gMC.flag |= JudgeHitMyCharTriangleC(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x73:
				gMC.flag |= JudgeHitMyCharTriangleD(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x74:
				gMC.flag |= JudgeHitMyCharTriangleE(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x75:
				gMC.flag |= JudgeHitMyCharTriangleF(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x76:
				gMC.flag |= JudgeHitMyCharTriangleG(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0x77:
				gMC.flag |= JudgeHitMyCharTriangleH(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			// Water current
			case 0xA0:
				gMC.flag |= JudgeHitMyCharVectLeft(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0xA1:
				gMC.flag |= JudgeHitMyCharVectUp(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0xA2:
				gMC.flag |= JudgeHitMyCharVectRight(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;

			case 0xA3:
				gMC.flag |= JudgeHitMyCharVectDown(x + offx[i], y + offy[i]);
				gMC.flag |= JudgeHitMyCharWater(x + offx[i], y + offy[i]);
				break;
		}
	}

	if (gMC.y > gWaterY + (4 * 0x200))
		gMC.flag |= 0x100;
}

int JudgeHitMyCharNPC(NPCHAR *npc)
{
	int hit = 0;

	if (gMC.y - gMC.hit.top < npc->y + npc->hit.bottom - (3 * 0x200)
		&& gMC.y + gMC.hit.bottom > npc->y - npc->hit.top + (3 * 0x200)
		&& gMC.x - gMC.hit.back < npc->x + npc->hit.back
		&& gMC.x - gMC.hit.back > npc->x)
	{
		if (gMC.xm < 0x200)
			gMC.xm += 0x200;
		hit |= 1;
	}

	if (gMC.y - gMC.hit.top < npc->y + npc->hit.bottom - (3 * 0x200)
		&& gMC.y + gMC.hit.bottom > npc->y - npc->hit.top + (3 * 0x200)
		&& gMC.x + gMC.hit.back - 0x200 > npc->x - npc->hit.back
		&& gMC.x + gMC.hit.back - 0x200 < npc->x)
	{
		if (gMC.xm > -0x200)
			gMC.xm -= 0x200;
		hit |= 4;
	}

	if (gMC.x - gMC.hit.back < npc->x + npc->hit.back - (3 * 0x200)
		&& gMC.x + gMC.hit.back > npc->x - npc->hit.back + (3 * 0x200)
		&& gMC.y - gMC.hit.top < npc->y + npc->hit.bottom
		&& gMC.y - gMC.hit.top > npc->y)
	{
		if (gMC.ym < 0)
			gMC.ym = 0;
		hit |= 2;
	}

	if (gMC.x - gMC.hit.back < npc->x + npc->hit.back - (3 * 0x200)
		&& gMC.x + gMC.hit.back > npc->x - npc->hit.back + (3 * 0x200)
		&& gMC.y + gMC.hit.bottom > npc->y - npc->hit.top
		&& gMC.hit.bottom + gMC.y < npc->y + (3 * 0x200))
	{
		if (npc->bits & NPC_BOUNCY)
		{
			gMC.ym = npc->ym - 0x200;
			hit |= 8;
		}
		else if (!(gMC.flag & 8) && gMC.ym > npc->ym)
		{
			gMC.y = npc->y - npc->hit.top - gMC.hit.bottom + 0x200;
			gMC.ym = npc->ym;
			gMC.x += npc->xm;
			hit |= 8;
		}
	}

	return hit;
}

unsigned char JudgeHitMyCharNPC3(NPCHAR *npc)
{
	if (npc->direct == 0)
	{
		if (gMC.x + (2 * 0x200) > npc->x - npc->hit.front
			&& gMC.x - (2 * 0x200) < npc->x + npc->hit.back
			&& gMC.y + (2 * 0x200) > npc->y - npc->hit.top
			&& gMC.y - (2 * 0x200) < npc->y + npc->hit.bottom)
			return 1;
	}
	else
	{
		if (gMC.x + (2 * 0x200) > npc->x - npc->hit.back
			&& gMC.x - (2 * 0x200) < npc->x + npc->hit.front
			&& gMC.y + (2 * 0x200) > npc->y - npc->hit.top
			&& gMC.y - (2 * 0x200) < npc->y + npc->hit.bottom)
			return 1;
	}

	return 0;
}

int JudgeHitMyCharNPC4(NPCHAR *npc)
{
	// TODO: comment this
	int hit = 0;

	float fy1;
	float fx1;
	float fx2;
	float fy2;

	if (npc->x > gMC.x)
		fx1 = (float)(npc->x - gMC.x);
	else
		fx1 = (float)(gMC.x - npc->x);

	if (npc->y > gMC.y)
		fy1 = (float)(npc->y - gMC.y);
	else
		fy1 = (float)(gMC.y - npc->y);

	fx2 = (float)npc->hit.back;
	fy2 = (float)npc->hit.top;

	if (fx1 == 0.0f)
		fx1 = 1.0f;
	if (fx2 == 0.0f)
		fx2 = 1.0f;

	if (fy1 / fx1 > fy2 / fx2)
	{
		if (gMC.x - gMC.hit.back < npc->x + npc->hit.back && gMC.x + gMC.hit.back > npc->x - npc->hit.back)
		{
			if (gMC.y - gMC.hit.top < npc->y + npc->hit.bottom && gMC.y - gMC.hit.top > npc->y)
			{
				if (gMC.ym < npc->ym)
				{
					gMC.y = npc->y + npc->hit.bottom + gMC.hit.top + 0x200;
					gMC.ym = npc->ym;
				}
				else
				{
					if (gMC.ym < 0)
						gMC.ym = 0;
				}

				hit |= 2;
			}

			if (gMC.y + gMC.hit.bottom > npc->y - npc->hit.top && gMC.hit.bottom + gMC.y < npc->y + (3 * 0x200))
			{
				if (gMC.ym - npc->ym > 2 * 0x200)
					PlaySoundObject(23, SOUND_MODE_PLAY);

				if (gMC.unit == 1)
				{
					gMC.y = npc->y - npc->hit.top - gMC.hit.bottom + 0x200;
					hit |= 8;
				}
				else if (npc->bits & NPC_BOUNCY)
				{
					gMC.ym = npc->ym - 0x200;
					hit |= 8;
				}
				else if (!(gMC.flag & 8) && gMC.ym > npc->ym)
				{
					gMC.y = npc->y - npc->hit.top - gMC.hit.bottom + 0x200;
					gMC.ym = npc->ym;
					gMC.x += npc->xm;
					hit |= 8;
				}
			}
		}
	}
	else
	{
		if (gMC.y - gMC.hit.top < npc->y + npc->hit.bottom && gMC.y + gMC.hit.bottom > npc->y - npc->hit.top)
		{
			if (gMC.x - gMC.hit.back < npc->x + npc->hit.back && gMC.x - gMC.hit.back > npc->x)
			{
				if (gMC.xm < npc->xm)
					gMC.xm = npc->xm;

				gMC.x = npc->x + npc->hit.back + gMC.hit.back;

				hit |= 1;
			}

			if (gMC.x + gMC.hit.back > npc->x - npc->hit.back && gMC.hit.back + gMC.x < npc->x)
			{
				if (gMC.xm > npc->xm)
					gMC.xm = npc->xm;

				gMC.x = npc->x - npc->hit.back - gMC.hit.back;

				hit |= 4;
			}
		}
	}

	return hit;
}

void HitMyCharNpChar(void)
{
	int i;
	int hit = 0;

	if (!(gMC.cond & 0x80) || gMC.cond & 2)
		return;

	for (i = 0; i < NPC_MAX; ++i)
	{
		if (!(gNPC[i].cond & 0x80))
			continue;

		hit = 0;

		if (gNPC[i].bits & NPC_SOLID_SOFT)
		{
			hit = JudgeHitMyCharNPC(&gNPC[i]);
			gMC.flag |= hit;
		}
		else if (gNPC[i].bits & NPC_SOLID_HARD)
		{
			hit = JudgeHitMyCharNPC4(&gNPC[i]);
			gMC.flag |= hit;
		}
		else
		{
			hit = JudgeHitMyCharNPC3(&gNPC[i]);
		}

		// Special NPCs (pickups)
		if (hit != 0 && gNPC[i].code_char == 1)
		{
			PlaySoundObject(14, SOUND_MODE_PLAY);
			AddExpMyChar(gNPC[i].exp);
			gNPC[i].cond = 0;
		}

		if (hit != 0 && gNPC[i].code_char == 86)
		{
			PlaySoundObject(42, SOUND_MODE_PLAY);
			AddBulletMyChar(gNPC[i].code_event, gNPC[i].exp);
			gNPC[i].cond = 0;
		}

		if (hit != 0 && gNPC[i].code_char == 87)
		{
			PlaySoundObject(20, SOUND_MODE_PLAY);
			AddLifeMyChar(gNPC[i].exp);
			gNPC[i].cond = 0;
		}

		// Run event on contact
		if (!(g_GameFlags & 4) && hit != 0 && gNPC[i].bits & NPC_EVENT_WHEN_TOUCHED)
			StartTextScript(gNPC[i].code_event);

		// NPC damage
		if (g_GameFlags & 2 && !(gNPC[i].bits & NPC_INTERACTABLE))
		{
			if (gNPC[i].bits & NPC_REAR_AND_TOP_DONT_HURT)
			{
				if (hit & 4 && gNPC[i].xm < 0)
					DamageMyChar(gNPC[i].damage);
				if (hit & 1 && gNPC[i].xm > 0)
					DamageMyChar(gNPC[i].damage);
				if (hit & 8 && gNPC[i].ym < 0)
					DamageMyChar(gNPC[i].damage);
				if (hit & 2 && gNPC[i].ym > 0)
					DamageMyChar(gNPC[i].damage);
			}
			else if (hit != 0 && gNPC[i].damage && !(g_GameFlags & 4))
			{
				DamageMyChar(gNPC[i].damage);
			}
		}

		// Interaction
		if (!(g_GameFlags & 4) && hit != 0 && gMC.cond & 1 && gNPC[i].bits & NPC_INTERACTABLE)
		{
			StartTextScript(gNPC[i].code_event);
			gMC.xm = 0;
			gMC.ques = FALSE;
		}
	}

	// Create question mark when NPC hasn't been interacted with
	if (gMC.ques)
		SetCaret(gMC.x, gMC.y, CARET_QUESTION_MARK, DIR_LEFT);
}

void HitMyCharBoss(void)
{
	int b;
	int hit = 0;

	if (!(gMC.cond & 0x80) || gMC.cond & 2)
		return;

	for (b = 0; b < BOSS_MAX; ++b)
	{
		if (!(gBoss[b].cond & 0x80))
			continue;

		hit = 0;

		if (gBoss[b].bits & NPC_SOLID_SOFT)
		{
			hit = JudgeHitMyCharNPC(&gBoss[b]);
			gMC.flag |= hit;
		}
		else if (gBoss[b].bits & NPC_SOLID_HARD)
		{
			hit = JudgeHitMyCharNPC4(&gBoss[b]);
			gMC.flag |= hit;
		}
		else
		{
			hit = JudgeHitMyCharNPC3(&gBoss[b]);
		}

		if (!(g_GameFlags & 4) && hit != 0 && gBoss[b].bits & NPC_EVENT_WHEN_TOUCHED)
		{
			StartTextScript(gBoss[b].code_event);
			gMC.ques = FALSE;
		}

		if (gBoss[b].bits & NPC_REAR_AND_TOP_DONT_HURT)
		{
			if (hit & 4 && gBoss[b].xm < 0)
				DamageMyChar(gBoss[b].damage);
			if (hit & 1 && gBoss[b].xm > 0)
				DamageMyChar(gBoss[b].damage);
		}
		else if (hit != 0 && gBoss[b].damage != 0 && !(g_GameFlags & 4))
		{
			DamageMyChar(gBoss[b].damage);
		}

		if (!(g_GameFlags & 4) && hit != 0 && gMC.cond & 1 && gBoss[b].bits & NPC_INTERACTABLE)
		{
			StartTextScript(gBoss[b].code_event);
			gMC.xm = 0;
			gMC.ques = FALSE;
		}
	}

	if (gMC.ques)
		SetCaret(gMC.x, gMC.y, CARET_QUESTION_MARK, DIR_LEFT);
}

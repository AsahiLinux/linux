// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "MycParam.h"

#include <stddef.h>
#include <stdio.h>

#include "WindowsWrapper.h"

#include "ArmsItem.h"
#include "CommonDefines.h"
#include "Caret.h"
#include "Draw.h"
#include "Game.h"
#include "Main.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "TextScr.h"
#include "ValueView.h"

ARMS_LEVEL gArmsLevelTable[14] =
{
	{{0,  0,  100}},
	{{30, 40, 16}},
	{{10, 20, 10}},
	{{10, 20, 20}},
	{{30, 40, 10}},
	{{10, 20, 10}},
	{{10, 20, 30}},
	{{10, 20, 5}},
	{{10, 20, 100}},
	{{30, 60, 0}},
	{{30, 60, 10}},
	{{10, 20, 100}},
	{{1,  1,  1}},
	{{40, 60, 200}}
};

void AddExpMyChar(int x)
{
	int lv = gArmsData[gSelectedArms].level - 1;
	int arms_code = gArmsData[gSelectedArms].code;

	gArmsData[gSelectedArms].exp += x;

	if (lv == 2)
	{
		if (gArmsData[gSelectedArms].exp >= gArmsLevelTable[arms_code].exp[lv])
		{
			gArmsData[gSelectedArms].exp = gArmsLevelTable[arms_code].exp[lv];

			if (gMC.equip & EQUIP_WHIMSICAL_STAR)
			{
				if (gMC.star < 3)
					++gMC.star;
			}
		}
	}
	else
	{
		for (; lv < 2; ++lv)
		{
			if (gArmsData[gSelectedArms].exp >= gArmsLevelTable[arms_code].exp[lv])
			{
				++gArmsData[gSelectedArms].level;
				gArmsData[gSelectedArms].exp = 0;

				if (gArmsData[gSelectedArms].code != 13)
				{
					PlaySoundObject(27, SOUND_MODE_PLAY);
					SetCaret(gMC.x, gMC.y, CARET_LEVEL_UP, DIR_LEFT);
				}
			}
		}
	}

	if (gArmsData[gSelectedArms].code != 13)
	{
		gMC.exp_count += x;
		gMC.exp_wait = 30;
	}
	else
	{
		gMC.exp_wait = 10;
	}
}

void ZeroExpMyChar(void)
{
	gArmsData[gSelectedArms].level = 1;
	gArmsData[gSelectedArms].exp = 0;
}

BOOL IsMaxExpMyChar(void)
{
	int arms_code;

	if (gArmsData[gSelectedArms].level == 3)
	{
		arms_code = gArmsData[gSelectedArms].code;

		if (gArmsData[gSelectedArms].exp >= gArmsLevelTable[arms_code].exp[2])
			return TRUE;
	}

	return FALSE;
}

void DamageMyChar(int damage)
{
#ifdef FIX_BUGS
	if (!(g_GameFlags & 2))
#else
	// I'm preeeetty sure this is a typo. The Linux port optimised this entire check out.
	if (!(g_GameFlags | 2))
#endif
		return;

	if (gMC.shock)
		return;

	// Damage player
	PlaySoundObject(16, SOUND_MODE_PLAY);
	gMC.cond &= ~1;
	gMC.shock = 128;

	if (gMC.unit == 1)
	{
		// Another weird case where there *has* to be an empty 'if' here to produce the same assembly.
		// Chances are there used to be some commented-out code here.
	}
	else
	{
		gMC.ym = -0x400;
	}

	gMC.life -= (short)damage;

	// Lose a whimsical star
	if (gMC.equip & EQUIP_WHIMSICAL_STAR && gMC.star > 0)
		gMC.star = (short)gMC.star - 1;	// For some reason, this does a cast to short. Might not be accurate to the original source code (possibly, Pixel was just being careful about int size/conversion, or this is from some weird macro)

	// Lose experience
	if (gMC.equip & EQUIP_ARMS_BARRIER)
		gArmsData[gSelectedArms].exp -= damage;
	else
		gArmsData[gSelectedArms].exp -= damage * 2;

	while (gArmsData[gSelectedArms].exp < 0)
	{
		if (gArmsData[gSelectedArms].level > 1)
		{
			--gArmsData[gSelectedArms].level;

			int lv = gArmsData[gSelectedArms].level - 1;
			int arms_code = gArmsData[gSelectedArms].code;

			gArmsData[gSelectedArms].exp = gArmsLevelTable[arms_code].exp[lv] + gArmsData[gSelectedArms].exp;

			if (gMC.life > 0 && gArmsData[gSelectedArms].code != 13)
				SetCaret(gMC.x, gMC.y, CARET_LEVEL_UP, DIR_RIGHT);
		}
		else
		{
			gArmsData[gSelectedArms].exp = 0;
		}
	}

	// Tell player how much damage was taken
	SetValueView(&gMC.x, &gMC.y, -damage);

	// Death
	if (gMC.life <= 0)
	{
		PlaySoundObject(17, SOUND_MODE_PLAY);
		gMC.cond = 0;
		SetDestroyNpChar(gMC.x, gMC.y, 10 * 0x200, 0x40);
		StartTextScript(40);
	}
}

void ZeroArmsEnergy_All(void)
{
	int a;

	for (a = 0; a < ARMS_MAX; a++)
	{
		gArmsData[a].level = 1;
		gArmsData[a].exp = 0;
	}
}

void AddBulletMyChar(int no, int val)
{
	int a;

	// Missile Launcher
	a = 0;
	while (a < ARMS_MAX && gArmsData[a].code != 5)
		++a;

	if (a == ARMS_MAX)
	{
		// Super Missile Launcher
		a = 0;
		while (a < ARMS_MAX && gArmsData[a].code != 10)
			++a;

		if (a == ARMS_MAX)
			return;
	}

	gArmsData[a].num += val;
	if (gArmsData[a].num > gArmsData[a].max_num)
		gArmsData[a].num = gArmsData[a].max_num;
}

void AddLifeMyChar(int x)
{
	gMC.life += (short)x;
	if (gMC.life > gMC.max_life)
		gMC.life = gMC.max_life;
	gMC.lifeBr = gMC.life;
}

void AddMaxLifeMyChar(int val)
{
	gMC.max_life += (short)val;
	if (gMC.max_life > 232)
		gMC.max_life = 232;
	gMC.life += (short)val;
	gMC.lifeBr = gMC.life;
}

void PutArmsEnergy(BOOL flash)
{
	static unsigned char add_flash;

	RECT rcPer = {72, 48, 80, 56};
	RECT rcLv = {80, 80, 96, 88};
	RECT rcView = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};
	RECT rcNone = {80, 48, 96, 56};

	if (gArmsEnergyX > 16)
		gArmsEnergyX -= 2;
	if (gArmsEnergyX < 16)
		gArmsEnergyX += 2;

	// Draw max ammo
	if (gArmsData[gSelectedArms].max_num)
	{
		PutNumber4(gArmsEnergyX + 32, 16, gArmsData[gSelectedArms].num, FALSE);
		PutNumber4(gArmsEnergyX + 32, 24, gArmsData[gSelectedArms].max_num, FALSE);
	}
	else
	{
		PutBitmap3(&rcView, gArmsEnergyX + 48, 16, &rcNone, SURFACE_ID_TEXT_BOX);
		PutBitmap3(&rcView, gArmsEnergyX + 48, 24, &rcNone, SURFACE_ID_TEXT_BOX);
	}

	// Draw experience and ammo
	if (flash == TRUE && (gMC.shock / 2) % 2)
		return;

	PutBitmap3(&rcView, gArmsEnergyX + 32, 24, &rcPer, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&rcView, gArmsEnergyX, 32, &rcLv, SURFACE_ID_TEXT_BOX);
	PutNumber4(gArmsEnergyX - 8, 32, gArmsData[gSelectedArms].level, FALSE);

	RECT rcExpBox = {0, 72, 40, 80};
	RECT rcExpVal = {0, 80, 0, 88};
	RECT rcExpMax = {40, 72, 80, 80};
	RECT rcExpFlash = {40, 80, 80, 88};

	int lv = gArmsData[gSelectedArms].level - 1;

#ifdef FIX_MAJOR_BUGS
	// When the player has no weapons, the default level is 0, which becomes -1.
	// Catch it, and set it to 0 instead, so the following array-accesses aren't
	// out-of-bounds.
	if (lv < 0)
		lv = 0;
#endif

	int arms_code = gArmsData[gSelectedArms].code;
	int exp_now = gArmsData[gSelectedArms].exp;
	int exp_next = gArmsLevelTable[arms_code].exp[lv];

	PutBitmap3(&rcView, gArmsEnergyX + 24, 32, &rcExpBox, SURFACE_ID_TEXT_BOX);

	if (lv == 2 && gArmsData[gSelectedArms].exp == gArmsLevelTable[arms_code].exp[lv])
	{
		PutBitmap3(&rcView, gArmsEnergyX + 24, 32, &rcExpMax, SURFACE_ID_TEXT_BOX);
	}
	else
	{
		if (exp_next != 0)
			rcExpVal.right += (exp_now * 40) / exp_next;
		else
			rcExpVal.right = 0;

		PutBitmap3(&rcView, gArmsEnergyX + 24, 32, &rcExpVal, SURFACE_ID_TEXT_BOX);
	}

	if (gMC.exp_wait && ((add_flash++ / 2) % 2))
		PutBitmap3(&rcView, gArmsEnergyX + 24, 32, &rcExpFlash, SURFACE_ID_TEXT_BOX);
}

void PutActiveArmsList(void)
{
	int x;
	int a;
	RECT rect = {0, 0, 0, 16};

	int arms_num = 0;
	while (gArmsData[arms_num].code != 0)
		++arms_num;

	if (arms_num == 0)
		return;

	for (a = 0; a < arms_num; ++a)
	{
		// Get X position to draw at
		x = ((a - gSelectedArms) * 16) + gArmsEnergyX;

		if (x < 8)
			x += 48 + (arms_num * 16);
		else if (x >= 24)
			x += 48;

		if (x >= 72 + ((arms_num - 1) * 16))
			x -= 48 + (arms_num * 16);
		if (x < 72 && x >= 24)
			x -= 48;

		// Draw icon
		rect.left = gArmsData[a].code * 16;
		rect.right = rect.left + 16;
		PutBitmap3(&grcGame, x, 16, &rect, SURFACE_ID_ARMS_IMAGE);
	}
}

void PutMyLife(BOOL flash)
{
	RECT rcCase = {0, 40, 232, 48};
	RECT rcLife = {0, 24, 232, 32};
	RECT rcBr = {0, 32, 232, 40};

	if (flash == TRUE && gMC.shock / 2 % 2)
		return;

	if (gMC.lifeBr < gMC.life)
		gMC.lifeBr = gMC.life;

	if (gMC.lifeBr > gMC.life)
	{
		if (++gMC.lifeBr_count > 30)
			--gMC.lifeBr;
	}
	else
	{
		gMC.lifeBr_count = 0;
	}

	// Draw bar
	rcCase.right = 64;
	rcLife.right = ((gMC.life * 40) / gMC.max_life) - 1;
	rcBr.right = ((gMC.lifeBr * 40) / gMC.max_life) - 1;

	PutBitmap3(&grcGame, 16, 40, &rcCase, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, 40, 40, &rcBr, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, 40, 40, &rcLife, SURFACE_ID_TEXT_BOX);
	PutNumber4(8, 40, gMC.lifeBr, FALSE);
}

void PutMyAir(int x, int y)
{
	RECT rcAir[2] = {
		{112, 72, 144, 80},
		{112, 80, 144, 88},
	};

	if (gMC.equip & EQUIP_AIR_TANK)
		return;

	if (gMC.air_get != 0)
	{
		// Draw how much air is left
		if (gMC.air_get % 6 < 4)
			PutNumber4(x + 32, y, gMC.air / 10, FALSE);

		// Draw "AIR" text
		if (gMC.air % 30 > 10)
			PutBitmap3(&grcGame, x, y, &rcAir[0], SURFACE_ID_TEXT_BOX);
		else
			PutBitmap3(&grcGame, x, y, &rcAir[1], SURFACE_ID_TEXT_BOX);
	}
}

static int time_count;

void PutTimeCounter(int x, int y)
{
	RECT rcTime[3] = {
		{112, 104, 120, 112},
		{120, 104, 128, 112},
		{128, 104, 160, 112},
	};

	if (gMC.equip & EQUIP_NIKUMARU_COUNTER)
	{
		// Draw clock and increase time
		if (g_GameFlags & 2)
		{
			if (time_count < 100 * 60 * 50)	// 100 minutes
				++time_count;

			if (time_count % 30 > 10)
				PutBitmap3(&grcGame, x, y, &rcTime[0], SURFACE_ID_TEXT_BOX);
			else
				PutBitmap3(&grcGame, x, y, &rcTime[1], SURFACE_ID_TEXT_BOX);
		}
		else
		{
			PutBitmap3(&grcGame, x, y, &rcTime[0], SURFACE_ID_TEXT_BOX);
		}

		// Draw time
		PutNumber4(x,		y, time_count / (60 * 50),	FALSE);
		PutNumber4(x + 20,	y, time_count / 50 % 60,	TRUE);
		PutNumber4(x + 32,	y, time_count / 5 % 10,		FALSE);
		PutBitmap3(&grcGame, x + 30, y, &rcTime[2], SURFACE_ID_TEXT_BOX);
	}
	else
	{
		time_count = 0;
	}
}

BOOL SaveTimeCounter(void)
{
	int i;
	unsigned char *p;
	REC rec;
	FILE *fp;
	char path[MAX_PATH];

	// Quit if player doesn't have the Nikumaru Counter
	if (!(gMC.equip & EQUIP_NIKUMARU_COUNTER))
		return TRUE;

	// Get last time
	sprintf(path, "%s\\290.rec", gModulePath);

	fp = fopen(path, "rb");
	if (fp != NULL)
	{
		// Read data
		fread(&rec, sizeof(REC), 1, fp);
		fclose(fp);

		p = (unsigned char*)&rec.counter[0];
		p[0] -= rec.random[0];
		p[1] -= rec.random[0];
		p[2] -= rec.random[0];
		p[3] -= rec.random[0] / 2;
		// If this is faster than our new time, quit
		if (rec.counter[0] < time_count)
			return TRUE;
	}

	// Save new time
	for (i = 0; i < 4; ++i)
	{
		rec.counter[i] = time_count;
		rec.random[i] = Random(0, 250) + i;

		p = (unsigned char*)&rec.counter[i];
		p[0] += rec.random[i];
		p[1] += rec.random[i];
		p[2] += rec.random[i];
		p[3] += rec.random[i] / 2;
	}

	fp = fopen(path, "wb");
	if (fp == NULL)
		return FALSE;

	fwrite(&rec, sizeof(REC), 1, fp);

	fclose(fp);
	return TRUE;
}

int LoadTimeCounter(void)
{
	int i;
	unsigned char *p;
	REC rec;
	FILE *fp;
	char path[MAX_PATH];

	// Open file
	sprintf(path, "%s\\290.rec", gModulePath);

	fp = fopen(path, "rb");
	if (fp == NULL)
		return 0;

	// Read data
	fread(&rec, sizeof(REC), 1, fp);
	fclose(fp);

	// Decode from checksum
	for (i = 0; i < 4; ++i)
	{
		p = (unsigned char*)&rec.counter[i];
		p[0] -= rec.random[i];
		p[1] -= rec.random[i];
		p[2] -= rec.random[i];
		p[3] -= rec.random[i] / 2;
	}

	// Verify checksum's result
	if (rec.counter[0] != rec.counter[1] || rec.counter[0] != rec.counter[2])
	{
		time_count = 0;
		return 0;
	}

	time_count = rec.counter[0];
	return time_count;
}

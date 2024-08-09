// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "NpcAct.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Bullet.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Frame.h"
#include "Flash.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Toroko (frenzied)
void ActNpc140(NPCHAR *npc)
{
	int i;

	RECT rcLeft[14] = {
		{0, 0, 32, 32},
		{32, 0, 64, 32},
		{64, 0, 96, 32},
		{96, 0, 128, 32},
		{128, 0, 160, 32},
		{160, 0, 192, 32},
		{192, 0, 224, 32},
		{224, 0, 256, 32},
		{0, 64, 32, 96},
		{32, 64, 64, 96},
		{64, 64, 96, 96},
		{96, 64, 128, 96},
		{128, 64, 160, 96},
		{0, 0, 0, 0},
	};

	RECT rcRight[14] = {
		{0, 32, 32, 64},
		{32, 32, 64, 64},
		{64, 32, 96, 64},
		{96, 32, 128, 64},
		{128, 32, 160, 64},
		{160, 32, 192, 64},
		{192, 32, 224, 64},
		{224, 32, 256, 64},
		{0, 96, 32, 128},
		{32, 96, 64, 128},
		{64, 96, 96, 128},
		{96, 96, 128, 128},
		{128, 96, 160, 128},
		{0, 0, 0, 0},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 9;
			npc->act_wait = 0;
			npc->bits &= ~NPC_INTERACTABLE;
			// Fallthrough
		case 1:
			if (++npc->act_wait > 50)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 8;
			}

			break;

		case 2:
			if (++npc->ani_no > 10)
				npc->ani_no = 9;

			if (++npc->act_wait > 50)
			{
				npc->act_no = 3;
				npc->act_wait = 0;
				npc->ani_no = 0;
			}

			break;

		case 3:
			if (++npc->act_wait > 50)
			{
				npc->act_no = 10;
				npc->bits |= NPC_SHOOTABLE;
			}

			break;

		case 10:
			npc->bits = npc->bits;	// Chances are this line isn't accurate to the original source code, but it produces the same assembly
			npc->act_no = 11;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->act_wait = Random(20, 130);
			npc->xm = 0;
			// Fallthrough
		case 11:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (CountArmsBullet(6) || CountArmsBullet(3) > 3)
				npc->act_no = 20;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				if (Random(0, 99) % 2)
					npc->act_no = 20;
				else
					npc->act_no = 50;
			}

			break;

		case 20:
			npc->act_no = 21;
			npc->ani_no = 2;
			npc->act_wait = 0;
			// Fallthrough
		case 21:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 22;
				npc->act_wait = 0;
				npc->ani_no = 3;
				npc->ym = -0x5FF;

				if (npc->direct == 0)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;
			}

			break;

		case 22:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 23;
				npc->act_wait = 0;
				npc->ani_no = 6;
				SetNpChar(141, 0, 0, 0, 0, 0, npc, 0);
			}

			break;

		case 23:
			if (++npc->act_wait > 30)
			{
				npc->act_no = 24;
				npc->act_wait = 0;
				npc->ani_no = 7;
			}

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			break;

		case 24:
			if (++npc->act_wait > 3)
			{
				npc->act_no = 25;
				npc->ani_no = 3;
			}

			break;

		case 25:
			if (npc->flag & 8)
			{
				npc->act_no = 26;
				npc->act_wait = 0;
				npc->ani_no = 2;
				PlaySoundObject(26, SOUND_MODE_PLAY);
				SetQuake(20);
			}

			break;

		case 26:
			npc->xm = (npc->xm * 8) / 9;

			if (++npc->act_wait > 20)
			{
				npc->act_no = 10;
				npc->ani_no = 0;
			}

			break;

		case 50:
			npc->act_no = 51;
			npc->act_wait = 0;
			npc->ani_no = 4;
			SetNpChar(141, 0, 0, 0, 0, 0, npc, 0);
			// Fallthrough
		case 51:
			if (++npc->act_wait > 30)
			{
				npc->act_no = 52;
				npc->act_wait = 0;
				npc->ani_no = 5;
			}

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			break;

		case 52:
			if (++npc->act_wait > 3)
			{
				npc->act_no = 10;
				npc->ani_no = 0;
			}

			break;

		case 100:
			npc->ani_no = 3;
			npc->act_no = 101;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->damage = 0;

			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

			break;

		case 101:
			if (npc->flag & 8)
			{
				npc->act_no = 102;
				npc->act_wait = 0;
				npc->ani_no = 2;
				PlaySoundObject(26, SOUND_MODE_PLAY);
				SetQuake(20);
			}

			break;

		case 102:
			npc->xm = (npc->xm * 8) / 9;

			if (++npc->act_wait > 50)
			{
				npc->act_no = 103;
				npc->act_wait = 0;
				npc->ani_no = 10;
			}

			break;

		case 103:
			if (++npc->act_wait > 50)
			{
				npc->ani_no = 9;
				npc->act_no = 104;
				npc->act_wait = 0;
			}

			break;

		case 104:
			if (++npc->ani_no > 10)
				npc->ani_no = 9;

			if (++npc->act_wait > 100)
			{
				npc->act_wait = 0;
				npc->ani_no = 9;
				npc->act_no = 105;
			}

			break;

		case 105:
			if (++npc->act_wait > 50)
			{
				npc->ani_wait = 0;
				npc->act_no = 106;
				npc->ani_no = 11;
			}

			break;

		case 106:
			if (++npc->ani_wait > 50)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 12)
				npc->ani_no = 12;

			break;

		case 140:
			npc->act_no = 141;
			npc->act_wait = 0;
			npc->ani_no = 12;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 141:
			if (++npc->ani_no > 13)
				npc->ani_no = 12;

			if (++npc->act_wait > 100)
			{
				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				npc->cond = 0;
			}

			break;
	}

	if (npc->act_no > 100 && npc->act_no < 105 && npc->act_wait % 9 == 0)
		SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

	npc->ym += 0x20;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;
	if (npc->ym < -0x5FF)
		npc->ym = -0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Toroko block projectile
void ActNpc141(NPCHAR *npc)
{
	int i;
	unsigned char deg;

	RECT rect[2] = {
		{288, 32, 304, 48},
		{304, 32, 320, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = 0;
			// Fallthrough
		case 1:
			if (npc->pNpc->direct == 0)
				npc->x = npc->pNpc->x + (10 * 0x200);
			else
				npc->x = npc->pNpc->x - (10 * 0x200);

			npc->y = npc->pNpc->y - (8 * 0x200);

			if (npc->pNpc->act_no == 24 || npc->pNpc->act_no == 52)
			{
				npc->act_no = 10;

				if (npc->pNpc->direct == 0)
					npc->x = npc->pNpc->x - (16 * 0x200);
				else
					npc->x = npc->pNpc->x + (16 * 0x200);

				npc->y = npc->pNpc->y;

				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				npc->ym = GetSin(deg) * 4;
				npc->xm = GetCos(deg) * 4;

				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			break;

		case 10:
			if (npc->flag & 0xF)
			{
				npc->act_no = 20;
				npc->act_wait = 0;
				SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
				PlaySoundObject(12, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x, npc->y, Random(-0x200, 0x200), Random(-0x200, 0x200), 0, NULL, 0x100);

				break;
			}

			npc->x += npc->xm;
			npc->y += npc->ym;

			break;

		case 20:
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (++npc->act_wait > 4)
			{
				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x, npc->y, Random(-0x200, 0x200), Random(-0x200, 0x200), 0, NULL, 0x100);

				npc->code_char = 142;
				npc->ani_no = 0;
				npc->act_no = 20;
				npc->xm = 0;
				npc->bits &= ~NPC_INVULNERABLE;
				npc->bits |= NPC_SHOOTABLE;
				npc->damage = 1;
			}

			break;
	}

	if (++npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Flower Cub
void ActNpc142(NPCHAR *npc)
{
	RECT rect[5] = {
		{0, 128, 16, 144},
		{16, 128, 32, 144},
		{32, 128, 48, 144},
		{48, 128, 64, 144},
		{64, 128, 80, 144},
	};

	switch (npc->act_no)
	{
		case 10:
			npc->act_no = 11;
			npc->ani_no = 0;
			npc->act_wait = 0;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 30)
			{
				npc->act_no = 12;
				npc->ani_no = 1;
				npc->ani_wait = 0;
			}

			break;

		case 12:
			if (++npc->ani_wait > 8)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no == 3)
			{
				npc->act_no = 20;
				npc->ym = -0x200;

				if (gMC.x < npc->x)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;
			}

			break;

		case 20:
			if (npc->ym > -0x80)
				npc->ani_no = 4;
			else
				npc->ani_no = 3;

			if (npc->flag & 8)
			{
				npc->ani_no = 2;
				npc->act_no = 21;
				npc->act_wait = 0;
				npc->xm = 0;
				PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			break;

		case 21:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 10;
				npc->ani_no = 0;
			}

			break;
	}

	npc->ym += 0x40;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;
	if (npc->ym < -0x5FF)
		npc->ym = -0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rect[npc->ani_no];
}

// Jenka (collapsed)
void ActNpc143(NPCHAR *npc)
{
	RECT rcLeft[1] = {
		{208, 32, 224, 48}
	};

	RECT rcRight[1] = {
		{208, 48, 224, 64}
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Toroko (teleporting in)
void ActNpc144(NPCHAR *npc)
{
	RECT rcLeft[5] = {
		{0, 64, 16, 80},
		{16, 64, 32, 80},
		{32, 64, 48, 80},
		{16, 64, 32, 80},
		{128, 64, 144, 80},
	};

	RECT rcRight[5] = {
		{0, 80, 16, 96},
		{16, 80, 32, 96},
		{32, 80, 48, 96},
		{16, 80, 32, 96},
		{128, 80, 144, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->tgt_x = npc->x;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 1:
			if (++npc->act_wait == 64)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			if (npc->flag & 8)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
				npc->ani_no = 4;
				PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 11:
			if (Random(0, 120) == 10)
			{
				npc->act_no = 12;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 12:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 11;
				npc->ani_no = 0;
			}

			break;
	}

	if (npc->act_no > 1)
	{
		npc->ym += 0x20;
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->y += npc->ym;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->act_no == 1)
	{
		npc->rect.bottom = npc->rect.top + npc->act_wait / 4;

		if (npc->act_wait / 2 % 2)
			npc->x = npc->tgt_x;
		else
			npc->x = npc->tgt_x + (1 * 0x200);
	}
}

// King's sword
void ActNpc145(NPCHAR *npc)
{
	RECT rcLeft[1] = {
		{96, 32, 112, 48}
	};

	RECT rcRight[1] = {
		{112, 32, 128, 48}
	};

	switch (npc->act_no)
	{
		case 0:
			if (npc->pNpc->count2 == 0)
			{
				if (npc->pNpc->direct == 0)
					npc->direct = 0;
				else
					npc->direct = 2;
			}
			else
			{
				if (npc->pNpc->direct == 0)
					npc->direct = 2;
				else
					npc->direct = 0;
			}

			if (npc->direct == 0)
				npc->x = npc->pNpc->x - (10 * 0x200);
			else
				npc->x = npc->pNpc->x + (10 * 0x200);

			npc->y = npc->pNpc->y;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Lightning
void ActNpc146(NPCHAR *npc)
{
	RECT rect[5] = {
		{0, 0, 0, 0},
		{256, 0, 272, 240},
		{272, 0, 288, 240},
		{288, 0, 304, 240},
		{304, 0, 320, 240},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 2)
				SetFlash(0, 0, FLASH_MODE_FLASH);
			// Fallthrough
		case 1:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 2;
				PlaySoundObject(101, SOUND_MODE_PLAY);
			}

			break;

		case 2:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no == 2)
				npc->damage = 10;

			if (npc->ani_no > 4)
			{
				SetDestroyNpChar(npc->x, npc->y, 0x1000, 8);
				npc->cond = 0;
			#ifdef FIX_MAJOR_BUGS
				return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
			#endif
			}

			break;
	}

	npc->rect = rect[npc->ani_no];
}

// Critter (purple)
void ActNpc147(NPCHAR *npc)
{
	int xm, ym;
	unsigned char deg;

	RECT rcLeft[6] = {
		{0, 96, 16, 112},
		{16, 96, 32, 112},
		{32, 96, 48, 112},
		{48, 96, 64, 112},
		{64, 96, 80, 112},
		{80, 96, 96, 112},
	};

	RECT rcRight[6] = {
		{0, 112, 16, 128},
		{16, 112, 32, 128},
		{32, 112, 48, 128},
		{48, 112, 64, 128},
		{64, 112, 80, 128},
		{80, 112, 96, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y += 3 * 0x200;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->act_wait >= 8 && npc->x - (96 * 0x200) < gMC.x && npc->x + (96 * 0x200) > gMC.x && npc->y - (96 * 0x200) < gMC.y && npc->y + (32 * 0x200) > gMC.y)
			{
				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				npc->ani_no = 1;
			}
			else
			{
				if (npc->act_wait < 8)
					++npc->act_wait;

				npc->ani_no = 0;
			}

			if (npc->shock)
			{
				npc->act_no = 2;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			if (npc->act_wait >= 8 && npc->x - (48 * 0x200) < gMC.x && npc->x + (48 * 0x200) > gMC.x && npc->y - (96 * 0x200) < gMC.y && npc->y + (32 * 0x200) > gMC.y)
			{
				npc->act_no = 2;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			break;

		case 2:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 3;
				npc->ani_no = 2;
				npc->ym = -0x5FF;
				PlaySoundObject(30, SOUND_MODE_PLAY);

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 3:
			if (npc->ym > 0x100)
			{
				npc->tgt_y = npc->y;
				npc->act_no = 4;
				npc->ani_no = 3;
				npc->act_wait = 0;
				npc->act_wait = 0;	// Duplicate line
			}

			break;

		case 4:
			if (npc->x < gMC.x)
				npc->direct = 2;
			else
				npc->direct = 0;

			++npc->act_wait;

			if (npc->flag & 7 || npc->act_wait > 60)
			{
				npc->damage = 3;
				npc->act_no = 5;
				npc->ani_no = 2;
				break;
			}

			if (npc->act_wait % 4 == 1)
				PlaySoundObject(109, SOUND_MODE_PLAY);

			if (npc->flag & 8)
				npc->ym = -0x200;

			if (npc->act_wait % 30 == 6)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-6, 6);
				ym = GetSin(deg) * 3;
				xm = GetCos(deg) * 3;

				SetNpChar(148, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 3;

			break;

		case 5:
			if (npc->flag & 8)
			{
				npc->damage = 2;
				npc->xm = 0;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->act_no = 1;
				PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			break;
	}

	if (npc->act_no != 4)
	{
		npc->ym += 0x20;
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;
	}
	else
	{
		if (npc->y > npc->tgt_y)
			npc->ym -= 0x10;
		else
			npc->ym += 0x10;

		if (npc->ym > 0x200)
			npc->ym = 0x200;
		if (npc->ym < -0x200)
			npc->ym = -0x200;

		if (npc->xm > 0x200)
			npc->xm = 0x200;
		if (npc->xm < -0x200)
			npc->xm = -0x200;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Purple Critter's projectile
void ActNpc148(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[2] = {
		{96, 96, 104, 104},
		{104, 96, 112, 104},
	};

	if (++npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rect_left[npc->ani_no];

	if (++npc->count1 > 300)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}
}

// Moving block (horizontal)
void ActNpc149(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->x += 8 * 0x200;
			npc->y += 8 * 0x200;

			if (npc->direct == 0)
				npc->act_no = 10;
			else
				npc->act_no = 20;

			npc->xm = 0;
			npc->ym = 0;

			npc->bits |= NPC_SOLID_HARD;
			break;

		case 10:
			npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
			npc->damage = 0;

			if (gMC.x < npc->x + (25 * 0x200) && gMC.x > npc->x - (25 * 0x10 * 0x200) && gMC.y < npc->y + (25 * 0x200) && gMC.y > npc->y - (25 * 0x200))
			{
				npc->act_no = 11;
				npc->act_wait = 0;
			}

			break;

		case 11:
			if (++npc->act_wait % 10 == 6)
				PlaySoundObject(107, SOUND_MODE_PLAY);

			if (npc->flag & 1)
			{
				npc->xm = 0;
				npc->direct = 2;
				npc->act_no = 20;
				SetQuake(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x - (16 * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				break;
			}

			if (gMC.flag & 1)
			{
				npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 100;
			}
			else
			{
				npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 0;
			}

			npc->xm -= 0x20;

			break;

		case 20:
			npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
			npc->damage = 0;

			if (gMC.x > npc->x - (25 * 0x200) && gMC.x < npc->x + (25 * 0x10 * 0x200) && gMC.y < npc->y + (25 * 0x200) && gMC.y > npc->y - (25 * 0x200))
			{
				npc->act_no = 21;
				npc->act_wait = 0;
			}

			break;

		case 21:
			if (++npc->act_wait % 10 == 6)
				PlaySoundObject(107, SOUND_MODE_PLAY);

			if (npc->flag & 4)
			{
				npc->xm = 0;
				npc->direct = 0;
				npc->act_no = 10;
				SetQuake(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (16 * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				break;
			}

			if (gMC.flag & 4)
			{
				npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 100;
			}
			else
			{
				npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 0;
			}

			npc->xm += 0x20;

			break;
	}

	if (npc->xm > 0x200)
		npc->xm = 0x200;
	if (npc->xm < -0x200)
		npc->xm = -0x200;

	npc->x += npc->xm;

	RECT rect = {16, 0, 48, 32};
	npc->rect = rect;
}

// Quote
void ActNpc150(NPCHAR *npc)
{
	int i;

	RECT rcLeft[9] = {
		{0, 0, 16, 16},
		{48, 0, 64, 16},
		{144, 0, 160, 16},
		{16, 0, 32, 16},
		{0, 0, 16, 16},
		{32, 0, 48, 16},
		{0, 0, 16, 16},
		{160, 0, 176, 16},
		{112, 0, 128, 16},
	};

	RECT rcRight[9] = {
		{0, 16, 16, 32},
		{48, 16, 64, 32},
		{144, 16, 160, 32},
		{16, 16, 32, 32},
		{0, 16, 16, 32},
		{32, 16, 48, 32},
		{0, 16, 16, 32},
		{160, 16, 176, 32},
		{112, 16, 128, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;

			if (npc->direct > 10)
			{
				npc->x = gMC.x;
				npc->y = gMC.y;
				npc->direct -= 10;
			}
			break;

		case 2:
			npc->ani_no = 1;
			break;

		case 10:
			npc->act_no = 11;

			for (i = 0; i < 4; ++i)
				SetNpChar(4, npc->x, npc->y, Random(-0x155, 0x155), Random(-0x600, 0), 0, NULL, 0x100);

			PlaySoundObject(71, SOUND_MODE_PLAY);
			// Fallthrough
		case 11:
			npc->ani_no = 2;
			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 64;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 21:
			if (--npc->act_wait == 0)
				npc->cond = 0;

			break;

		case 50:
			npc->act_no = 51;
			npc->ani_no = 3;
			npc->ani_wait = 0;
			// Fallthrough
		case 51:
			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 6)
				npc->ani_no = 3;

			if (npc->direct == 0)
				npc->x -= 1 * 0x200;
			else
				npc->x += 1 * 0x200;

			break;

		case 60:
			npc->act_no = 61;
			npc->ani_no = 7;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			// Fallthrough
		case 61:
			npc->tgt_y += 0x100;
			npc->x = npc->tgt_x + (Random(-1, 1) * 0x200);
			npc->y = npc->tgt_y + (Random(-1, 1) * 0x200);
			break;

		case 70:
			npc->act_no = 71;
			npc->act_wait = 0;
			npc->ani_no = 3;
			npc->ani_wait = 0;
			// Fallthrough
		case 71:
			if (npc->direct == 0)
				npc->x += 0x100;
			else
				npc->x -= 0x100;

			if (++npc->ani_wait > 8)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 6)
				npc->ani_no = 3;

			break;

		case 80:
			npc->ani_no = 8;
			break;

		case 99:
		case 100:
			npc->act_no = 101;
			npc->ani_no = 3;
			npc->ani_wait = 0;
			// Fallthrough
		case 101:
			npc->ym += 0x40;

			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (npc->flag & 8)
			{
				npc->ym = 0;
				npc->act_no = 102;
			}

			npc->y += npc->ym;
			break;

		case 102:
			if (++npc->ani_wait > 8)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 6)
				npc->ani_no = 3;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->act_no == 21)
	{
		npc->rect.bottom = npc->rect.top + (npc->act_wait / 4);

		if (npc->act_wait / 2 % 2)
			++npc->rect.left;
	}

	// Use a different sprite if the player is wearing the Mimiga Mask
	if (gMC.equip & EQUIP_MIMIGA_MASK)
	{
		npc->rect.top += 32;
		npc->rect.bottom += 32;
	}
}

// Blue robot (standing)
void ActNpc151(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{192, 0, 208, 16},
		{208, 0, 224, 16},
	};

	RECT rcRight[2] = {
		{192, 16, 208, 32},
		{208, 16, 224, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough

		case 1:
			if (Random(0, 100) == 0)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 2:
			if (++npc->act_wait > 16)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Shutter stuck
void ActNpc152(NPCHAR *npc)
{
	RECT rc = {0, 0, 0, 0};

	switch (npc->act_no)
	{
		case 0:
			if (npc->direct == 2)
				npc->y += 16 * 0x200;

			npc->act_no = 1;
			break;
	}

	npc->rect = rc;
}

const RECT grcKitL[21] = {
	{0, 0, 24, 24},
	{24, 0, 48, 24},
	{48, 0, 72, 24},
	{0, 0, 24, 24},
	{72, 0, 96, 24},
	{0, 0, 24, 24},
	{96, 0, 120, 24},
	{120, 0, 144, 24},
	{144, 0, 168, 24},
	{168, 0, 192, 24},
	{192, 0, 216, 24},
	{216, 0, 240, 24},
	{240, 0, 264, 24},
	{264, 0, 288, 24},
	{0, 48, 24, 72},
	{24, 48, 48, 72},
	{48, 48, 72, 72},
	{72, 48, 96, 72},
	{288, 0, 312, 24},
	{24, 48, 48, 72},
	{96, 48, 120, 72}
};

const RECT grcKitR[21] = {
	{0, 24, 24, 48},
	{24, 24, 48, 48},
	{48, 24, 72, 48},
	{0, 24, 24, 48},
	{72, 24, 96, 48},
	{0, 24, 24, 48},
	{96, 24, 120, 48},
	{120, 24, 144, 48},
	{144, 24, 168, 48},
	{168, 24, 192, 48},
	{192, 24, 216, 48},
	{216, 24, 240, 48},
	{240, 24, 264, 48},
	{264, 24, 288, 48},
	{0, 72, 24, 96},
	{24, 72, 48, 96},
	{48, 72, 72, 96},
	{72, 72, 96, 96},
	{288, 24, 312, 48},
	{24, 72, 48, 96},
	{96, 72, 120, 96}
};

// Gaudi
void ActNpc153(NPCHAR *npc)
{
	if (npc->x > gMC.x + (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->x < gMC.x - (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->y > gMC.y + (((WINDOW_HEIGHT / 2) + 120) * 0x200) || npc->y < gMC.y - (((WINDOW_HEIGHT / 2) + 120) * 0x200))
		return;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->xm = 0;
			npc->ani_no = 0;
			npc->y += 3 * 0x200;
			// Fallthrough
		case 1:
			if (Random(0, 100) == 1)
			{
				npc->act_no = 2;
				npc->ani_no = 1;
				npc->act_wait = 0;
			}

			if (Random(0, 100) == 1)
			{
				if (npc->direct == 0)
					npc->direct = 2;
				else
					npc->direct = 0;
			}

			if (Random(0, 100) == 1)
				npc->act_no = 10;

			break;

		case 2:
			if (++npc->act_wait > 20)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = Random(25, 100);
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 11:
			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			if (npc->direct == 0)
				npc->xm = -0x200;
			else
				npc->xm = 0x200;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->act_no = 1;
				npc->ani_no = 0;
				npc->xm = 0;
			}

			if (npc->direct == 0 && npc->flag & 1)
			{
				npc->ani_no = 2;
				npc->ym = -0x5FF;
				npc->act_no = 20;

				if (!(gMC.cond & 2))
					PlaySoundObject(30, SOUND_MODE_PLAY);
			}
			else if (npc->direct == 2 && npc->flag & 4)
			{
				npc->ani_no = 2;
				npc->ym = -0x5FF;
				npc->act_no = 20;

				if (!(gMC.cond & 2))
					PlaySoundObject(30, SOUND_MODE_PLAY);
			}

			break;

		case 20:
			if (npc->direct == 0 && npc->flag & 1)
				++npc->count1;
			else if (npc->direct == 2 && npc->flag & 4)
				++npc->count1;
			else
				npc->count1 = 0;

			if (npc->count1 > 10)
			{
				if (npc->direct == 0)
					npc->direct = 2;
				else
					npc->direct = 0;
			}

			if (npc->direct == 0)
				npc->xm = -0x100;
			else
				npc->xm = 0x100;

			if (npc->flag & 8)
			{
				npc->act_no = 21;
				npc->ani_no = 20;
				npc->act_wait = 0;
				npc->xm = 0;

				if (!(gMC.cond & 2))
					PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			break;

		case 21:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;
	}

	npc->ym += 0x40;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = grcKitL[npc->ani_no];
	else
		npc->rect = grcKitR[npc->ani_no];

	if (npc->life <= 985)
	{
		npc->code_char = 154;
		npc->act_no = 0;
	}
}

// Gaudi (dead)
void ActNpc154(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			npc->damage = 0;
			npc->act_no = 1;
			npc->ani_no = 9;
			npc->ym = -0x200;

			if (npc->direct == 0)
				npc->xm = 0x100;
			else
				npc->xm = -0x100;

			PlaySoundObject(53, SOUND_MODE_PLAY);
			break;

		case 1:
			if (npc->flag & 8)
			{
				npc->ani_no = 10;
				npc->ani_wait = 0;
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			npc->xm = (npc->xm * 8) / 9;

			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 11)
				npc->ani_no = 10;

			if (++npc->act_wait > 50)
				npc->cond |= 8;

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = grcKitL[npc->ani_no];
	else
		npc->rect = grcKitR[npc->ani_no];
}

// Gaudi (flying)
void ActNpc155(NPCHAR *npc)
{
	unsigned char deg;
	int xm, ym;

	if (npc->x > gMC.x + (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->x < gMC.x - (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->y > gMC.y + (((WINDOW_HEIGHT / 2) + 120) * 0x200) || npc->y < gMC.y - (((WINDOW_HEIGHT / 2) + 120) * 0x200))
		return;

	switch (npc->act_no)
	{
		case 0:
			deg = Random(0, 0xFF);
			npc->xm = GetCos(deg);
			deg += 0x40;
			npc->tgt_x = npc->x + (GetCos(deg) * 8);

			deg = Random(0, 0xFF);
			npc->ym = GetSin(deg);
			deg += 0x40;
			npc->tgt_y = npc->y + (GetSin(deg) * 8);

			npc->act_no = 1;
			npc->count1 = 120;
			npc->act_wait = Random(70, 150);
			npc->ani_no = 14;
			// Fallthrough
		case 1:
			if (++npc->ani_no > 15)
				npc->ani_no = 14;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->act_no = 2;
				npc->ani_no = 18;
			}

			break;

		case 2:
			if (++npc->ani_no > 19)
				npc->ani_no = 18;

			if (++npc->act_wait > 30)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-6, 6);
				ym = GetSin(deg) * 3;
				xm = GetCos(deg) * 3;
				SetNpChar(156, npc->x, npc->y, xm, ym, 0, NULL, 0x100);

				if (!(gMC.cond & 2))
					PlaySoundObject(39, SOUND_MODE_PLAY);

				npc->act_no = 1;
				npc->act_wait = Random(70, 150);
				npc->ani_no = 14;
				npc->ani_wait = 0;
			}

			break;
	}

	if (gMC.x < npc->x)
		npc->direct = 0;
	else
		npc->direct = 2;

	if (npc->tgt_x < npc->x)
		npc->xm -= 0x10;
	if (npc->tgt_x > npc->x)
		npc->xm += 0x10;

	if (npc->tgt_y < npc->y)
		npc->ym -= 0x10;
	if (npc->tgt_y > npc->y)
		npc->ym += 0x10;

	if (npc->xm > 0x200)
		npc->xm = 0x200;
	if (npc->xm < -0x200)
		npc->xm = -0x200;

	if (npc->ym > 0x200)
		npc->ym = 0x200;
	if (npc->ym < -0x200)
		npc->ym = -0x200;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = grcKitL[npc->ani_no];
	else
		npc->rect = grcKitR[npc->ani_no];

	if (npc->life <= 985)
	{
		npc->code_char = 154;
		npc->act_no = 0;
	}
}

// Gaudi projectile
void ActNpc156(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[3] = {
		{96, 112, 112, 128},
		{112, 112, 128, 128},
		{128, 112, 144, 128},
	};

	if (++npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rect_left[npc->ani_no];

	if (++npc->count1 > 300)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}
}

// Moving block (vertical)
void ActNpc157(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->x += 8 * 0x200;
			npc->y += 8 * 0x200;

			if (npc->direct == 0)
				npc->act_no = 10;
			else
				npc->act_no = 20;

			npc->xm = 0;
			npc->ym = 0;
			npc->bits |= NPC_SOLID_HARD;

			break;

		case 10:
			npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
			npc->damage = 0;

			if (gMC.y < npc->y + (25 * 0x200) && gMC.y > npc->y - (25 * 0x10 * 0x200) && gMC.x < npc->x + (25 * 0x200) && gMC.x > npc->x - (25 * 0x200))
			{
				npc->act_no = 11;
				npc->act_wait = 0;
			}

			break;

		case 11:
			if (++npc->act_wait % 10 == 6)
				PlaySoundObject(107, SOUND_MODE_PLAY);

			if (npc->flag & 2)
			{
				npc->ym = 0;
				npc->direct = 2;
				npc->act_no = 20;
				SetQuake(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y - (16 * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				break;
			}

			if (gMC.flag & 2)
			{
				npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 100;
			}
			else
			{
				npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 0;
			}

			npc->ym -= 0x20;

			break;

		case 20:
			npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
			npc->damage = 0;

			if (gMC.y > npc->y - (25 * 0x200) && gMC.y < npc->y + (25 * 0x10 * 0x200) && gMC.x < npc->x + (25 * 0x200) && gMC.x > npc->x - (25 * 0x200))
			{
				npc->act_no = 21;
				npc->act_wait = 0;
			}

			break;

		case 21:
			if (++npc->act_wait % 10 == 6)
				PlaySoundObject(107, SOUND_MODE_PLAY);

			if (npc->flag & 8)
			{
				npc->ym = 0;
				npc->direct = 0;
				npc->act_no = 10;
				SetQuake(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (16 * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				break;
			}

			if (gMC.flag & 8)
			{
				npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 100;
			}
			else
			{
				npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 0;
			}

			npc->ym += 0x20;

			break;
	}

	if (npc->ym > 0x200)
		npc->ym = 0x200;
	if (npc->ym < -0x200)
		npc->ym = -0x200;

	npc->y += npc->ym;

	RECT rect = {16, 0, 48, 32};
	npc->rect = rect;
}

// Fish Missile
void ActNpc158(NPCHAR *npc)
{
	int dir;

	RECT rect[8] = {
		{0, 224, 16, 240},
		{16, 224, 32, 240},
		{32, 224, 48, 240},
		{48, 224, 64, 240},
		{64, 224, 80, 240},
		{80, 224, 96, 240},
		{96, 224, 112, 240},
		{112, 224, 128, 240},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			switch (npc->direct)
			{
				case 0:
					npc->count1 = 0xA0;
					break;

				case 1:
					npc->count1 = 0xE0;
					break;

				case 2:
					npc->count1 = 0x20;
					break;

				case 3:
					npc->count1 = 0x60;
					break;
			}
			// Fallthrough
		case 1:
			npc->xm = 2 * GetCos(npc->count1);
			npc->ym = 2 * GetSin(npc->count1);
			npc->y += npc->ym;
			npc->x += npc->xm;
			dir = GetArktan(npc->x - gMC.x, npc->y - gMC.y);

			if (dir < npc->count1)
			{
				if (npc->count1 - dir < 0x80)
					--npc->count1;
				else
					++npc->count1;
			}
			else
			{
				if (dir - npc->count1 < 0x80)
					++npc->count1;
				else
					--npc->count1;
			}

			if (npc->count1 > 0xFF)
				npc->count1 -= 0x100;
			if (npc->count1 < 0)
				npc->count1 += 0x100;

			break;
	}

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		SetCaret(npc->x, npc->y, CARET_EXHAUST, DIR_AUTO);
	}

	npc->ani_no = (npc->count1 + 0x10) / 0x20;

	if (npc->ani_no > 7)
		npc->ani_no = 7;

	npc->rect = rect[npc->ani_no];
}

// Monster X (defeated)
void ActNpc159(NPCHAR *npc)
{
	int i;

	RECT rect = {144, 128, 192, 200};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x + (Random(-16, 16) * 0x200), npc->y + (Random(-16, 16) * 0x200), Random(-341, 341), Random(-341, 341), 0, NULL, 0x100);
			// Fallthrough
		case 1:
			if (++npc->act_wait > 50)
			{
				npc->act_no = 2;
				npc->xm = -0x100;
			}

			if (npc->act_wait / 2 % 2)
				npc->x += 0x200;
			else
				npc->x -= 0x200;

			break;

		case 2:
			++npc->act_wait;
			npc->ym += 0x40;

			if (npc->y > 40 * 0x10 * 0x200)
				npc->cond = 0;

			break;
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	npc->rect = rect;

	if (npc->act_wait % 8 == 1)
		SetNpChar(4, npc->x + (Random(-16, 16) * 0x200), npc->y + (Random(-16, 16) * 0x200), Random(-341, 341), Random(-341, 341), 0, NULL, 0x100);
}

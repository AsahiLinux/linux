// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "NpcAct.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Caret.h"
#include "CommonDefines.h"
#include "Frame.h"
#include "Game.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Mimiga (jailed)
void ActNpc240(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{160, 64, 176, 80},
		{176, 64, 192, 80},
		{192, 64, 208, 80},
		{160, 64, 176, 80},
		{208, 64, 224, 80},
		{160, 64, 176, 80},
	};

	RECT rcRight[6] = {
		{160, 80, 176, 96},
		{176, 80, 192, 96},
		{192, 80, 208, 96},
		{160, 80, 176, 96},
		{208, 80, 224, 96},
		{160, 80, 176, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->xm = 0;
			// Fallthrough
		case 1:
			if (Random(0, 60) == 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			if (Random(0, 60) == 1)
			{
				npc->act_no = 10;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 2:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = Random(0, 16);
			npc->ani_no = 2;
			npc->ani_wait = 0;

			if (Random(0, 9) % 2)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 11:
			if (npc->direct == 0 && npc->flag & 1)
				npc->direct = 2;
			else if (npc->direct == 2 && npc->flag & 4)
				npc->direct = 0;

			if (npc->direct == 0)
				npc->xm = -0x200;
			else
				npc->xm = 0x200;

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			if (++npc->act_wait > 32)
				npc->act_no = 0;

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Critter (Last Cave)
void ActNpc241(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
		{32, 0, 48, 16},
	};

	RECT rcRight[3] = {
		{0, 16, 16, 32},
		{16, 16, 32, 32},
		{32, 16, 48, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y += 3 * 0x200;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->act_wait >= 8 && npc->x - (144 * 0x200) < gMC.x && npc->x + (144 * 0x200) > gMC.x && npc->y - (80 * 0x200) < gMC.y && npc->y + (80 * 0x200) > gMC.y)
			{
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

			if (npc->act_wait >= 8 && npc->x - (96 * 0x200) < gMC.x && npc->x + (96 * 0x200) > gMC.x && npc->y - (80 * 0x200) < gMC.y && npc->y + (96 * 0x200) > gMC.y)
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

				if (npc->direct == 0)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;
			}

			break;

		case 3:
			if (npc->flag & 8)
			{
				npc->xm = 0;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->act_no = 1;
				PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			break;
	}

	npc->ym += 0x55;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Bat (Last Cave)
void ActNpc242(NPCHAR *npc)
{
	if (npc->x < 0 || npc->x > gMap.width * 0x10 * 0x200)
	{
		VanishNpChar(npc);
		return;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			npc->act_wait = Random(0, 50);
			// Fallthrough
		case 1:
			if (npc->act_wait != 0)
			{
				--npc->act_wait;
				break;
			}

			npc->act_no = 2;
			npc->ym = 0x400;
			// Fallthrough
		case 2:
			if (npc->direct == 0)
				npc->xm = -0x100;
			else
				npc->xm = 0x100;

			if (npc->tgt_y < npc->y)
				npc->ym -= 0x10;
			if (npc->tgt_y > npc->y)
				npc->ym += 0x10;

			if (npc->ym > 0x300)
				npc->ym = 0x300;
			if (npc->ym < -0x300)
				npc->ym = -0x300;

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[4] = {
		{32, 32, 48, 48},
		{48, 32, 64, 48},
		{64, 32, 80, 48},
		{80, 32, 96, 48},
	};

	RECT rect_right[4] = {
		{32, 48, 48, 64},
		{48, 48, 64, 64},
		{64, 48, 80, 64},
		{80, 48, 96, 64},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Bat generator (Last Cave)
void ActNpc243(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = Random(0, 500);
			// Fallthrough
		case 1:
			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->act_no = 0;
				SetNpChar(242, npc->x, npc->y + (Random(-0x20, 0x20) * 0x200), 0, 0, npc->direct, NULL, 0x100);
			}

			break;
	}
}

// Lava drop
void ActNpc244(NPCHAR *npc)
{
	RECT rc = {96, 0, 104, 16};
	BOOL bHit;

	int i;

	npc->ym += 0x40;

	bHit = FALSE;

	if (npc->flag & 0xFF)
		bHit = TRUE;

	if (npc->act_wait > 10 && npc->flag & 0x100)
		bHit = TRUE;

	if (bHit)
	{
		for (i = 0; i < 3; ++i)
			SetCaret(npc->x, npc->y + (4 * 0x200), CARET_BUBBLE, DIR_RIGHT);

		if (npc->x > gMC.x - (256 * 0x200) && npc->x < gMC.x + (256 * 0x200) && npc->y > gMC.y - (160 * 0x200) && npc->y < gMC.y + (160 * 0x200))
			PlaySoundObject(21, SOUND_MODE_PLAY);

		npc->cond = 0;
	}
	else
	{
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->y += npc->ym;

		npc->rect = rc;
	}
}

// Lava drop generator
void ActNpc245(NPCHAR *npc)
{
	RECT rc[4] = {
		{0, 0, 0, 0},
		{104, 0, 112, 16},
		{112, 0, 120, 16},
		{120, 0, 128, 16},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_x = npc->x;
			npc->act_wait = npc->code_event;
			// Fallthrough
		case 1:
			npc->ani_no = 0;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
				return;
			}

			npc->act_no = 10;
			npc->ani_wait = 0;
			break;

		case 10:
			if (++npc->ani_wait > 10)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
			{
				npc->ani_no = 0;
				npc->act_no = 1;
				npc->act_wait = npc->code_flag;
				SetNpChar(244, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
			}

			break;
	}

	if (npc->ani_wait / 2 % 2)
		npc->x = npc->tgt_x;
	else
		npc->x = npc->tgt_x + 0x200;

	npc->rect = rc[npc->ani_no];
}

// Press (proximity)
void ActNpc246(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{144, 112, 160, 136},
		{160, 112, 176, 136},
		{176, 112, 192, 136},
	};

	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 4 * 0x200;
			// Fallthrough
		case 1:
			if (gMC.x < npc->x + (8 * 0x200) && gMC.x > npc->x - (8 * 0x200) && gMC.y > npc->y + (8 * 0x200) && gMC.y < npc->y + (128 * 0x200))
				npc->act_no = 5;

			break;

		case 5:
			if (npc->flag & 8)
			{
				// Another place where this blank space is needed for ASM-accuracy.
				// Chances are there used to be commented-out code here.
			}
			else
			{
				npc->act_no = 10;
				npc->ani_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 10:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 2;

			if (gMC.y > npc->y)
			{
				npc->bits &= ~NPC_SOLID_HARD;
				npc->damage = 0x7F;
			}
			else
			{
				npc->bits |= NPC_SOLID_HARD;
				npc->damage = 0;
			}

			if (npc->flag & 8)
			{
				if (npc->ani_no > 1)
				{
					for (i = 0; i < 4; ++i)
						SetNpChar(4, npc->x, npc->y, Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

					PlaySoundObject(26, SOUND_MODE_PLAY);
					SetQuake(10);
				}

				npc->act_no = 20;
				npc->ani_no = 0;
				npc->ani_wait = 0;
				npc->bits |= NPC_SOLID_HARD;
				npc->damage = 0;
			}

			break;
	}

	if (npc->act_no >= 5)
	{
		npc->ym += 0x80;
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->y += npc->ym;
	}

	npc->rect = rcLeft[npc->ani_no];
}

// Misery (boss)
void ActNpc247(NPCHAR *npc)
{
	unsigned char deg;
	int xm, ym;

	RECT rcLeft[9] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
		{32, 0, 48, 16},
		{48, 0, 64, 16},
		{64, 0, 80, 16},
		{80, 0, 96, 16},
		{96, 0, 112, 16},
		{0, 0, 0, 0},
		{112, 0, 128, 16},
	};

	RECT rcRight[9] = {
		{0, 16, 16, 32},
		{16, 16, 32, 32},
		{32, 16, 48, 32},
		{48, 16, 64, 32},
		{64, 16, 80, 32},
		{80, 16, 96, 32},
		{96, 16, 112, 32},
		{0, 0, 0, 0},
		{112, 16, 128, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 6 * 0x200;
			npc->tgt_y = 64 * 0x200;
			// Fallthrough
		case 1:
			if (Random(0, 120) == 10)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 2:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;

		case 20:
			npc->xm = 0;
			npc->ym += 0x40;

			if (npc->flag & 8)
			{
				npc->act_no = 21;
				npc->ani_no = 2;
			}

			break;

		case 21:
			if (Random(0, 120) == 10)
			{
				npc->act_no = 22;
				npc->act_wait = 0;
				npc->ani_no = 3;
			}

			break;

		case 22:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 21;
				npc->ani_no = 2;
			}

			break;

		case 100:
			npc->act_no = 101;
			npc->act_wait = 0;
			npc->ani_no = 0;
			npc->xm = 0;
			npc->bits |= NPC_SHOOTABLE;
			npc->count2 = npc->life;
			// Fallthrough
		case 101:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->y < npc->tgt_y)
				npc->ym += 0x20;
			else
				npc->ym -= 0x20;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x200)
				npc->ym = 0x200;

			if (++npc->act_wait > 200 || npc->life <= npc->count2 - 80)
			{
				npc->act_wait = 0;
				npc->act_no = 110;
			}

			break;

		case 110:
			npc->act_no = 111;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			// Fallthrough
		case 111:
			if (++npc->act_wait % 2)
				npc->ani_no = 5;
			else
				npc->ani_no = 6;

			if (npc->act_wait > 30)
			{
				npc->act_wait = 0;

				if (++npc->count1 % 3 == 0)
					npc->act_no = 113;
				else
					npc->act_no = 112;

				npc->ani_no = 4;
			}

			break;

		case 112:
			if (++npc->act_wait % 6 == 0)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-4, 4);
				ym = GetSin(deg) * 4;
				xm = GetCos(deg) * 4;

				SetNpChar(248, npc->x, npc->y + (4 * 0x200), xm, ym, 0, NULL, 0x100);
				PlaySoundObject(34, SOUND_MODE_PLAY);
			}

			if (npc->act_wait > 30)
			{
				npc->act_wait = 0;
				npc->act_no = 150;
			}

			break;

		case 113:
			if (++npc->act_wait == 10)
			{
				ym = gMC.y - (64 * 0x200);
				SetNpChar(279, gMC.x, ym, 0, 0, 1, NULL, 0x100);
			}

			if (npc->act_wait > 30)
			{
				npc->act_wait = 0;
				npc->act_no = 150;
			}

			break;

		case 150:
			npc->act_no = 151;
			npc->act_wait = 0;
			npc->ani_no = 7;

			SetNpChar(249, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
			SetNpChar(249, npc->x, npc->y, 0, 0, 2, NULL, 0x100);

			npc->tgt_x = Random(9, 31) * 0x200 * 0x10;
			npc->tgt_y = Random(5, 7) * 0x200 * 0x10;

			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 151:
			if (++npc->act_wait == 42)
			{
				SetNpChar(249, npc->tgt_x + (16 * 0x200), npc->tgt_y, 0, 0, 0, NULL, 0x100);
				SetNpChar(249, npc->tgt_x - (16 * 0x200), npc->tgt_y, 0, 0, 2, NULL, 0x100);
			}

			if (npc->act_wait > 50)
			{
				npc->act_wait = 0;
				npc->ym = -0x200;
				npc->bits |= NPC_SHOOTABLE;
				npc->x = npc->tgt_x;
				npc->y = npc->tgt_y;

				if (npc->life < 340)
				{
					SetNpChar(252, 0, 0, 0, 0, 0, npc, 0x100);
					SetNpChar(252, 0, 0, 0, 0, 0x80, npc, 0x100);
				}

				if (npc->life < 180)
				{
					SetNpChar(252, 0, 0, 0, 0, 0x40, npc, 0x100);
					SetNpChar(252, 0, 0, 0, 0, 0xC0, npc, 0x100);
				}

				if (gMC.x < npc->x - (112 * 0x200) || gMC.x > npc->x + (112 * 0x200))
					npc->act_no = 160;
				else
					npc->act_no = 100;
			}

			break;

		case 160:
			npc->act_no = 161;
			npc->act_wait = 0;
			npc->ani_no = 4;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 161:
			if (npc->y < npc->tgt_y)
				npc->ym += 0x20;
			else
				npc->ym -= 0x20;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x200)
				npc->ym = 0x200;

			if (++npc->act_wait % 24 == 0)
			{
				SetNpChar(250, npc->x, npc->y + (4 * 0x200), 0, 0, 0, NULL, 0x100);
				PlaySoundObject(34, SOUND_MODE_PLAY);
			}

			if (npc->act_wait > 72)
			{
				npc->act_wait = 0;
				npc->act_no = 100;
			}

			break;

		case 1000:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->act_no = 1001;
			npc->act_wait = 0;
			npc->ani_no = 4;

			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;

			npc->xm = 0;
			npc->ym = 0;

			DeleteNpCharCode(252, TRUE);

			SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
			SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
			SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
			// Fallthrough
		case 1001:
			if (++npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x + (1 * 0x200);
			else
				npc->x = npc->tgt_x;

			break;

		case 1010:
			npc->ym += 0x10;

			if (npc->flag & 8)
			{
				npc->act_no = 1020;
				npc->ani_no = 8;
			}

			break;
	}

	if (npc->xm < -0x200)
		npc->xm = -0x200;
	if (npc->xm > 0x200)
		npc->xm = 0x200;

	if (npc->ym < -0x400)
		npc->ym = -0x400;
	if (npc->ym > 0x400)
		npc->ym = 0x400;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Boss Misery (vanishing)
void ActNpc248(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		npc->cond = 0;
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[3] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;

		if (++npc->ani_no > 2)
			npc->ani_no = 0;
	}

	npc->rect = rect_left[npc->ani_no];

	if (++npc->count1 > 300)
	{
		npc->cond = 0;
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
	}
}

// Boss Misery energy shot
void ActNpc249(NPCHAR *npc)
{
	RECT rc[2] = {
		{48, 48, 64, 64},
		{64, 48, 80, 64},
	};

	if (++npc->act_wait > 8)
		npc->cond = 0;

	if (npc->direct == 0)
	{
		npc->rect = rc[0];
		npc->x -= 2 * 0x200;
	}
	else
	{
		npc->rect = rc[1];
		npc->x += 2 * 0x200;
	}
}

// Boss Misery lightning ball
void ActNpc250(NPCHAR *npc)
{
	RECT rc[3] = {
		{0, 32, 16, 48},
		{16, 32, 32, 48},
		{32, 32, 48, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_y = npc->y;
			npc->xm = 0;
			npc->ym = -0x200;
			// Fallthrough
		case 1:
			if (npc->x < gMC.x)
				npc->xm += 0x10;
			else
				npc->xm -= 0x10;

			if (npc->y < npc->tgt_y)
				npc->ym += 0x20;
			else
				npc->ym -= 0x20;

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

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (gMC.x > npc->x - (8 * 0x200) && gMC.x < npc->x + (8 * 0x200) && gMC.y > npc->y)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 10)
			{
				SetNpChar(251, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				PlaySoundObject(101, SOUND_MODE_PLAY);
				npc->cond = 0;
				return;
			}

			if (npc->act_wait / 2 % 2)
				npc->ani_no = 2;
			else
				npc->ani_no = 1;
	}

	npc->rect = rc[npc->ani_no];
}

// Boss Misery lightning
void ActNpc251(NPCHAR *npc)
{
	RECT rc[2] = {
		{80, 32, 96, 64},
		{96, 32, 112, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->ani_no > 1)
				npc->ani_no = 0;

			npc->y += 8 * 0x200;

			if (npc->flag & 0xFF)
			{
				SetDestroyNpChar(npc->x, npc->y, npc->view.back, 3);
				npc->cond = 0;
			}

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Boss Misery bats
void ActNpc252(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{48, 32, 64, 48},
		{112, 32, 128, 48},
		{128, 32, 144, 48},
		{144, 32, 160, 48},
	};

	RECT rcRight[4] = {
		{48, 32, 64, 48},
		{112, 48, 128, 64},
		{128, 48, 144, 64},
		{144, 48, 160, 64},
	};

	unsigned char deg;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = 0;
			npc->count1 = npc->direct;
			// Fallthrough
		case 1:
			npc->count1 += 2;
			npc->count1 %= 0x100;

			deg = npc->count1;

			if (npc->act_wait < 192)
				++npc->act_wait;

			npc->x = npc->pNpc->x + (GetCos(deg) * npc->act_wait) / 4;
			npc->y = npc->pNpc->y + (GetSin(deg) * npc->act_wait) / 4;

			if (npc->pNpc->act_no == 151)
			{
				npc->act_no = 10;
				npc->ani_no = 0;
			}

			break;

		case 10:
			npc->act_no = 11;
			npc->bits |= NPC_SHOOTABLE;
			npc->bits &= ~NPC_INVULNERABLE;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;

			deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
			deg += (unsigned char)Random(-3, 3);
			npc->xm = GetCos(deg);
			npc->ym = GetSin(deg);

			npc->ani_no = 1;
			npc->ani_wait = 0;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 11:
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 0xFF)
			{
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				npc->cond = 0;
			}

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 1;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// EXP capsule
void ActNpc253(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	if (npc->life <= 100)
	{
		SetExpObjects(npc->x, npc->y, npc->code_flag);
		SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
		PlaySoundObject(25, SOUND_MODE_PLAY);
		npc->cond = 0;
	}

	RECT rc[2] = {
		{0, 64, 16, 80},
		{16, 64, 32, 80},
	};

	npc->rect = rc[npc->ani_no];
}

// Helicopter
void ActNpc254(NPCHAR *npc)
{
	RECT rc[2] = {
		{0, 0, 128, 64},
		{0, 64, 128, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			SetNpChar(255, npc->x + (18 * 0x200), npc->y - (57 * 0x200), 0, 0, 0, npc, 0x100);
			SetNpChar(255, npc->x - (32 * 0x200), npc->y - (52 * 0x200), 0, 0, 2, npc, 0x100);
			break;

		case 20:
			npc->act_wait = 0;
			npc->count1 = 60;
			npc->act_no = 21;
			break;

		case 30:
			npc->act_no = 21;
			SetNpChar(223, npc->x - (11 * 0x200), npc->y - (14 * 0x200), 0, 0, 0, NULL, 0x100);
			break;

		case 40:
			npc->act_no = 21;
			SetNpChar(223, npc->x - (9 * 0x200), npc->y - (14 * 0x200), 0, 0, 0, NULL, 0x100);
			SetNpChar(40, npc->x - (22 * 0x200), npc->y - (14 * 0x200), 0, 0, 0, NULL, 0x100);
			SetNpChar(93, npc->x - (35 * 0x200), npc->y - (14 * 0x200), 0, 0, 0, NULL, 0x100);
			break;
	}

	if (npc->direct == 0)
		npc->rect = rc[0];
	else
		npc->rect = rc[1];
}

// Helicopter blades
void ActNpc255(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{128, 0, 240, 16},
		{128, 16, 240, 32},
		{128, 32, 240, 48},
		{128, 16, 240, 32},
	};

	RECT rcRight[4] = {
		{240, 0, 320, 16},
		{240, 16, 320, 32},
		{240, 32, 320, 48},
		{240, 16, 320, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
			{
				npc->view.front = (56 * 0x200);
				npc->view.back = (56 * 0x200);
			}
			else
			{
				npc->view.front = (40 * 0x200);
				npc->view.back = (40 * 0x200);
			}
			// Fallthrough
		case 1:
			if (npc->pNpc->act_no >= 20)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			// Fallthrough
		case 11:
			if (++npc->ani_no > 3)
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
	{
		npc->x = npc->pNpc->x + (18 * 0x200);
		npc->y = npc->pNpc->y - (57 * 0x200);
	}
	else
	{
		npc->x = npc->pNpc->x - (32 * 0x200);
		npc->y = npc->pNpc->y - (52 * 0x200);
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Doctor (facing away)
void ActNpc256(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{48, 160, 72, 192},
		{72, 160, 96, 192},
		{0, 128, 24, 160},
		{24, 128, 48, 160},
		{0, 160, 24, 192},
		{24, 160, 48, 192},
	};

	switch (npc->act_no)
	{
		case 0:
			gSuperXpos = 0;
			npc->act_no = 1;
			npc->y -= 8 * 0x200;
			// Fallthrough
		case 1:
			npc->ani_no = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_wait = 0;
			npc->ani_no = 0;
			npc->count1 = 0;
			// Fallthrough
		case 11:
			if (++npc->ani_wait > 5)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
			{
				npc->ani_no = 0;
				++npc->count1;
			}

			if (npc->count1 > 5)
				npc->act_no = 1;

			break;

		case 20:
			npc->act_no = 21;
			// Fallthrough
		case 21:
			npc->ani_no = 2;
			break;

		case 40:
			npc->act_no = 41;
			SetNpChar(257, npc->x - (14 * 0x200), npc->y - (16 * 0x200), 0, 0, 0, NULL, 0x100);
			SetNpChar(257, npc->x - (14 * 0x200), npc->y - (16 * 0x200), 0, 0, 2, NULL, 0xAA);
			// Fallthrough
		case 41:
			npc->ani_no = 4;
			break;

		case 50:
			npc->act_no = 51;
			npc->ani_wait = 0;
			npc->ani_no = 4;
			npc->count1 = 0;
			// Fallthrough
		case 51:
			if (++npc->ani_wait > 5)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
			{
				npc->ani_no = 4;
				++npc->count1;
			}

			if (npc->count1 > 5)
				npc->act_no = 41;

			break;
	}

	npc->rect = rcLeft[npc->ani_no];
}

// Red crystal
void ActNpc257(NPCHAR *npc)
{
	RECT rc[3] = {
		{176, 32, 184, 48},
		{184, 32, 192, 48},
		{0, 0, 0, 0},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (gSuperXpos != 0)
				npc->act_no = 10;

			break;

		case 10:
			if (npc->x < gSuperXpos)
				npc->xm += 0x55;
			if (npc->x > gSuperXpos)
				npc->xm -= 0x55;

			if (npc->y < gSuperYpos)
				npc->ym += 0x55;
			if (npc->y > gSuperYpos)
				npc->ym -= 0x55;

			if (npc->xm > 0x400)
				npc->xm = 0x400;
			if (npc->xm < -0x400)
				npc->xm = -0x400;

			if (npc->ym > 0x400)
				npc->ym = 0x400;
			if (npc->ym < -0x400)
				npc->ym = -0x400;

			npc->x += npc->xm;
			npc->y += npc->ym;

			break;
	}

	if (++npc->ani_wait > 3)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	if (npc->direct == 0 && npc->xm > 0)
		npc->ani_no = 2;
	if (npc->direct == 2 && npc->xm < 0)
		npc->ani_no = 2;

	npc->rect = rc[npc->ani_no];
}

// Mimiga (sleeping)
void ActNpc258(NPCHAR *npc)
{
	RECT rc = {48, 32, 64, 48};
	npc->rect = rc;
}

// Curly (carried and unconcious)
void ActNpc259(NPCHAR *npc)
{
	RECT rcLeft = {224, 96, 240, 112};
	RECT rcRight = {224, 112, 240, 128};

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_INTERACTABLE;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (gMC.direct == 0)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->y = gMC.y - (4 * 0x200);

			if (npc->direct == 0)
			{
				npc->x = gMC.x + (3 * 0x200);
				npc->rect = rcLeft;
			}
			else
			{
				npc->x = gMC.x - (3 * 0x200);
				npc->rect = rcRight;
			}

			if (gMC.ani_no % 2)
				++npc->rect.top;

			break;

		case 10:
			npc->act_no = 11;
			npc->xm = 0x40;
			npc->ym = -0x20;

			npc->rect = rcLeft;
			break;

		case 11:
			if (npc->y < (64 * 0x200))
				npc->ym = 0x20;

			npc->x += npc->xm;
			npc->y += npc->ym;
			break;

		case 20:
			VanishNpChar(npc);
			SetDestroyNpCharUp(npc->x, npc->y, 0x2000, 0x40);
			break;
	}
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "NpcAct.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Bullet.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Frame.h"
#include "Game.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Demon crown (opening)
void ActNpc300(NPCHAR *npc)
{
	RECT rc = {192, 80, 208, 96};

	if (npc->act_no == 0)
	{
		npc->act_no = 1;
		npc->y += 6 * 0x200;
	}

	if (++npc->ani_wait % 8 == 1)
		SetCaret(npc->x + (Random(-8, 8) * 0x200), npc->y + (8 * 0x200), CARET_TINY_PARTICLES, DIR_UP);

	npc->rect = rc;
}

// Fish missile (Misery)
void ActNpc301(NPCHAR *npc)
{
	int dir;

	RECT rect[8] = {
		{144, 0, 160, 16},
		{160, 0, 176, 16},
		{176, 0, 192, 16},
		{192, 0, 208, 16},
		{144, 16, 160, 32},
		{160, 16, 176, 32},
		{176, 16, 192, 32},
		{192, 16, 208, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->count1 = npc->direct;
			// Fallthrough
		case 1:
			npc->xm = GetCos(npc->count1) * 2;
			npc->ym = GetSin(npc->count1) * 2;

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

// Camera focus marker
void ActNpc302(NPCHAR *npc)
{
	int n;

	switch (npc->act_no)
	{
		case 10:
			npc->x = gMC.x;
			npc->y = gMC.y - 32 * 0x200;
			break;

		case 20:
			switch (npc->direct)
			{
				case 0:
					npc->x -= 2 * 0x200;
					break;

				case 1:
					npc->y -= 2 * 0x200;
					break;

				case 2:
					npc->x += 2 * 0x200;
					break;

				case 3:
					npc->y += 2 * 0x200;
					break;
			}

			gMC.x = npc->x;
			gMC.y = npc->y;
			break;

		case 30:
			npc->x = gMC.x;
			npc->y = gMC.y + (80 * 0x200);
			break;

		case 100:
			npc->act_no = 101;

			if (npc->direct != 0)
			{
				for (n = 0xAA; n < NPC_MAX; ++n)
				{
					if (gNPC[n].cond & 0x80 && gNPC[n].code_event == npc->direct)
					{
						npc->pNpc = &gNPC[n];
						break;
					}
				}

				if (n == NPC_MAX)
				{
					npc->cond = 0;
					break;
				}
			}
			else
			{
				npc->pNpc = gBoss;
			}
			// Fallthrough
		case 101:
			npc->x = (gMC.x + npc->pNpc->x) / 2;
			npc->y = (gMC.y + npc->pNpc->y) / 2;
			break;
	}
}

// Curly's machine gun
void ActNpc303(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{216, 152, 232, 168},
		{232, 152, 248, 168},
	};

	RECT rcRight[2] = {
		{216, 168, 232, 184},
		{232, 168, 248, 184},
	};

	if (npc->pNpc == NULL)
		return;

	// Set position
	if (npc->pNpc->direct == 0)
	{
		npc->direct = 0;
		npc->x = npc->pNpc->x - (8 * 0x200);
	}
	else
	{
		npc->direct = 2;
		npc->x = npc->pNpc->x + (8 * 0x200);
	}

	npc->y = npc->pNpc->y;

	// Animation
	npc->ani_no = 0;
	if (npc->pNpc->ani_no == 3 || npc->pNpc->ani_no == 5)
		npc->y -= 1 * 0x200;

	// Set framerect
	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Gaudi in hospital
void ActNpc304(NPCHAR *npc)
{
	RECT rc[4] = {
		{0, 176, 24, 192},
		{24, 176, 48, 192},
		{48, 176, 72, 192},
		{72, 176, 96, 192},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 10 * 0x200;
			// Fallthrough
		case 1:
			npc->ani_no = 0;
			break;

		case 10:
			npc->ani_no = 1;
			break;

		case 20:
			npc->act_no = 21;
			npc->ani_no = 2;
			// Fallthrough
		case 21:
			if (++npc->ani_wait > 10)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Small puppy
void ActNpc305(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{160, 144, 176, 160},
		{176, 144, 192, 160},
	};

	RECT rcRight[2] = {
		{160, 160, 176, 176},
		{176, 160, 192, 176},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 16 * 0x200;
			npc->ani_wait = Random(0, 6);
			// Fallthrough

		case 1:
			if (++npc->ani_wait > 6)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Balrog (nurse)
void ActNpc306(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{240, 96, 280, 128},
		{280, 96, 320, 128},
	};

	RECT rcRight[2] = {
		{160, 152, 200, 184},
		{200, 152, 240, 184},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->y += 4 * 0x200;
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
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Caged Santa
void ActNpc307(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{0, 32, 16, 48},
		{16, 32, 32, 48},
	};

	RECT rcRight[2] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->x += 1 * 0x200;
			npc->y -= 2 * 0x200;
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 1:
			if (Random(0, 160) == 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 2:
			if (++npc->act_wait > 12)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;
	}

	if (gMC.x < npc->x)
		npc->direct = 0;
	else
		npc->direct = 2;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Stumpy
void ActNpc308(NPCHAR *npc)
{
	unsigned char deg;

	RECT rcLeft[2] = {
		{128, 112, 144, 128},
		{144, 112, 160, 128},
	};

	RECT rcRight[2] = {
		{128, 128, 144, 144},
		{144, 128, 160, 144},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (gMC.x < npc->x + (240 * 0x200) && gMC.x > npc->x - (240 * 0x200) && gMC.y < npc->y + (192 * 0x200) && gMC.y > npc->y - (192 * 0x200))
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->xm2 = 0;
			npc->ym2 = 0;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 50)
				npc->act_no = 20;

			++npc->ani_wait;

			if (npc->act_wait > 1)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 1)
					npc->ani_no = 0;
			}

			if (gMC.x > npc->x + (320 * 0x200) || gMC.x < npc->x - (320 * 0x200) || gMC.y > npc->y + (240 * 0x200) || gMC.y < npc->y - (240 * 0x200))
				npc->act_no = 0;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;

			deg = (unsigned char)GetArktan(npc->x - gMC.x, npc->y - gMC.y);
			deg += (unsigned char)Random(-3, 3);
			npc->ym2 = GetSin(deg) * 2;
			npc->xm2 = GetCos(deg) * 2;

			if (npc->xm2 < 0)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 21:
			if (npc->xm2 < 0 && npc->flag & 1)
			{
				npc->direct = 2;
				npc->xm2 *= -1;
			}

			if (npc->xm2 > 0 && npc->flag & 4)
			{
				npc->direct = 0;
				npc->xm2 *= -1;
			}

			if (npc->ym2 < 0 && npc->flag & 2)
				npc->ym2 *= -1;
			if (npc->ym2 > 0 && npc->flag & 8)
				npc->ym2 *= -1;

			if (npc->flag & 0x100)
				npc->ym2 = -0x200;

			npc->x += npc->xm2;
			npc->y += npc->ym2;

			if (++npc->act_wait > 50)
				npc->act_no = 10;

			if (++npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Bute
void ActNpc309(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
	};

	RECT rcRight[2] = {
		{0, 16, 16, 32},
		{16, 16, 32, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->direct == 0)
			{
				if (gMC.x > npc->x - (288 * 0x200) && gMC.x < npc->x - (272 * 0x200))
				{
					npc->act_no = 10;
					break;
				}
			}
			else
			{
				if (gMC.x < npc->x + (288 * 0x200) && gMC.x > npc->x + (272 * 0x200))
				{
					npc->act_no = 10;
					break;
				}
			}

			return;

		case 10:
			npc->act_no = 11;
			npc->bits |= NPC_SHOOTABLE;
			npc->damage = 5;
			// Fallthrough
		case 11:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->direct == 0)
				npc->xm2 -= 0x10;
			else
				npc->xm2 += 0x10;

			if (npc->y > gMC.y)
				npc->ym2 -= 0x10;
			else
				npc->ym2 += 0x10;

			if (npc->xm2 < 0 && npc->flag & 1)
				npc->xm2 *= -1;
			if (npc->xm2 > 0 && npc->flag & 4)
				npc->xm2 *= -1;

			if (npc->ym2 < 0 && npc->flag & 2)
				npc->ym2 *= -1;
			if (npc->ym2 > 0 && npc->flag & 8)
				npc->ym2 *= -1;

			if (npc->xm2 < -0x5FF)
				npc->xm2 = -0x5FF;
			if (npc->xm2 > 0x5FF)
				npc->xm2 = 0x5FF;

			if (npc->ym2 < -0x5FF)
				npc->ym2 = -0x5FF;
			if (npc->ym2 > 0x5FF)
				npc->ym2 = 0x5FF;

			npc->x += npc->xm2;
			npc->y += npc->ym2;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->life <= 996)
	{
		npc->code_char = 316;
		npc->act_no = 0;
	}
}

// Bute (with sword)
void ActNpc310(NPCHAR *npc)
{
	RECT rcLeft[5] = {
		{32, 0, 56, 16},
		{56, 0, 80, 16},
		{80, 0, 104, 16},
		{104, 0, 128, 16},
		{128, 0, 152, 16},
	};

	RECT rcRight[5] = {
		{32, 16, 56, 32},
		{56, 16, 80, 32},
		{80, 16, 104, 32},
		{104, 16, 128, 32},
		{128, 16, 152, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits |= NPC_INVULNERABLE;
			npc->damage = 0;
			// Fallthrough
		case 1:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->ani_no = 0;

			if (gMC.x > npc->x - (128 * 0x200) && gMC.x < npc->x + (128 * 0x200) && gMC.y > npc->y - (128 * 0x200) && gMC.y < npc->y + (16 * 0x200))
				npc->act_no = 10;

			break;

		case 10:
			npc->xm = 0;
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits |= NPC_INVULNERABLE;
			npc->damage = 0;
			npc->ani_no = 0;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 30)
				npc->act_no = 20;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			npc->bits &= ~NPC_INVULNERABLE;
			npc->bits |= NPC_SHOOTABLE;
			npc->damage = 0;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 21:
			if (npc->direct == 0)
				npc->xm = -0x400;
			else
				npc->xm = 0x400;

			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (++npc->act_wait > 50)
				npc->act_no = 10;

			if (npc->x < gMC.x + (40 * 0x200) && npc->x > gMC.x - (40 * 0x200))
			{
				npc->ym = -0x300;
				npc->xm /= 2;
				npc->ani_no = 2;
				npc->act_no = 30;
				PlaySoundObject(30, SOUND_MODE_PLAY);
			}

			break;

		case 30:
			if (npc->ym > -0x80)
			{
				npc->act_no = 31;
				npc->ani_wait = 0;
				npc->ani_no = 3;
				npc->damage = 9;
			}

			break;

		case 31:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				npc->ani_no = 4;
			}

			if (npc->flag & 8)
			{
				npc->act_no = 32;
				npc->act_wait = 0;
				npc->xm = 0;
				npc->damage = 3;
			}

			break;

		case 32:
			if (++npc->act_wait > 30)
			{
				npc->act_no = 10;
				npc->damage = 0;
			}

			break;
	}

	npc->ym += 0x20;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->life <= 996)
	{
		npc->code_char = 316;
		npc->act_no = 0;
	}
}

// Bute archer
void ActNpc311(NPCHAR *npc)
{
	RECT rcLeft[7] = {
		{0, 32, 24, 56},
		{24, 32, 48, 56},
		{48, 32, 72, 56},
		{72, 32, 96, 56},
		{96, 32, 120, 56},
		{120, 32, 144, 56},
		{144, 32, 168, 56},
	};

	RECT rcRight[7] = {
		{0, 56, 24, 80},
		{24, 56, 48, 80},
		{48, 56, 72, 80},
		{72, 56, 96, 80},
		{96, 56, 120, 80},
		{120, 56, 144, 80},
		{144, 56, 168, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->direct == 0)
			{
				if (gMC.x > npc->x - (320 * 0x200) && gMC.x < npc->x && gMC.y > npc->y - (160 * 0x200) && gMC.y < npc->y + (160 * 0x200))
					npc->act_no = 10;
			}
			else
			{
				if (gMC.x > npc->x && gMC.x < npc->x + (320 * 0x200) && gMC.y > npc->y - (160 * 0x200) && gMC.y < npc->y + (160 * 0x200))
					npc->act_no = 10;
			}

			break;

		case 10:
			npc->act_no = 11;
			// Fallthrough
		case 11:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (gMC.x > npc->x - (224 * 0x200) && gMC.x < npc->x + (224 * 0x200) && gMC.y > npc->y - (8 * 0x200))
			{
				npc->ani_no = 1;
				npc->count1 = 0;
			}
			else
			{
				npc->ani_no = 4;
				npc->count1 = 1;
			}

			if (++npc->act_wait > 10)
				npc->act_no = 20;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			// Fallthrough
		case 21:
			if (npc->count1 == 0)
			{
				if (++npc->ani_no > 2)
					npc->ani_no = 1;
			}
			else
			{
				if (++npc->ani_no > 5)
					npc->ani_no = 4;
			}

			if (++npc->act_wait > 30)
				npc->act_no = 30;

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;

			if (npc->count1 == 0)
			{
				if (npc->direct == 0)
					SetNpChar(312, npc->x, npc->y, -0x600, 0, 0, NULL, 0x100);
				else
					SetNpChar(312, npc->x, npc->y, 0x600, 0, 2, NULL, 0x100);

				npc->ani_no = 3;
			}
			else
			{
				if (npc->direct == 0)
					SetNpChar(312, npc->x, npc->y, -0x600, -0x600, 0, NULL, 0x100);
				else
					SetNpChar(312, npc->x, npc->y, 0x600, -0x600, 2, NULL, 0x100);

				npc->ani_no = 6;
			}
			// Fallthrough
		case 31:
			if (++npc->act_wait > 30)
			{
				npc->act_no = 40;
				npc->act_wait = Random(0, 100);
			}

			break;

		case 40:
			npc->ani_no = 0;

			if (++npc->act_wait > 150)
				npc->act_no = 10;

			if (gMC.x < npc->x - (352 * 0x200) || gMC.x > npc->x + (352 * 0x200) || gMC.y < npc->y - (240 * 0x200) || gMC.y > npc->y + (240 * 0x200))
			{
				npc->act_no = 40;
				npc->act_wait = 0;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->life <= 992)
	{
		npc->code_char = 316;
		npc->act_no = 0;
	}
}

// Bute arrow projectile
void ActNpc312(NPCHAR *npc)
{
	RECT rcLeft[5] = {
		{0, 160, 16, 176},
		{16, 160, 32, 176},
		{32, 160, 48, 176},
		{48, 160, 64, 176},
		{64, 160, 80, 176},
	};

	RECT rcRight[5] = {
		{0, 176, 16, 192},
		{16, 176, 32, 192},
		{32, 176, 48, 192},
		{48, 176, 64, 192},
		{64, 176, 80, 192},
	};

	if (npc->act_no > 0 && npc->act_no < 20 && npc->flag & 0xFF)
		npc->act_no = 20;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = 0;

			if (npc->xm < 0)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->ym < 0)
				npc->ani_no = 0;
			else
				npc->ani_no = 2;
			// Fallthrough
		case 1:
			++npc->act_wait;

			if (npc->act_wait == 4)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->act_wait > 10)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->ani_wait = 0;
			npc->xm = 3 * npc->xm / 4;
			npc->ym = 3 * npc->ym / 4;
			// Fallthrough
		case 11:
			npc->ym += 0x20;

			if (++npc->ani_wait > 10)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 4)
				npc->ani_no = 4;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;
			npc->damage = 0;
			// Fallthrough
		case 21:
			if (++npc->act_wait > 30)
				npc->act_no = 30;

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;
			// Fallthrough
		case 31:
			if (++npc->act_wait > 30)
			{
				npc->cond = 0;
				return;
			}

			break;
	}

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->act_no == 31)
	{
		if (npc->act_wait / 2 % 2)
		{
			npc->rect.left = 0;
			npc->rect.right = 0;
		}
	}
}

// Ma Pignon
void ActNpc313(NPCHAR *npc)
{
	RECT rcLeft[14] = {
		{128, 0, 144, 16},
		{144, 0, 160, 16},
		{160, 0, 176, 16},
		{176, 0, 192, 16},
		{192, 0, 208, 16},
		{208, 0, 224, 16},
		{224, 0, 240, 16},
		{240, 0, 256, 16},
		{256, 0, 272, 16},
		{272, 0, 288, 16},
		{288, 0, 304, 16},
		{128, 0, 144, 16},
		{176, 0, 192, 16},
		{304, 0, 320, 16},
	};

	RECT rcRight[14] = {
		{128, 16, 144, 32},
		{144, 16, 160, 32},
		{160, 16, 176, 32},
		{176, 16, 192, 32},
		{192, 16, 208, 32},
		{208, 16, 224, 32},
		{224, 16, 240, 32},
		{240, 16, 256, 32},
		{256, 16, 272, 32},
		{272, 16, 288, 32},
		{288, 16, 304, 32},
		{128, 16, 144, 32},
		{176, 16, 192, 32},
		{304, 16, 320, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->y += 4 * 0x200;
			// Fallthrough
		case 1:
			npc->ym += 0x40;

			if (Random(0, 120) == 10)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			if (npc->x - (32 * 0x200) < gMC.x && npc->x + (32 * 0x200) > gMC.x)
			{
				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 2:
			if (++npc->act_wait > 8)
			{
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;

		case 100:
			npc->act_no = 110;
			npc->act_wait = 0;
			npc->count1 = 0;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 110:
			npc->damage = 1;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->ani_no = 0;

			if (++npc->act_wait > 4)
			{
				npc->act_wait = 0;
				npc->act_no = 120;

				if (++npc->count2 > 12)
				{
					npc->count2 = 0;
					npc->act_no = 300;
				}
			}

			break;

		case 120:
			npc->ani_no = 2;

			if (++npc->act_wait > 4)
			{
				npc->act_no = 130;
				npc->ani_no = 3;
				npc->xm = 2 * Random(-0x200, 0x200);
				npc->ym = -0x800;
				PlaySoundObject(30, SOUND_MODE_PLAY);
				++npc->count1;
			}

			break;

		case 130:
			npc->ym += 0x80;

			if (npc->y > 128 * 0x200)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->xm < 0 && npc->flag & 1)
				npc->xm *= -1;
			if (npc->xm > 0 && npc->flag & 4)
				npc->xm *= -1;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->ym < -0x200)
				npc->ani_no = 3;
			else if (npc->ym > 0x200)
				npc->ani_no = 4;
			else
				npc->ani_no = 0;

			if (npc->flag & 8)
			{
				npc->act_no = 140;
				npc->act_wait = 0;
				npc->ani_no = 2;
				npc->xm = 0;
			}

			if (npc->count1 > 4 && gMC.y < npc->y + (4 * 0x200))
			{
				npc->act_no = 200;
				npc->act_wait = 0;
				npc->xm = 0;
				npc->ym = 0;
			}

			break;

		case 140:
			npc->ani_no = 2;

			if (++npc->act_wait > 4)
				npc->act_no = 110;

			break;

		case 200:
			npc->ani_no = 5;

			if (++npc->act_wait > 10)
			{
				npc->act_no = 210;
				npc->ani_no = 6;

				if (npc->direct == 0)
					npc->xm = -0x5FF;
				else
					npc->xm = 0x5FF;

				PlaySoundObject(25, SOUND_MODE_PLAY);
				npc->bits &= ~NPC_SHOOTABLE;
				npc->bits |= NPC_INVULNERABLE;
				npc->damage = 10;
			}

			break;

		case 210:
			if (++npc->ani_no > 7)
				npc->ani_no = 6;

			if (npc->xm < 0 && npc->flag & 1)
				npc->act_no = 220;
			if (npc->xm > 0 && npc->flag & 4)
				npc->act_no = 220;

			break;

		case 220:
			npc->act_no = 221;
			npc->act_wait = 0;
			SetQuake(16);
			PlaySoundObject(26, SOUND_MODE_PLAY);
			npc->damage = 4;
			// Fallthrough
		case 221:
			if (++npc->ani_no > 7)
				npc->ani_no = 6;

			if (++npc->act_wait % 6 == 0)
				SetNpChar(314, Random(4, 16) * 0x200 * 0x10, 1 * 0x200 * 0x10, 0, 0, 0, NULL, 0x100);

			if (npc->act_wait > 30)
			{
				npc->count1 = 0;
				npc->act_no = 130;
				npc->bits |= NPC_SHOOTABLE;
				npc->bits &= ~NPC_INVULNERABLE;
				npc->damage = 3;
			}

			break;

		case 300:
			npc->act_no = 301;
			npc->ani_no = 9;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 301:
			if (++npc->ani_no > 11)
				npc->ani_no = 9;

			if (npc->direct == 0)
				npc->xm = -0x400;
			else
				npc->xm = 0x400;

			if (gMC.x > npc->x - (4 * 0x200) && gMC.x < npc->x + (4 * 0x200))
			{
				npc->act_no = 310;
				npc->act_wait = 0;
				npc->ani_no = 2;
				npc->xm = 0;
			}

			break;

		case 310:
			npc->ani_no = 2;

			if (++npc->act_wait > 4)
			{
				npc->act_no = 320;
				npc->ani_no = 12;
				npc->ym = -0x800;
				PlaySoundObject(25, SOUND_MODE_PLAY);
				npc->bits |= NPC_IGNORE_SOLIDITY;
				npc->bits &= ~NPC_SHOOTABLE;
				npc->bits |= NPC_INVULNERABLE;
				npc->damage = 10;
			}

			break;

		case 320:
			if (++npc->ani_no > 13)
				npc->ani_no = 12;

			if (npc->y < (16 * 0x200))
				npc->act_no = 330;

			break;

		case 330:
			npc->ym = 0;
			npc->act_no = 331;
			npc->act_wait = 0;
			SetQuake(16);
			PlaySoundObject(26, SOUND_MODE_PLAY);
			// Fallthrough
		case 331:
			if (++npc->ani_no > 13)
				npc->ani_no = 12;

			if (++npc->act_wait % 6 == 0)
				SetNpChar(315, Random(4, 16) * 0x200 * 0x10, 0, 0, 0, 0, NULL, 0x100);

			if (npc->act_wait > 30)
			{
				npc->count1 = 0;
				npc->act_no = 130;
				npc->bits |= NPC_SHOOTABLE;
				npc->bits &= ~NPC_INVULNERABLE;
				npc->damage = 3;
			}

			break;

		case 500:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->act_no = 501;
			npc->act_wait = 0;
			npc->ani_no = 8;
			npc->tgt_x = npc->x;
			npc->damage = 0;
			DeleteNpCharCode(315, TRUE);
			// Fallthrough
		case 501:
			npc->ym += 0x20;

			if (++npc->act_wait % 2)
				npc->x = npc->tgt_x;
			else
				npc->x = npc->tgt_x + (1 * 0x200);

			break;
	}

	if (npc->act_no > 100 && npc->act_no < 500 && npc->act_no != 210 && npc->act_no != 320)
	{
		if (IsActiveSomeBullet())
		{
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits |= NPC_INVULNERABLE;
		}
		else
		{
			npc->bits |= NPC_SHOOTABLE;
			npc->bits &= ~NPC_INVULNERABLE;
		}
	}

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Ma Pignon rock
void ActNpc314(NPCHAR *npc)
{
	RECT rc[3] = {
		{64, 64, 80, 80},
		{80, 64, 96, 80},
		{96, 64, 112, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->count2 = 0;
			npc->act_no = 100;
			npc->bits |= NPC_INVULNERABLE;
			npc->ani_no = Random(0, 2);
			// Fallthrough
		case 100:
			npc->ym += 0x40;

			if (npc->ym > 0x700)
				npc->ym = 0x700;

			if (npc->y > 128 * 0x200)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->flag & 8)
			{
				int i;

				npc->ym = -0x200;
				npc->act_no = 110;
				npc->bits |= NPC_IGNORE_SOLIDITY;
				PlaySoundObject(12, SOUND_MODE_PLAY);
				SetQuake(10);

				for (i = 0; i < 2; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (16 * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);
			}

			break;

		case 110:
			npc->ym += 0x40;

			if (npc->y > (gMap.length * 0x200 * 0x10) + (2 * 0x200 * 0x10))
			{
				npc->cond = 0;
				return;
			}

			break;
	}

	if (++npc->ani_wait > 6)
	{
		++npc->ani_wait;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	if (gMC.y > npc->y)
		npc->damage = 10;
	else
		npc->damage = 0;

	npc->y += npc->ym;

	npc->rect = rc[npc->ani_no];
}

// Ma Pignon clone
void ActNpc315(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{128, 0, 144, 16},
		{160, 0, 176, 16},
		{176, 0, 192, 16},
		{192, 0, 208, 16},
	};

	RECT rcRight[4] = {
		{128, 16, 144, 32},
		{160, 16, 176, 32},
		{176, 16, 192, 32},
		{192, 16, 208, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 3;
			npc->ym += 0x80;

			if (npc->y > 128 * 0x200)
			{
				npc->act_no = 130;
				npc->bits &= ~NPC_IGNORE_SOLIDITY;
			}

			break;

		case 100:
			npc->act_no = 110;
			npc->act_wait = 0;
			npc->count1 = 0;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 110:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->ani_no = 0;

			if (++npc->act_wait > 4)
			{
				npc->act_wait = 0;
				npc->act_no = 120;
			}

			break;

		case 120:
			npc->ani_no = 1;

			if (++npc->act_wait > 4)
			{
				npc->act_no = 130;
				npc->ani_no = 3;
				npc->xm = 2 * Random(-0x200, 0x200);
				npc->ym = -0x800;
				PlaySoundObject(30, SOUND_MODE_PLAY);
			}

			break;

		case 130:
			npc->ym += 0x80;

			if (npc->xm < 0 && npc->flag & 1)
				npc->xm *= -1;
			if (npc->xm > 0 && npc->flag & 4)
				npc->xm *= -1;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->ym < -0x200)
				npc->ani_no = 2;
			else if (npc->ym > 0x200)
				npc->ani_no = 0;
			else
				npc->ani_no = 3;

			if (npc->flag & 8)
			{
				npc->act_no = 140;
				npc->act_wait = 0;
				npc->ani_no = 1;
				npc->xm = 0;
			}

			break;

		case 140:
			npc->ani_no = 1;

			if (++npc->act_wait > 4)
			{
				npc->act_no = 110;
				npc->bits |= NPC_SHOOTABLE;
			}

			break;
	}

	if (npc->act_no > 100)
	{
		if (IsActiveSomeBullet())
		{
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits |= NPC_INVULNERABLE;
		}
		else
		{
			npc->bits |= NPC_SHOOTABLE;
			npc->bits &= ~NPC_INVULNERABLE;
		}
	}

	if (++npc->count2 > 300)
	{
		VanishNpChar(npc);
	}
	else
	{
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->x += npc->xm;
		npc->y += npc->ym;

		if (npc->direct == 0)
			npc->rect = rcLeft[npc->ani_no];
		else
			npc->rect = rcRight[npc->ani_no];
	}
}

// Bute (dead)
void ActNpc316(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{248, 32, 272, 56},
		{272, 32, 296, 56},
		{296, 32, 320, 56},
	};

	RECT rcRight[3] = {
		{248, 56, 272, 80},
		{272, 56, 296, 80},
		{296, 56, 320, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			npc->damage = 0;
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->view.front = 12 * 0x200;
			npc->view.back = 12 * 0x200;
			npc->view.top = 12 * 0x200;
			npc->ym = -0x200;

			if (npc->direct == 0)
				npc->xm = 0x100;
			else
				npc->xm = -0x100;

			PlaySoundObject(50, SOUND_MODE_PLAY);

			break;

		case 1:
			if (npc->flag & 8)
			{
				npc->ani_no = 1;
				npc->ani_wait = 0;
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			npc->xm = 8 * npc->xm / 9;

			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

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
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Mesa
void ActNpc317(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{0, 80, 32, 120},
		{32, 80, 64, 120},
		{64, 80, 96, 120},
		{96, 80, 128, 120},
	};

	RECT rcRight[4] = {
		{0, 120, 32, 160},
		{32, 120, 64, 160},
		{64, 120, 96, 160},
		{96, 120, 128, 160},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 8 * 0x200;
			npc->tgt_x = npc->x;
			// Fallthrough
		case 1:
			npc->xm = 0;
			npc->act_no = 2;
			npc->ani_no = 0;
			npc->count1 = 0;
			// Fallthrough
		case 2:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (++npc->ani_wait > 40)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (gMC.x > npc->x - (320 * 0x200) && gMC.x < npc->x + (320 * 0x200) && gMC.y > npc->y - (160 * 0x200) && gMC.y < npc->y + (160 * 0x200) && ++npc->count1 > 50)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_no = 2;
			SetNpChar(319, npc->x, npc->y, 0, 0, 0, npc, 0x100);
			// Fallthrough
		case 11:
			if (++npc->act_wait > 50)
			{
				npc->act_wait = 0;
				npc->act_no = 12;
				npc->ani_no = 3;
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			break;

		case 12:
			if (++npc->act_wait > 20)
				npc->act_no = 1;

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

	if (npc->life <= 936)
	{
		npc->code_char = 318;
		npc->act_no = 0;
	}
}

// Mesa (dead)
void ActNpc318(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{224, 80, 256, 120},
		{256, 80, 288, 120},
		{288, 80, 320, 120},
	};

	RECT rcRight[3] = {
		{224, 120, 256, 160},
		{256, 120, 288, 160},
		{288, 120, 320, 160},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			npc->bits &= ~NPC_SOLID_SOFT;
			npc->damage = 0;
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ym = -0x200;

			if (npc->direct == 0)
				npc->xm = 0x40;
			else
				npc->xm = -0x40;

			PlaySoundObject(54, SOUND_MODE_PLAY);
			break;

		case 1:
			if (npc->flag & 8)
			{
				npc->ani_no = 1;
				npc->ani_wait = 0;
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			npc->xm = 8 * npc->xm / 9;

			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

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
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Mesa block
void ActNpc319(NPCHAR *npc)
{
	RECT rc[3] = {
		{16, 0, 32, 16},
		{16, 0, 32, 16},
		{96, 80, 112, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y = npc->pNpc->y + (10 * 0x200);

			if (npc->pNpc->direct == 0)
				npc->x = npc->pNpc->x + (7 * 0x200);
			else
				npc->x = npc->pNpc->x - (7 * 0x200);

			if (npc->pNpc->code_char == 318)
			{
				SetDestroyNpChar(npc->x, npc->y, 0, 3);
				npc->cond = 0;
				return;
			}

			if (npc->pNpc->ani_no != 2)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ym = -0x400;
				npc->y = npc->pNpc->y - (4 * 0x200);

				if (npc->pNpc->direct == 0)
					npc->xm = -0x400;
				else
					npc->xm = 0x400;
			}

			break;

		case 2:
			if (++npc->act_wait == 4)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			npc->ym += 0x2A;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 8)
			{
				PlaySoundObject(12, SOUND_MODE_PLAY);
				SetDestroyNpChar(npc->x, npc->y, 0, 3);
				npc->cond = 0;
			}

			break;
	}

	if (++npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rc[npc->ani_no];
}

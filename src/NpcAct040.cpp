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
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Santa
void ActNpc040(NPCHAR *npc)
{
	RECT rcLeft[7] = {
		{0, 32, 16, 48},
		{16, 32, 32, 48},
		{32, 32, 48, 48},
		{0, 32, 16, 48},
		{48, 32, 64, 48},
		{0, 32, 16, 48},
		{64, 32, 80, 48},
	};

	RECT rcRight[7] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
		{0, 48, 16, 64},
		{48, 48, 64, 64},
		{0, 48, 16, 64},
		{64, 48, 80, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 1:
			if (Random(0, 120) == 10)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			if (npc->x - (32 * 0x200) < gMC.x && npc->x + (32 * 0x200) > gMC.x && npc->y - (32 * 0x200) < gMC.y && npc->y + (16 * 0x200) > gMC.y)
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

		case 3:
			npc->act_no = 4;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 4:
			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				npc->ani_no++;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			if (npc->direct == 0)
				npc->x -= 1 * 0x200;
			else
				npc->x += 1 * 0x200;

			break;

		case 5:
			npc->ani_no = 6;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Busted Door
void ActNpc041(NPCHAR *npc)
{
	RECT rect = {0, 80, 48, 112};

	if (npc->act_no == 0)
	{
		++npc->act_no;
		npc->y -= 1 * 0x10 * 0x200;	// Move one tile up
	}

	npc->rect = rect;
}

// Sue
void ActNpc042(NPCHAR *npc)
{
	int n;

	RECT rcLeft[13] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
		{32, 0, 48, 16},
		{0, 0, 16, 16},
		{48, 0, 64, 16},
		{0, 0, 16, 16},
		{64, 0, 80, 16},
		{80, 32, 96, 48},
		{96, 32, 112, 48},
		{128, 32, 144, 48},
		{0, 0, 16, 16},
		{112, 32, 128, 48},
		{160, 32, 176, 48},
	};

	RECT rcRight[13] = {
		{0, 16, 16, 32},
		{16, 16, 32, 32},
		{32, 16, 48, 32},
		{0, 16, 16, 32},
		{48, 16, 64, 32},
		{0, 16, 16, 32},
		{64, 16, 80, 32},
		{80, 48, 96, 64},
		{96, 48, 112, 64},
		{128, 48, 144, 64},
		{0, 16, 16, 32},
		{112, 48, 128, 64},
		{160, 48, 176, 64},
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

		case 3:
			npc->act_no = 4;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 4:
			if (++npc->ani_wait > 4)
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

			break;

		case 5:
			npc->ani_no = 6;
			npc->xm = 0;
			break;

		case 6:
			PlaySoundObject(50, SOUND_MODE_PLAY);
			npc->act_wait = 0;
			npc->act_no = 7;
			npc->ani_no = 7;
			// Fallthrough
		case 7:
			if (++npc->act_wait > 10)
				npc->act_no = 0;

			break;

		case 8:
			PlaySoundObject(50, SOUND_MODE_PLAY);
			npc->act_wait = 0;
			npc->act_no = 9;
			npc->ani_no = 7;
			npc->ym = -0x200;

			if (npc->direct == 0)
				npc->xm = 0x400;
			else
				npc->xm = -0x400;

			// Fallthrough
		case 9:
			if (++npc->act_wait > 3 && npc->flag & 8)
			{
				npc->act_no = 10;

				if (npc->direct == 0)
					npc->direct = 2;
				else
					npc->direct = 0;
			}

			break;

		case 10:
			npc->xm = 0;
			npc->ani_no = 8;
			break;

		case 11:
			npc->act_no = 12;
			npc->act_wait = 0;
			npc->ani_no = 9;
			npc->ani_wait = 0;
			npc->xm = 0;
			// Fallthrough
		case 12:
			if (++npc->ani_wait > 8)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 10)
				npc->ani_no = 9;

			break;

		case 13:
			npc->ani_no = 11;
			npc->xm = 0;
			npc->ym = 0;
			npc->act_no = 14;

			for (n = 0; n < NPC_MAX; ++n)
				if (gNPC[n].code_event == 501)
					break;

			if (n == NPC_MAX)
			{
				npc->act_no = 0;
				break;
			}

			npc->pNpc = &gNPC[n];
			// Fallthrough
		case 14:
			if (npc->pNpc->direct == 0)
				npc->direct = 2;
			else
				npc->direct = 0;

			if (npc->pNpc->direct == 0)
				npc->x = npc->pNpc->x - (6 * 0x200);
			else
				npc->x = npc->pNpc->x + (6 * 0x200);

			npc->y = npc->pNpc->y + (4 * 0x200);

			if (npc->pNpc->ani_no == 2 || npc->pNpc->ani_no == 4)
				npc->y -= 1 * 0x200;

			break;

		case 15:
			npc->act_no = 16;
			SetNpChar(257, npc->x + (128 * 0x200), npc->y, 0, 0, 0, NULL, 0);
			SetNpChar(257, npc->x + (128 * 0x200), npc->y, 0, 0, 2, NULL, 0x80);
			npc->xm = 0;
			npc->ani_no = 0;
			// Fallthrough
		case 16:
			gSuperXpos = npc->x - (24 * 0x200);
			gSuperYpos = npc->y - (8 * 0x200);
			break;

		case 17:
			npc->xm = 0;
			npc->ani_no = 12;
			gSuperXpos = npc->x;
			gSuperYpos = npc->y - (8 * 0x200);
			break;

		case 20:
			npc->act_no = 21;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 21:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			if (npc->direct == 0)
				npc->xm = -0x400;
			else
				npc->xm = 0x400;

			if (npc->x < gMC.x - (8 * 0x200))
			{
				npc->direct = 2;
				npc->act_no = 0;
			}

			break;

		case 30:
			npc->act_no = 31;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 31:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			if (npc->direct == 0)
				npc->xm = -0x400;
			else
				npc->xm = 0x400;

			break;

		case 40:
			npc->act_no = 41;
			npc->ani_no = 9;
			npc->ym = -0x400;
			break;
	}

	if (npc->act_no != 14)
	{
		npc->ym += 0x40;

		if (npc->xm > 0x400)
			npc->xm = 0x400;
		if (npc->xm < -0x400)
			npc->xm = -0x400;

		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->x += npc->xm;
		npc->y += npc->ym;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Chalkboard
void ActNpc043(NPCHAR *npc)
{
	RECT rcLeft = {128, 80, 168, 112};
	RECT rcRight = {168, 80, 208, 112};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 1 * 0x10 * 0x200;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight;
}

// Polish
void ActNpc044(NPCHAR *npc)
{
	// Yeah, Pixel defined these backwards for some reason.
	RECT rcRight[3] = {
		{0, 0, 32, 32},
		{32, 0, 64, 32},
		{64, 0, 96, 32},
	};

	RECT rcLeft[3] = {
		{0, 0, 32, 32},
		{96, 0, 128, 32},
		{128, 0, 160, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->ani_no = 0;

			if (npc->direct == 0)
				npc->act_no = 8;
			else
				npc->act_no = 2;
			// Fallthrough
		case 2:
			npc->ym += 0x20;

			if (npc->ym > 0 && npc->flag & 8)
			{
				npc->ym = -0x100;
				npc->xm += 0x100;
			}

			if (npc->flag & 4)
				npc->act_no = 3;

			break;

		case 3:
			npc->xm += 0x20;

			if (npc->xm > 0 && npc->flag & 4)
			{
				npc->xm = -0x100;
				npc->ym -= 0x100;
			}

			if (npc->flag & 2)
				npc->act_no = 4;

			break;

		case 4:
			npc->ym -= 0x20;

			if (npc->ym < 0 && npc->flag & 2)
			{
				npc->ym = 0x100;
				npc->xm -= 0x100;
			}

			if (npc->flag & 1)
				npc->act_no = 5;

			break;

		case 5:
			npc->xm -= 0x20;

			if (npc->xm < 0 && npc->flag & 1)
			{
				npc->xm = 0x100;
				npc->ym += 0x100;
			}

			if (npc->flag & 8)
				npc->act_no = 2;

			break;

		case 6:
			npc->ym += 0x20;

			if (npc->ym > 0 && npc->flag & 8)
			{
				npc->ym = -0x100;
				npc->xm -= 0x100;
			}

			if (npc->flag & 1)
				npc->act_no = 7;

			break;

		case 7:
			npc->xm -= 0x20;

			if (npc->xm < 0 && npc->flag & 1)
			{
				npc->xm = 0x100;
				npc->ym -= 0x100;
			}

			if (npc->flag & 2)
				npc->act_no = 8;

			break;

		case 8:
			npc->ym -= 0x20;

			if (npc->ym < 0 && npc->flag & 2)
			{
				npc->ym = 0x100;
				npc->xm += 0x100;
			}

			if (npc->flag & 4)
				npc->act_no = 9;

			break;

		case 9:
			npc->xm += 0x20;

			if (npc->xm > 0 && npc->flag & 4)
			{
				npc->xm = -0x100;
				npc->ym += 0x100;
			}

			if (npc->flag & 8)
				npc->act_no = 6;

			break;
	}

	if (npc->life <= 100)
	{
		int i;

		for (i = 0; i < 10; ++i)
			SetNpChar(45, npc->x, npc->y, 0, 0, 0, NULL, 0x100);

		SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
		PlaySoundObject(25, SOUND_MODE_PLAY);
		npc->cond = 0;
	}

	if (npc->xm > 0x200)
		npc->xm = 0x200;
	if (npc->xm < -0x200)
		npc->xm = -0x200;

	if (npc->ym > 0x200)
		npc->ym = 0x200;
	if (npc->ym < -0x200)
		npc->ym = -0x200;

	if (npc->shock)
	{
		npc->x += npc->xm / 2;
		npc->y += npc->ym / 2;
	}
	else
	{
		npc->x += npc->xm;
		npc->y += npc->ym;
	}

	if (npc->act_no >= 2 && npc->act_no <= 9 && ++npc->ani_no > 2)
		npc->ani_no = 1;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Baby
void ActNpc045(NPCHAR *npc)
{
	RECT rect[3] = {
		{0, 32, 16, 48},
		{16, 32, 32, 48},
		{32, 32, 48, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 2;

			if (Random(0, 1))
				npc->xm = Random(-0x200, -0x100);
			else
				npc->xm = Random(0x100, 0x200);

			if (Random(0, 1))
				npc->ym = Random(-0x200, -0x100);
			else
				npc->ym = Random(0x100, 0x200);

			npc->xm2 = npc->xm;
			npc->ym2 = npc->ym;
			// Fallthrough
		case 1:
		case 2:
			if (++npc->ani_no > 2)
				npc->ani_no = 1;

			break;
	}

	if (npc->xm2 < 0 && npc->flag & 1)
		npc->xm2 *= -1;
	if (npc->xm2 > 0 && npc->flag & 4)
		npc->xm2 *= -1;

	if (npc->ym2 < 0 && npc->flag & 2)
		npc->ym2 *= -1;
	if (npc->ym2 > 0 && npc->flag & 8)
		npc->ym2 *= -1;

	if (npc->xm2 > 0x200)
		npc->xm2 = 0x200;
	if (npc->xm2 < -0x200)
		npc->xm2 = -0x200;

	if (npc->ym2 > 0x200)
		npc->ym2 = 0x200;
	if (npc->ym2 < -0x200)
		npc->ym2 = -0x200;

	if (npc->shock)
	{
		npc->x += npc->xm2 / 2;
		npc->y += npc->ym2 / 2;
	}
	else
	{
		npc->x += npc->xm2;
		npc->y += npc->ym2;
	}

	npc->rect = rect[npc->ani_no];
}

// H/V Trigger
void ActNpc046(NPCHAR *npc)
{
	RECT rect = {0, 0, 16, 16};

	npc->bits |= NPC_EVENT_WHEN_TOUCHED;

	if (npc->direct == 0)
	{
		if (npc->x < gMC.x)
			npc->x += 0x5FF;
		else
			npc->x -= 0x5FF;
	}
	else
	{
		if (npc->y < gMC.y)
			npc->y += 0x5FF;
		else
			npc->y -= 0x5FF;
	}

	npc->rect = rect;
}

// Sandcroc
void ActNpc047(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 0;
			npc->act_no = 1;
			npc->act_wait = 0;
			npc->tgt_y = npc->y;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->bits &= ~NPC_INVULNERABLE;
			npc->bits &= ~NPC_SOLID_SOFT;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 1:
			if (gMC.x > npc->x - (8 * 0x200) && gMC.x < npc->x + (8 * 0x200) && gMC.y > npc->y && gMC.y < npc->y + (8 * 0x200))
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				PlaySoundObject(102, SOUND_MODE_PLAY);
			}

			if (npc->x < gMC.x)
				npc->x += 2 * 0x200;

			if (npc->x > gMC.x)
				npc->x -= 2 * 0x200;

			break;

		case 2:
			if (++npc->ani_wait > 3)
			{
				++npc->ani_no;
				npc->ani_wait = 0;
			}

			if (npc->ani_no == 3)
				npc->damage = 10;

			if (npc->ani_no == 4)
			{
				npc->bits |= NPC_SHOOTABLE;
				npc->act_no = 3;
				npc->act_wait = 0;
			}

			break;

		case 3:
			npc->bits |= NPC_SOLID_SOFT;
			npc->damage = 0;
			++npc->act_wait;

			if (npc->shock)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
			}

			break;

		case 4:
			npc->bits |= NPC_IGNORE_SOLIDITY;
			npc->y += 1 * 0x200;

			if (++npc->act_wait == 32)
			{
				npc->bits &= ~NPC_SOLID_SOFT;
				npc->bits &= ~NPC_SHOOTABLE;
				npc->act_no = 5;
				npc->act_wait = 0;
			}

			break;

		case 5:
			if (npc->act_wait < 100)
			{
				++npc->act_wait;
			}
			else
			{
				npc->y = npc->tgt_y;
				npc->ani_no = 0;
				npc->act_no = 0;
			}

			break;
	}

	RECT rect[5] = {
		{0, 48, 48, 80},
		{48, 48, 96, 80},
		{96, 48, 144, 80},
		{144, 48, 192, 80},
		{192, 48, 240, 80},
	};

	npc->rect = rect[npc->ani_no];
}

// Omega projectiles
void ActNpc048(NPCHAR *npc)
{
	if (npc->flag & 1 && npc->xm < 0)
	{
		npc->xm *= -1;
	}
	else if (npc->flag & 4 && npc->xm > 0)
	{
		npc->xm *= -1;
	}
	else if (npc->flag & 8)
	{
		if (++npc->count1 > 2 || npc->direct == 2)
		{
			VanishNpChar(npc);
			SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		}
		else
		{
			npc->ym = -0x100;
		}
	}

	if (npc->direct == 2)
	{
		npc->bits &= ~NPC_SHOOTABLE;
		npc->bits |= NPC_INVULNERABLE;
	}

	npc->ym += 5;
	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rcLeft[2] = {
		{288, 88, 304, 104},
		{304, 88, 320, 104},
	};

	RECT rcRight[2] = {
		{288, 104, 304, 120},
		{304, 104, 320, 120},
	};

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		if (++npc->ani_no > 1)
			npc->ani_no = 0;
	}

	if (++npc->act_wait > 750)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Skullhead
void ActNpc049(NPCHAR *npc)
{
	unsigned char deg;
	int xm, ym;

	if (npc->act_no >= 10 && npc->pNpc->code_char == 3)
	{
		npc->act_no = 3;
		npc->xm = 0;
		npc->ym = 0;
		npc->count2 = 1;
	}

	if (npc->flag & 1)
	{
		npc->direct = 2;
		npc->xm = 0x100;
	}

	if (npc->flag & 4)
	{
		npc->direct = 0;
		npc->xm = -0x100;
	}

	switch (npc->act_no)
	{
		case 0:
			if (npc->pNpc != NULL)
				npc->act_no = 10;
			else
				npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->act_wait > 3)
			{
				npc->ym = -0x400;
				npc->act_no = 3;
				npc->ani_no = 2;

				if (npc->count2 != 0)
				{
					if (npc->direct == 0)
						npc->xm = -0x200;
					else
						npc->xm = 0x200;
				}
				else
				{
					if (npc->direct == 0)
						npc->xm = -0x100;
					else
						npc->xm = 0x100;
				}
			}

			npc->ani_no = 1;
			break;

		case 3:
			if (npc->flag & 8)
			{
				npc->act_no = 1;
				npc->act_wait = 0;
				npc->xm = 0;
			}

			if (npc->flag & 8 || npc->ym > 0)
				npc->ani_no = 1;
			else
				npc->ani_no = 2;

			break;

		case 10:
			if (npc->count1 < 50)
			{
				++npc->count1;
			}
			else
			{
				if (npc->x - (128 * 0x200) < gMC.x && npc->x + (128 * 0x200) > gMC.x && npc->y - (96 * 0x200) < gMC.y && npc->y + (96 * 0x200) > gMC.y)
				{
					npc->act_no = 11;
					npc->act_wait = 0;
					npc->ani_no = 2;
				}
			}

			break;

		case 11:
			if (++npc->act_wait == 30 || npc->act_wait == 35)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y + (4 * 0x200) - gMC.y);
				ym = GetSin(deg) * 2;
				xm = GetCos(deg) * 2;
				SetNpChar(50, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			if (npc->act_wait > 50)
			{
				npc->count1 = 0;
				npc->act_no = 10;
				npc->ani_no = 1;
			}

			break;
	}

	if (npc->act_no >= 10)
	{
		npc->x = npc->pNpc->x;
		npc->y = npc->pNpc->y + (16 * 0x200);
		npc->direct = npc->pNpc->direct;
		--npc->pNpc->count1;
	}

	npc->ym += 0x40;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rcLeft[3] = {
		{0, 80, 32, 104},
		{32, 80, 64, 104},
		{64, 80, 96, 104},
	};

	RECT rcRight[3] = {
		{0, 104, 32, 128},
		{32, 104, 64, 128},
		{64, 104, 96, 128},
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Skeleton projectile
void ActNpc050(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			if (npc->direct == 2)
				npc->act_no = 2;
			// Fallthrough
		case 1:
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 1)
			{
				npc->act_no = 2;
				npc->xm = 0x200;
				++npc->count1;
			}

			if (npc->flag & 4)
			{
				npc->act_no = 2;
				npc->xm = -0x200;
				++npc->count1;
			}

			if (npc->flag & 2)
			{
				npc->act_no = 2;
				npc->ym = 0x200;
				++npc->count1;
			}

			if (npc->flag & 8)
			{
				npc->act_no = 2;
				npc->ym = -0x200;
				++npc->count1;
			}

			break;

		case 2:
			npc->ym += 0x40;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 8)
			{
				if (++npc->count1 > 1)
				{
					SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
					npc->cond = 0;
				}
			}

			break;
	}

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;
	if (npc->ym < -0x5FF)
		npc->ym = -0x5FF;

	RECT rect[4] = {
		{48, 32, 64, 48},
		{64, 32, 80, 48},
		{80, 32, 96, 48},
		{96, 32, 112, 48},
	};

	if (npc->direct == 0)
	{
		if (++npc->ani_wait > 1)
		{
			npc->ani_wait = 0;
			++npc->ani_no;
		}

		if (npc->ani_no > 3)
			npc->ani_no = 0;
	}
	else
	{
		if (++npc->ani_wait > 1)
		{
			npc->ani_wait = 0;
			--npc->ani_no;
		}

		if (npc->ani_no < 0)
			npc->ani_no = 3;
	}

	npc->rect = rect[npc->ani_no];
}

// Crow & Skullhead
void ActNpc051(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			if (npc->x - (((WINDOW_WIDTH / 2) + 160) * 0x200) < gMC.x && npc->x + (((WINDOW_WIDTH / 2) + 160) * 0x200) > gMC.x && npc->y - (((WINDOW_HEIGHT / 2) + 200) * 0x200) < gMC.y && npc->y + (((WINDOW_HEIGHT / 2) + 200) * 0x200) > gMC.y)
			{
				npc->tgt_x = npc->x;
				npc->tgt_y = npc->y;

				if (npc->direct == 0)	// Completely redundant as both the conditions are the same
					npc->ym = 0x400;
				else
					npc->ym = 0x400;

				npc->act_no = 1;
				SetNpChar(49, 0, 0, 0, 0, 0, npc, 0);
			}
			else
			{
				break;
			}

			// Fallthrough
		case 1:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->tgt_y < npc->y)
				npc->ym -= 10;
			if (npc->tgt_y > npc->y)
				npc->ym += 10;

			if (npc->ym > 0x200)
				npc->ym = 0x200;
			if (npc->ym < -0x200)
				npc->ym = -0x200;

			if (npc->count1 < 10)
				++npc->count1;
			else
				npc->act_no = 2;

			break;

		case 2:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->y > gMC.y + (32 * 0x200))
			{
				if (gMC.x < npc->x)
					npc->xm += 0x10;
				if (gMC.x > npc->x)
					npc->xm -= 0x10;
			}
			else
			{
				if (gMC.x < npc->x)
					npc->xm -= 0x10;
				if (gMC.x > npc->x)
					npc->xm += 0x10;
			}

			if (gMC.y < npc->y)
				npc->ym -= 0x10;
			if (gMC.y > npc->y)
				npc->ym += 0x10;

			if (npc->shock)
			{
				npc->ym += 0x20;
				npc->xm = 0;
			}

			break;
	}

	if (npc->xm < 0 && npc->flag & 1)
		npc->xm = 0x100;
	if (npc->xm > 0 && npc->flag & 4)
		npc->xm = -0x100;

	if (npc->ym < 0 && npc->flag & 2)
		npc->ym = 0x100;
	if (npc->ym > 0 && npc->flag & 8)
		npc->ym = -0x100;

	if (npc->xm > 0x400)
		npc->xm = 0x400;
	if (npc->xm < -0x400)
		npc->xm = -0x400;

	if (npc->ym > 0x200)
		npc->ym = 0x200;
	if (npc->ym < -0x200)
		npc->ym = -0x200;

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[5] = {
		{96, 80, 128, 112},
		{128, 80, 160, 112},
		{160, 80, 192, 112},
		{192, 80, 224, 112},
		{224, 80, 256, 112},
	};

	RECT rect_right[5] = {
		{96, 112, 128, 144},
		{128, 112, 160, 144},
		{160, 112, 192, 144},
		{192, 112, 224, 144},
		{224, 112, 256, 144},
	};

	if (npc->shock)
	{
		npc->ani_no = 4;
	}
	else if (npc->act_no == 2 && npc->y < gMC.y - (32 * 0x200))
	{
		npc->ani_no = 0;
	}
	else
	{
		if (npc->act_no != 0)
		{
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;
		}
	}

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Blue robot (sitting)
void ActNpc052(NPCHAR *npc)
{
	RECT rect = {240, 96, 256, 112};

	npc->rect = rect;
}

// Skullstep leg
void ActNpc053(NPCHAR *npc)
{
	unsigned char deg;

	RECT rcLeft[2] = {
		{0, 128, 24, 144},
		{24, 128, 48, 144},
	};

	RECT rcRight[2] = {
		{48, 128, 72, 144},
		{72, 128, 96, 144},
	};

	if (npc->pNpc->code_char == 3)
	{
		VanishNpChar(npc);
		SetDestroyNpChar(npc->x, npc->y, npc->view.back, 4);
		return;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->count1 = 10;
			// Fallthrough
		case 1:
			if (npc->direct == 0 && npc->flag & 0x20)
			{
				npc->pNpc->y -= 2 * 0x200;
				npc->pNpc->ym -= 0x100;
			}

			if (npc->direct == 2 && npc->flag & 0x10)
			{
				npc->pNpc->y -= 2 * 0x200;
				npc->pNpc->ym -= 0x100;
			}

			if (npc->flag & 8)
			{
				npc->pNpc->y -= 2 * 0x200;
				npc->pNpc->ym -= 0x100;

				if (npc->pNpc->direct == 0)
					npc->pNpc->xm -= 0x80;
				else
					npc->pNpc->xm += 0x80;
			}

			deg = (unsigned char)npc->xm + (unsigned char)npc->pNpc->count2;
			npc->x = npc->pNpc->x + npc->count1 * GetCos(deg);
			npc->y = npc->pNpc->y + npc->count1 * GetSin(deg);

			npc->direct = npc->pNpc->direct;

			break;
	}

	npc->direct = npc->pNpc->direct;

	if (deg >= 20 && deg <= 108)
		npc->ani_no = 0;
	else
		npc->ani_no = 1;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Skullstep
void ActNpc054(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{0, 80, 32, 104},
		{32, 80, 64, 104},
		{64, 80, 96, 104},
	};

	RECT rcRight[3] = {
		{0, 104, 32, 128},
		{32, 104, 64, 128},
		{64, 104, 96, 128},
	};

	unsigned char deg;

	switch (npc->act_no)
	{
		case 0:
			SetNpChar(53, 0, 0, 0, 0, npc->direct, npc, 0x100);
			SetNpChar(53, 0, 0, 128, 0, npc->direct, npc, 0);
			npc->act_no = 1;
			npc->ani_no = 1;
			// Fallthrough
		case 1:
			deg = npc->count2;

			if (npc->direct == 0)
				deg -= 6;
			else
				deg += 6;

			npc->count2 = deg;

			if (npc->flag & 8)
			{
				npc->xm = (npc->xm * 3) / 4;

				if (++npc->act_wait > 60)
				{
					npc->act_no = 2;
					npc->act_wait = 0;
				}
			}
			else
			{
				npc->act_wait = 0;
			}

			if (npc->direct == 0 && npc->flag & 1)
			{
				if (++npc->count1 > 8)
				{
					npc->direct = 2;
					npc->xm *= -1;
				}
			}
			else if (npc->direct == 2 && npc->flag & 4)
			{
				if (++npc->count1 > 8)
				{
					npc->direct = 0;
					npc->xm *= -1;
				}
			}
			else
			{
				npc->count1 = 0;
			}

			break;

		case 2:
			++npc->act_wait;
			npc->shock += (unsigned char)npc->act_wait;

			if (npc->act_wait > 50)
			{
				VanishNpChar(npc);
				SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
				PlaySoundObject(25, SOUND_MODE_PLAY);
			}

			break;
	}

	npc->ym += 0x80;

	if (npc->xm > 0x2FF)
		npc->xm = 0x2FF;
	if (npc->xm < -0x2FF)
		npc->xm = -0x2FF;

	if (npc->ym > 0x2FF)
		npc->ym = 0x2FF;
	if (npc->ym < -0x2FF)
		npc->ym = -0x2FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Kazuma
void ActNpc055(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{192, 192, 208, 216},
		{208, 192, 224, 216},
		{192, 192, 208, 216},
		{224, 192, 240, 216},
		{192, 192, 208, 216},
		{240, 192, 256, 216},
	};

	RECT rcRight[6] = {
		{192, 216, 208, 240},
		{208, 216, 224, 240},
		{192, 216, 208, 240},
		{224, 216, 240, 240},
		{192, 216, 208, 240},
		{240, 216, 256, 240},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			break;

		case 3:
			npc->act_no = 4;
			npc->ani_no = 1;
			npc->ani_wait = 0;
			// Fallthrough
		case 4:
			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 4)
				npc->ani_no = 1;

			if (npc->direct == 0)
				npc->x -= 1 * 0x200;
			else
				npc->x += 1 * 0x200;

			break;

		case 5:
			npc->ani_no = 5;
			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Beetle (Sand Zone)
void ActNpc056(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{0, 144, 16, 160},
		{16, 144, 32, 160},
		{32, 144, 48, 160},
	};

	RECT rcRight[3] = {
		{0, 160, 16, 176},
		{16, 160, 32, 176},
		{32, 160, 48, 176},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
				npc->act_no = 1;
			else
				npc->act_no = 3;

			break;

		case 1:
			npc->xm -= 0x10;

			if (npc->xm < -0x400)
				npc->xm = -0x400;

			if (npc->shock)
				npc->x += npc->xm / 2;
			else
				npc->x += npc->xm;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

			if (npc->flag & 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->xm = 0;
				npc->direct = 2;
			}

			break;

		case 2:
			if (npc->x < gMC.x && npc->x > gMC.x - (16 * 0x10 * 0x200) && npc->y < gMC.y + (8 * 0x200) && npc->y > gMC.y - (8 * 0x200))
			{
				npc->act_no = 3;
				npc->ani_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 3:
			npc->xm += 0x10;

			if (npc->xm > 0x400)
				npc->xm = 0x400;

			if (npc->shock)
				npc->x += npc->xm / 2;
			else
				npc->x += npc->xm;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

			if (npc->flag & 4)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->xm = 0;
				npc->direct = 0;
			}

			break;

		case 4:
			if (npc->x < gMC.x + (16 * 0x10 * 0x200) && npc->x > gMC.x && npc->y < gMC.y + (8 * 0x200) && npc->y > gMC.y - (8 * 0x200))
			{
				npc->act_no = 1;
				npc->ani_wait = 0;
				npc->ani_no = 1;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Crow
void ActNpc057(NPCHAR *npc)
{
	unsigned char deg;

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
			npc->ani_no = Random(0, 1);
			npc->ani_wait = Random(0, 4);
			// Fallthrough
		case 1:
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

			if (npc->shock)
			{
				npc->act_no = 2;
				npc->act_wait = 0;

				if (npc->direct == 2)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;

				npc->ym = 0;
			}

			break;

		case 2:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->y > gMC.y + (48 * 0x200))
			{
				if (gMC.x < npc->x)
					npc->xm += 0x10;
				if (gMC.x > npc->x)
					npc->xm -= 0x10;
			}
			else
			{
				if (gMC.x < npc->x)
					npc->xm -= 0x10;
				if (gMC.x > npc->x)
					npc->xm += 0x10;
			}

			if (gMC.y < npc->y)
				npc->ym -= 0x10;
			if (gMC.y > npc->y)
				npc->ym += 0x10;

			if (npc->shock)
			{
				npc->ym += 0x20;
				npc->xm = 0;
			}

			if (npc->xm < 0 && npc->flag & 1)
				npc->xm = 0x200;
			if (npc->xm > 0 && npc->flag & 4)
				npc->xm = -0x200;

			if (npc->ym < 0 && npc->flag & 2)
				npc->ym = 0x200;
			if (npc->ym > 0 && npc->flag & 8)
				npc->ym = -0x200;

			if (npc->xm > 0x5FF)
				npc->xm = 0x5FF;
			if (npc->xm < -0x5FF)
				npc->xm = -0x5FF;

			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;
			if (npc->ym < -0x5FF)
				npc->ym = -0x5FF;

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[5] = {
		{96, 80, 128, 112},
		{128, 80, 160, 112},
		{160, 80, 192, 112},
		{192, 80, 224, 112},
		{224, 80, 256, 112},
	};

	RECT rect_right[5] = {
		{96, 112, 128, 144},
		{128, 112, 160, 144},
		{160, 112, 192, 144},
		{192, 112, 224, 144},
		{224, 112, 256, 144},
	};

	if (npc->shock)
	{
		npc->ani_no = 4;
	}
	else
	{
		if (++npc->ani_wait > 1)
		{
			npc->ani_wait = 0;
			++npc->ani_no;
		}

		if (npc->ani_no > 1)
			npc->ani_no = 0;
	}

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Basu (Egg Corridor)
void ActNpc058(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{192, 0, 216, 24},
		{216, 0, 240, 24},
		{240, 0, 264, 24},
	};

	RECT rcRight[3] = {
		{192, 24, 216, 48},
		{216, 24, 240, 48},
		{240, 24, 264, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			if (gMC.x < npc->x + (16 * 0x200) && gMC.x > npc->x - (16 * 0x200))
			{
				npc->bits |= NPC_SHOOTABLE;
				npc->ym = -0x100;
				npc->tgt_x = npc->x;
				npc->tgt_y = npc->y;
				npc->act_no = 1;
				npc->act_wait = 0;
				npc->count1 = npc->direct;
				npc->count2 = 0;
				npc->damage = 6;

				if (npc->direct == 0)
				{
					npc->x = gMC.x + (16 * 0x10 * 0x200);
					npc->xm = -0x2FF;
				}
				else
				{
					npc->x = gMC.x - (16 * 0x10 * 0x200);
					npc->xm = 0x2FF;
				}

				return;
			}

			npc->rect.right = 0;
			npc->damage = 0;
			npc->xm = 0;
			npc->ym = 0;
			npc->bits &= ~NPC_SHOOTABLE;

			return;

		case 1:
			if (npc->x > gMC.x)
			{
				npc->direct = 0;
				npc->xm -= 0x10;
			}
			else
			{
				npc->direct = 2;
				npc->xm += 0x10;
			}

			if (npc->flag & 1)
				npc->xm = 0x200;

			if (npc->flag & 4)
				npc->xm = -0x200;

			if (npc->y < npc->tgt_y)
				npc->ym += 8;
			else
				npc->ym -= 8;

			if (npc->xm > 0x2FF)
				npc->xm = 0x2FF;
			if (npc->xm < -0x2FF)
				npc->xm = -0x2FF;

			if (npc->ym > 0x100)
				npc->ym = 0x100;
			if (npc->ym < -0x100)
				npc->ym = -0x100;

			if (npc->shock)
			{
				npc->x += npc->xm / 2;
				npc->y += npc->ym / 2;
			}
			else
			{
				npc->x += npc->xm;
				npc->y += npc->ym;
			}

			if (gMC.x > npc->x + (400 * 0x200) || gMC.x < npc->x - (400 * 0x200))	// TODO: Maybe do something about this for widescreen
			{
				npc->act_no = 0;
				npc->xm = 0;
				npc->direct = npc->count1;
				npc->x = npc->tgt_x;
				npc->rect.right = 0;
				npc->damage = 0;
				return;
			}

			break;
	}

	if (npc->act_no != 0)	// This is always true
	{
		if (npc->act_wait < 150)
			++npc->act_wait;

		if (npc->act_wait == 150)
		{
			if ((++npc->count2 % 8) == 0 && npc->x < gMC.x + (160 * 0x200) && npc->x > gMC.x - (160 * 0x200))	// TODO: Maybe do something about this for widescreen
			{
				unsigned char deg;
				int xm;
				int ym;

				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-6, 6);
				ym = GetSin(deg) * 2;
				xm = GetCos(deg) * 2;
				SetNpChar(84, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			if (npc->count2 > 8)
			{
				npc->act_wait = 0;
				npc->count2 = 0;
			}
		}
	}

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	if (npc->act_wait > 120 && npc->act_wait / 2 % 2 == 1 && npc->ani_no == 1)
		npc->ani_no = 2;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Eye door
void ActNpc059(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{224, 16, 240, 40},
		{208, 80, 224, 104},
		{224, 80, 240, 104},
		{240, 80, 256, 104},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->x - (64 * 0x200) < gMC.x && npc->x + (64 * 0x200) > gMC.x && npc->y - (64 * 0x200) < gMC.y && npc->y + (64 * 0x200) > gMC.y)
			{
				npc->act_no = 2;
				npc->ani_wait = 0;
			}

			break;

		case 2:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no == 2)
				npc->act_no = 3;

			break;

		case 3:
			if (npc->x - (64 * 0x200) < gMC.x && npc->x + (64 * 0x200) > gMC.x && npc->y - (64 * 0x200) < gMC.y && npc->y + (64 * 0x200) > gMC.y)
			{
				// There probably used to be some commented-out code here
			}
			else
			{
				npc->act_no = 4;
				npc->ani_wait = 0;
			}

			break;

		case 4:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				--npc->ani_no;
			}

			if (npc->ani_no == 0)
				npc->act_no = 1;

			break;
	}

	if (npc->shock)
		npc->rect = rcLeft[3];
	else
		npc->rect = rcLeft[npc->ani_no];
}

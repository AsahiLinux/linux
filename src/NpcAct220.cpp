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

// Shovel Brigade
void ActNpc220(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{0, 64, 16, 80},
		{16, 64, 32, 80},
	};

	RECT rcRight[2] = {
		{0, 80, 16, 96},
		{16, 80, 32, 96},
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

// Shovel Brigade (walking)
void ActNpc221(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{0, 64, 16, 80},
		{16, 64, 32, 80},
		{32, 64, 48, 80},
		{0, 64, 16, 80},
		{48, 64, 64, 80},
		{0, 64, 16, 80},
	};

	RECT rcRight[6] = {
		{0, 80, 16, 96},
		{16, 80, 32, 96},
		{32, 80, 48, 96},
		{0, 80, 16, 96},
		{48, 80, 64, 96},
		{0, 80, 16, 96},
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

// Prison bars
void ActNpc222(NPCHAR *npc)
{
	RECT rc = {96, 168, 112, 200};

	if (npc->act_no == 0)
	{
		++npc->act_no;
		npc->y -= 8 * 0x200;
	}

	npc->rect = rc;
}

// Momorin
void ActNpc223(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{80, 192, 96, 216},
		{96, 192, 112, 216},
		{112, 192, 128, 216},
	};

	RECT rcRight[3] = {
		{80, 216, 96, 240},
		{96, 216, 112, 240},
		{112, 216, 128, 240},
	};

	switch (npc->act_no)
	{
		case 0:
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

		case 3:
			npc->ani_no = 2;
			break;
	}

	if (npc->act_no < 2 && gMC.y < npc->y + (16 * 0x200) && gMC.y > npc->y - (16 * 0x200))
	{
		if (gMC.x < npc->x)
			npc->direct = 0;
		else
			npc->direct = 2;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Chie
void ActNpc224(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{112, 32, 128, 48},
		{128, 32, 144, 48},
	};

	RECT rcRight[2] = {
		{112, 48, 128, 64},
		{128, 48, 144, 64},
	};

	switch (npc->act_no)
	{
		case 0:
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

	if (npc->act_no < 2 && gMC.y < npc->y + (16 * 0x200) && gMC.y > npc->y - (16 * 0x200))
	{
		if (gMC.x < npc->x)
			npc->direct = 0;
		else
			npc->direct = 2;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Megane
void ActNpc225(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{64, 64, 80, 80},
		{80, 64, 96, 80},
	};

	RECT rcRight[2] = {
		{64, 80, 80, 96},
		{80, 80, 96, 96},
	};

	switch (npc->act_no)
	{
		case 0:
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

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Kanpachi
void ActNpc226(NPCHAR *npc)
{
	RECT rcRight[7] = {
		{256, 56, 272, 80},
		{272, 56, 288, 80},
		{288, 56, 304, 80},
		{256, 56, 272, 80},
		{304, 56, 320, 80},
		{256, 56, 272, 80},
		{240, 56, 256, 80},
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
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 11:
			npc->xm = 0x200;

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			++npc->act_wait;

			break;

		case 20:
			npc->xm = 0;
			npc->ani_no = 6;
			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rcRight[npc->ani_no];
}

// Bucket
void ActNpc227(NPCHAR *npc)
{
	RECT rc = {208, 32, 224, 48};
	npc->rect = rc;
}

// Droll (guard)
void ActNpc228(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{0, 0, 32, 40},
		{32, 0, 64, 40},
		{64, 0, 96, 40},
		{96, 0, 128, 40},
	};

	RECT rcRight[4] = {
		{0, 40, 32, 80},
		{32, 40, 64, 80},
		{64, 40, 96, 80},
		{96, 40, 128, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 8 * 0x200;
			// Fallthrough
		case 1:
			npc->xm = 0;
			npc->act_no = 2;
			npc->ani_no = 0;
			// Fallthrough
		case 2:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (++npc->ani_wait > 50)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 2;
			npc->act_wait = 0;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 10)
			{
				npc->act_no = 12;
				npc->ani_no = 3;
				npc->ym = -0x600;

				if (npc->direct == 0)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;
			}

			break;

		case 12:
			if (npc->flag & 8)
			{
				npc->ani_no = 2;
				npc->act_no = 13;
				npc->act_wait = 0;
			}

			break;

		case 13:
			npc->xm /= 2;

			if (++npc->act_wait > 10)
				npc->act_no = 1;

			break;
	}

	npc->ym += 0x40;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Red Flowers (sprouts)
void ActNpc229(NPCHAR *npc)
{
	RECT rc[2] = {
		{0, 96, 48, 112},
		{0, 112, 48, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y -= 16 * 0x200;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rc[0];
	else
		npc->rect = rc[1];
}

// Red Flowers (blooming)
void ActNpc230(NPCHAR *npc)
{
	RECT rc[2] = {
		{48, 96, 96, 128},
		{96, 96, 144, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->x -= 16 * 0x200;
			npc->y -= 16 * 0x200;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rc[0];
	else
		npc->rect = rc[1];
}

// Rocket
void ActNpc231(NPCHAR *npc)
{
	int i;

	RECT rc[2] = {
		{176, 32, 208, 48},
		{176, 48, 208, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->ani_no = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			// Fallthrough
		case 11:
			++npc->act_wait;
			npc->ym += 8;

			if (npc->flag & 8)
			{
				if (npc->act_wait < 10)
					npc->act_no = 12;
				else
					npc->act_no = 1;
			}

			break;

		case 12:
			npc->bits &= ~NPC_INTERACTABLE;
			npc->act_no = 13;
			npc->act_wait = 0;
			npc->ani_no = 1;

			for (i = 0; i < 10; ++i)
			{
				SetNpChar(4, npc->x + (Random(-16, 16) * 0x200), npc->y + (Random(-8, 8) * 0x200), 0, 0, 0, NULL, 0x100);
				PlaySoundObject(12, SOUND_MODE_PLAY);	// Wait, it does this in a loop?
			}
			// Fallthrough
		case 13:
			npc->ym -= 8;

			++npc->act_wait;

			if (npc->act_wait % 2 == 0)
				SetCaret(npc->x - (10 * 0x200), npc->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);
			if (npc->act_wait % 2 == 1)
				SetCaret(npc->x + (10 * 0x200), npc->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);

			if (npc->act_wait % 4 == 1)
				PlaySoundObject(34, SOUND_MODE_PLAY);

			if (npc->flag & 2 || gMC.flag & 2 || npc->act_wait > 450)
			{
				if (npc->flag & 2 || gMC.flag & 2)
					npc->ym = 0;

				npc->act_no = 15;

				for (i = 0; i < 6; ++i)
				{
					SetNpChar(4, npc->x + (Random(-16, 16) * 0x200), npc->y + (Random(-8, 8) * 0x200), 0, 0, 0, NULL, 0x100);
					PlaySoundObject(12, SOUND_MODE_PLAY);	// Here it does it again...
				}
			}

			break;

		case 15:
			npc->ym += 8;
			++npc->act_wait;

			if (npc->ym < 0)
			{
				if (npc->act_wait % 8 == 0)
					SetCaret(npc->x - (10 * 0x200), npc->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);

				if (npc->act_wait % 8 == 4)
					SetCaret(npc->x + (10 * 0x200), npc->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);

				if (npc->act_wait % 16 == 1)
					PlaySoundObject(34, SOUND_MODE_PLAY);
			}

			if (npc->flag & 8)
			{
				npc->bits |= NPC_INTERACTABLE;
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;
	}

	if (npc->ym < -0x5FF)
		npc->ym = -0x5FF;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;

	npc->rect = rc[npc->ani_no];
}

// Orangebell
void ActNpc232(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			npc->ym = 0x200;

			for (i = 0; i < 8; ++i)
				SetNpChar(233, npc->x, npc->y, 0, 0, npc->direct, npc, 0x100);
			// Fallthrough
		case 1:
			if (npc->xm < 0 && npc->flag & 1)
				npc->direct = 2;
			if (npc->xm > 0 && npc->flag & 4)
				npc->direct = 0;

			if (npc->direct == 0)
				npc->xm = -0x100;
			else
				npc->xm = 0x100;

			if (npc->y < npc->tgt_y)
				npc->ym += 8;
			else
				npc->ym -= 8;

			if (npc->ym > 0x200)
				npc->ym = 0x200;
			if (npc->ym < -0x200)
				npc->ym = -0x200;

			if (++npc->ani_wait > 5)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 0;

			break;
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rcLeft[3] = {
		{128, 0, 160, 32},
		{160, 0, 192, 32},
		{192, 0, 224, 32},
	};

	RECT rcRight[3] = {
		{128, 32, 160, 64},
		{160, 32, 192, 64},
		{192, 32, 224, 64},
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Orangebell bat
void ActNpc233(NPCHAR *npc)
{
	unsigned char deg;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			deg = Random(0, 0xFF);
			npc->xm = GetCos(deg);
			deg = Random(0, 0xFF);
			npc->ym = GetSin(deg);

			npc->count1 = 120;
			npc->count2 = Random(-0x20, 0x20) * 0x200;
			// Fallthrough
		case 1:
			if (npc->pNpc->code_char == 232)
			{
				npc->tgt_x = npc->pNpc->x;
				npc->tgt_y = npc->pNpc->y;
				npc->direct = npc->pNpc->direct;
			}

			if (npc->tgt_x < npc->x)
				npc->xm -= 8;
			if (npc->tgt_x > npc->x)
				npc->xm += 8;

			if (npc->tgt_y + npc->count2 < npc->y)
				npc->ym -= 0x20;
			if (npc->tgt_y + npc->count2 > npc->y)
				npc->ym += 0x20;

			if (npc->xm > 0x400)
				npc->xm = 0x400;
			if (npc->xm < -0x400)
				npc->xm = -0x400;

			if (npc->ym > 0x400)
				npc->ym = 0x400;
			if (npc->ym < -0x400)
				npc->ym = -0x400;

			if (npc->count1 < 120)
			{
				++npc->count1;
				break;
			}

			if (npc->x - (8 * 0x200) < gMC.x && npc->x + (8 * 0x200) > gMC.x && npc->y < gMC.y && npc->y + (176 * 0x200) > gMC.y)
			{
				npc->xm /= 4;
				npc->ym = 0;
				npc->act_no = 3;
				npc->bits &= ~NPC_IGNORE_SOLIDITY;
			}

			break;

		case 3:
			npc->ym += 0x40;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (npc->flag & 8)
			{
				npc->ym = 0;
				npc->xm *= 2;
				npc->count1 = 0;
				npc->act_no = 1;
				npc->bits |= NPC_IGNORE_SOLIDITY;
			}

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rcLeft[4] = {
		{256, 0, 272, 16},
		{272, 0, 288, 16},
		{288, 0, 304, 16},
		{304, 0, 320, 16},
	};

	RECT rcRight[4] = {
		{256, 16, 272, 32},
		{272, 16, 288, 32},
		{288, 16, 304, 32},
		{304, 16, 320, 32},
	};

	if (npc->act_no == 3)
	{
		npc->ani_no = 3;
	}
	else
	{
		if (++npc->ani_wait > 1)
		{
			npc->ani_wait = 0;
			++npc->ani_no;
		}

		if (npc->ani_no > 2)
			npc->ani_no = 0;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Red Flowers (picked)
void ActNpc234(NPCHAR *npc)
{
	RECT rc[2] = {
		{144, 96, 192, 112},
		{144, 112, 192, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 16 * 0x200;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rc[0];
	else
		npc->rect = rc[1];
}

// Midorin
void ActNpc235(NPCHAR *npc)
{
	RECT rcLeft[4] = {
		{192, 96, 208, 112},
		{208, 96, 224, 112},
		{224, 96, 240, 112},
		{192, 96, 208, 112},
	};

	RECT rcRight[4] = {
		{192, 112, 208, 128},
		{208, 112, 224, 128},
		{224, 112, 240, 128},
		{192, 112, 208, 128},
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
			if (Random(0, 30) == 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			if (Random(0, 30) == 1)
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
				npc->xm = -0x400;
			else
				npc->xm = 0x400;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			if (++npc->act_wait > 64)
				npc->act_no = 0;

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->ani_no == 2)
		npc->hit.top = 5 * 0x200;
	else
		npc->hit.top = 4 * 0x200;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Gunfish
void ActNpc236(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{128, 64, 152, 88},
		{152, 64, 176, 88},
		{176, 64, 200, 88},
		{200, 64, 224, 88},
		{224, 64, 248, 88},
		{248, 64, 272, 88},
	};

	RECT rcRight[6] = {
		{128, 88, 152, 112},
		{152, 88, 176, 112},
		{176, 88, 200, 112},
		{200, 88, 224, 112},
		{224, 88, 248, 112},
		{248, 88, 272, 112},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = Random(0, 50);
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			npc->ym = 0;
			// Fallthrough
		case 1:
			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->ym = 0x200;
				npc->act_no = 2;
			}

			break;

		case 2:
			if (npc->x < gMC.x)
				npc->direct = 2;
			else
				npc->direct = 0;

			if (gMC.x < npc->x + (128 * 0x200) && gMC.x > npc->x - (128 * 0x200) && gMC.y < npc->y + (32 * 0x200) && gMC.y > npc->y - (160 * 0x200))
				++npc->act_wait;

			if (npc->act_wait > 80)
			{
				npc->act_no = 10;
				npc->act_wait = 0;
			}

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;

		case 10:
			if (++npc->act_wait > 20)
			{
				npc->act_wait = 0;
				npc->act_no = 20;
			}

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			break;

		case 20:
			if (++npc->act_wait > 60)
			{
				npc->act_wait = 0;
				npc->act_no = 2;
			}

			if (npc->act_wait % 10 == 3)
			{
				PlaySoundObject(39, SOUND_MODE_PLAY);

				if (npc->direct == 0)
					SetNpChar(237, npc->x - (8 * 0x200), npc->y - (8 * 0x200), -0x400, -0x400, 0, NULL, 0x100);
				else
					SetNpChar(237, npc->x + (8 * 0x200), npc->y - (8 * 0x200), 0x400, -0x400, 0, NULL, 0x100);
			}

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 4;

			break;
	}

	if (npc->y < npc->tgt_y)
		npc->ym += 0x10;
	else
		npc->ym -= 0x10;

	if (npc->ym > 0x100)
		npc->ym = 0x100;
	if (npc->ym < -0x100)
		npc->ym = -0x100;

	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Gunfish projectile
void ActNpc237(NPCHAR *npc)
{
	RECT rc = {312, 32, 320, 40};

	int i;
	BOOL bHit;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			bHit = FALSE;

			++npc->act_wait;

			if (npc->flag & 0xFF)
				bHit = TRUE;

			if (npc->act_wait > 10 && npc->flag & 0x100)
				bHit = TRUE;

			if (bHit)
			{
				for (i = 0; i < 5; ++i)
					SetCaret(npc->x, npc->y, CARET_BUBBLE, DIR_LEFT);

				PlaySoundObject(21, SOUND_MODE_PLAY);
				npc->cond = 0;
				return;
			}

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rc;
}

// Press (sideways)
void ActNpc238(NPCHAR *npc)
{
	int i;

	RECT rc[3] = {
		{184, 200, 208, 216},
		{208, 200, 232, 216},
		{232, 200, 256, 216},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			npc->view.front = 16 * 0x200;
			npc->view.back = 8 * 0x200;
			// Fallthrough
		case 1:
			if (npc->direct == 0 && gMC.x < npc->x && gMC.x > npc->x - (192 * 0x200) && gMC.y > npc->y - (4 * 0x200) && gMC.y < npc->y + (8 * 0x200))
			{
				npc->act_no = 10;
				npc->act_wait = 0;
				npc->ani_no = 2;
			}

			if (npc->direct == 2 && gMC.x > npc->x && gMC.x < npc->x + (192 * 0x200) && gMC.y > npc->y - (4 * 0x200) && gMC.y < npc->y + (8 * 0x200))
			{
				npc->act_no = 10;
				npc->act_wait = 0;
				npc->ani_no = 2;
			}

			break;

		case 10:
			npc->damage = 0x7F;

			if (npc->direct == 0)
				npc->x -= 6 * 0x200;
			else
				npc->x += 6 * 0x200;

			if (++npc->act_wait == 8)
			{
				npc->act_no = 20;
				npc->act_wait = 0;

				for (i = 0; i < 4; ++i)
				{
					SetNpChar(4, npc->x + (Random(-16, 16) * 0x200), npc->y + (Random(-8, 8) * 0x200), 0, 0, 0, NULL, 0x100);
					PlaySoundObject(12, SOUND_MODE_PLAY);
				}
			}

			break;

		case 20:
			npc->damage = 0;

			if (++npc->act_wait > 50)
			{
				npc->act_wait = 0;
				npc->act_no = 30;
			}

			break;

		case 30:
			npc->damage = 0;
			npc->ani_no = 1;

			if (++npc->act_wait == 12)
			{
				npc->act_no = 1;
				npc->act_wait = 0;
				npc->ani_no = 0;
			}

			if (npc->direct == 0)
				npc->x += 4 * 0x200;
			else
				npc->x -= 4 * 0x200;

			break;
	}

	if (npc->direct == 0 && gMC.x < npc->x)
		npc->hit.back = 16 * 0x200;
	else if (npc->direct == 2 && gMC.x > npc->x)
		npc->hit.back = 16 * 0x200;
	else
		npc->hit.back = 8 * 0x200;

	npc->rect = rc[npc->ani_no];
}

// Cage bars
void ActNpc239(NPCHAR *npc)
{
	RECT rcLeft = {192, 48, 256, 80};
	RECT rcRight = {96, 112, 144, 144};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
			{
				npc->x += 8 * 0x200;
				npc->y += 16 * 0x200;
			}
			else
			{
				npc->view.front = 24 * 0x200;
				npc->view.back = 24 * 0x200;
				npc->view.top = 8 * 0x200;
				npc->view.back = 24 * 0x200;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight;
}

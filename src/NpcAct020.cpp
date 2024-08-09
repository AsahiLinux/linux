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
#include "Frame.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Computer
void ActNpc020(NPCHAR *npc)
{
	RECT rcLeft = {288, 16, 320, 40};

	RECT rcRight[3] = {
		{288, 40, 320, 64},
		{288, 40, 320, 64},
		{288, 64, 320, 88},
	};

	if (++npc->ani_wait > 3)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight[npc->ani_no];
}

// Chest (open)
void ActNpc021(NPCHAR *npc)
{
	if (npc->act_no == 0)
	{
		npc->act_no = 1;

		if (npc->direct == 2)
			npc->y += 16 * 0x200;
	}

	RECT rect = {224, 40, 240, 48};

	npc->rect = rect;
}

// Teleporter
void ActNpc022(NPCHAR *npc)
{
	RECT rect[2] = {
		{240, 16, 264, 48},
		{248, 152, 272, 184},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 0;
			break;

		case 1:
			if (++npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	npc->rect = rect[npc->ani_no];
}

// Teleporter lights
void ActNpc023(NPCHAR *npc)
{
	RECT rect[8] = {
		{264, 16, 288, 20},
		{264, 20, 288, 24},
		{264, 24, 288, 28},
		{264, 28, 288, 32},
		{264, 32, 288, 36},
		{264, 36, 288, 40},
		{264, 40, 288, 44},
		{264, 44, 288, 48},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 7)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Power Critter
void ActNpc024(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{0, 0, 24, 24},
		{24, 0, 48, 24},
		{48, 0, 72, 24},
		{72, 0, 96, 24},
		{96, 0, 120, 24},
		{120, 0, 144, 24},
	};

	RECT rcRight[6] = {
		{0, 24, 24, 48},
		{24, 24, 48, 48},
		{48, 24, 72, 48},
		{72, 24, 96, 48},
		{96, 24, 120, 48},
		{120, 24, 144, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y += 3 * 0x200;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->act_wait >= 8 && npc->x - (128 * 0x200) < gMC.x && npc->x + (128 * 0x200) > gMC.x && npc->y - (128 * 0x200) < gMC.y && npc->y + (48 * 0x200) > gMC.y)
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

			if (npc->act_wait >= 8 && npc->x - (96 * 0x200) < gMC.x && npc->x + (96 * 0x200) > gMC.x && npc->y - (96 * 0x200) < gMC.y && npc->y + (48 * 0x200) > gMC.y)
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
				PlaySoundObject(108, SOUND_MODE_PLAY);

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				if (npc->direct == 0)
					npc->xm = -0x100;
				else
					npc->xm = 0x100;
			}

			break;

		case 3:
			if (npc->ym > 0x200)
			{
				npc->tgt_y = npc->y;
				npc->act_no = 4;
				npc->ani_no = 3;
				npc->act_wait = 0;
				npc->act_wait = 0;	// Pixel duplicated this line
			}

			break;

		case 4:
			if (npc->x < gMC.x)
				npc->direct = 2;
			else
				npc->direct = 0;

			++npc->act_wait;

			if (npc->flag & 7 || npc->act_wait > 100)
			{
				npc->damage = 12;
				npc->act_no = 5;
				npc->ani_no = 2;
				npc->xm /= 2;
				break;
			}

			if (npc->act_wait % 4 == 1)
				PlaySoundObject(110, SOUND_MODE_PLAY);

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
				PlaySoundObject(26, SOUND_MODE_PLAY);
				SetQuake(30);
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
		if (npc->x < gMC.x)
			npc->xm += 0x20;
		else
			npc->xm -= 0x20;

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

// Egg Corridor lift
void ActNpc025(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{256, 64, 288, 80},
		{256, 80, 288, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->x += 8 * 0x200;
			// Fallthrough
		case 1:
			if (++npc->act_wait > 150)
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 2:	// Identical to case 4
			if (++npc->act_wait <= 0x40)
			{
				npc->y -= 1 * 0x200;
			}
			else
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 3:
			if (++npc->act_wait > 150)
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 4:	// Identical to case 2
			if (++npc->act_wait <= 0x40)
			{
				npc->y -= 1 * 0x200;
			}
			else
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 5:
			if (++npc->act_wait > 150)
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 6:
			if (++npc->act_wait <= 0x40)
			{
				npc->y += 1 * 0x200;
			}
			else
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 7:
			if (++npc->act_wait > 150)
			{
				npc->act_wait = 0;
				++npc->act_no;
			}

			break;

		case 8:
			if (++npc->act_wait <= 0x40)
			{
				npc->y += 1 * 0x200;
			}
			else
			{
				npc->act_wait = 0;
				npc->act_no = 1;
			}

			break;
	}

	switch (npc->act_no)
	{
		case 2:
		case 4:
		case 6:
		case 8:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	npc->rect = rcLeft[npc->ani_no];
}

// Bat (Grasstown, flying)
void ActNpc026(NPCHAR *npc)
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

			if (npc->count1 < 120)
			{
				++npc->count1;
				break;
			}

			if (npc->x - (8 * 0x200) < gMC.x && npc->x + (8 * 0x200) > gMC.x && npc->y < gMC.y && npc->y + (96 * 0x200) > gMC.y)
			{
				npc->xm /= 2;
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

	RECT rect_left[4] = {
		{32, 80, 48, 96},
		{48, 80, 64, 96},
		{64, 80, 80, 96},
		{80, 80, 96, 96},
	};

	RECT rect_right[4] = {
		{32, 96, 48, 112},
		{48, 96, 64, 112},
		{64, 96, 80, 112},
		{80, 96, 96, 112},
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
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Death trap
void ActNpc027(NPCHAR *npc)
{
	RECT rcLeft[1] = {
		{96, 64, 128, 88}
	};

	npc->rect = rcLeft[npc->ani_no];
}

// Flying Critter (Grasstown)
void ActNpc028(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
		{48, 48, 64, 64},
		{64, 48, 80, 64},
		{80, 48, 96, 64},
	};

	RECT rcRight[6] = {
		{0, 64, 16, 80},
		{16, 64, 32, 80},
		{32, 64, 48, 80},
		{48, 64, 64, 80},
		{64, 64, 80, 80},
		{80, 64, 96, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y += 3 * 0x200;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->act_wait >= 8 && npc->x - (128 * 0x200) < gMC.x && npc->x + (128 * 0x200) > gMC.x && npc->y - (128 * 0x200) < gMC.y && npc->y + (48 * 0x200) > gMC.y)
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

			if (npc->act_wait >= 8 && npc->x - (96 * 0x200) < gMC.x && npc->x + (96 * 0x200) > gMC.x && npc->y - (96 * 0x200) < gMC.y && npc->y + (48 * 0x200) > gMC.y)
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
				npc->ym = -0x4CC;
				PlaySoundObject(30, SOUND_MODE_PLAY);

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				if (npc->direct == 0)
					npc->xm = -0x100;
				else
					npc->xm = 0x100;
			}

			break;

		case 3:
			if (npc->ym > 0x100)
			{
				npc->tgt_y = npc->y;
				npc->act_no = 4;
				npc->ani_no = 3;
				npc->act_wait = 0;
				npc->act_wait = 0;	// Pixel duplicated this line
			}

			break;

		case 4:
			if (npc->x < gMC.x)
				npc->direct = 2;
			else
				npc->direct = 0;

			++npc->act_wait;

			if (npc->flag & 7 || npc->act_wait > 100)
			{
				npc->damage = 3;
				npc->act_no = 5;
				npc->ani_no = 2;
				npc->xm /= 2;
				break;
			}

			if (npc->act_wait % 4 == 1)
				PlaySoundObject(109, SOUND_MODE_PLAY);

			if (npc->flag & 8)
				npc->ym = -0x200;

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
		npc->ym += 0x40;
		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;
	}
	else
	{
		if (npc->x < gMC.x)
			npc->xm += 0x20;
		else
			npc->xm -= 0x20;

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

// Cthulhu
void ActNpc029(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{0, 192, 16, 216},
		{16, 192, 32, 216},
	};

	RECT rcRight[2] = {
		{0, 216, 16, 240},
		{16, 216, 32, 240},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 1:
			if (npc->x - (48 * 0x200) < gMC.x && npc->x + (48 * 0x200) > gMC.x && npc->y - (48 * 0x200) < gMC.y && npc->y + (16 * 0x200) > gMC.y)
				npc->ani_no = 1;
			else
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Gunsmith
void ActNpc030(NPCHAR *npc)
{
	RECT rc[3] = {
		{48, 0, 64, 16},
		{48, 16, 64, 32},
		{0, 32, 16, 48},
	};

	if (npc->direct == 0)
	{
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
	}
	else
	{
		if (npc->act_no == 0)
		{
			npc->act_no = 1;
			npc->y += 16 * 0x200;
			npc->ani_no = 2;
		}

		if (++npc->act_wait > 100)
		{
			npc->act_wait = 0;
			SetCaret(npc->x, npc->y - (2 * 0x200), CARET_ZZZ, DIR_LEFT);
		}
	}

	npc->rect = rc[npc->ani_no];
}

// Bat (Grasstown, hanging)
void ActNpc031(NPCHAR *npc)
{
	RECT rcLeft[5] = {
		{0, 80, 16, 96},
		{16, 80, 32, 96},
		{32, 80, 48, 96},
		{48, 80, 64, 96},
		{64, 80, 80, 96},
	};

	RECT rcRight[5] = {
		{0, 96, 16, 112},
		{16, 96, 32, 112},
		{32, 96, 48, 112},
		{48, 96, 64, 112},
		{64, 96, 80, 112},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (Random(0, 120) == 10)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			if (npc->x - (8 * 0x200) < gMC.x && npc->x + (8 * 0x200) > gMC.x && npc->y - (8 * 0x200) < gMC.y && npc->y + (96 * 0x200) > gMC.y)
			{
				npc->ani_no = 0;
				npc->act_no = 3;
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
			npc->ani_no = 0;

			if (npc->shock || npc->x - (20 * 0x200) > gMC.x || npc->x + (20 * 0x200) < gMC.x)
			{
				npc->ani_no = 1;
				npc->ani_wait = 0;
				npc->act_no = 4;
				npc->act_wait = 0;
			}

			break;

		case 4:
			npc->ym += 0x20;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (++npc->act_wait < 20 && !(npc->flag & 8))
				break;

			if (npc->flag & 8 || npc->y > gMC.y - (16 * 0x200))
			{
				npc->ani_wait = 0;
				npc->ani_no = 2;
				npc->act_no = 5;
				npc->tgt_y = npc->y;

				if (npc->flag & 8)
					npc->ym = -0x200;
			}

			break;

		case 5:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 4)
				npc->ani_no = 2;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (gMC.x < npc->x)
				npc->xm -= 0x10;
			if (gMC.x > npc->x)
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

			if (npc->flag & 8)
				npc->ym = -0x200;
			if (npc->flag & 2)
				npc->ym = 0x200;

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Life capsule
void ActNpc032(NPCHAR *npc)
{
	RECT rect[2] = {
		{32, 96, 48, 112},
		{48, 96, 64, 112},
	};

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Balrog bouncing projectile
void ActNpc033(NPCHAR *npc)
{
	if (npc->flag & 5)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}
	else if (npc->flag & 8)
	{
		npc->ym = -0x400;
	}

	npc->ym += 0x2A;

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[2] = {
		{240, 64, 256, 80},
		{240, 80, 256, 96},
	};

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;

		if (++npc->ani_no > 1)
			npc->ani_no = 0;
	}

	npc->rect = rect_left[npc->ani_no];

	if (++npc->act_wait > 250)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}
}

// Bed
void ActNpc034(NPCHAR *npc)
{
	RECT rcLeft = {192, 48, 224, 64};
	RECT rcRight = {192, 184, 224, 200};

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight;
}

// Mannan
void ActNpc035(NPCHAR *npc)
{
	if (npc->act_no < 3 && npc->life < 90)
	{
		PlaySoundObject(71, SOUND_MODE_PLAY);
		SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
		SetExpObjects(npc->x, npc->y, npc->exp);
		npc->act_no = 3;
		npc->act_wait = 0;
		npc->ani_no = 2;
		npc->bits &= ~NPC_SHOOTABLE;
		npc->damage = 0;
	}

	switch (npc->act_no)
	{
		case 0:
		case 1:
			if (npc->shock)
			{
				if (npc->direct == 0)
					SetNpChar(103, npc->x - (8 * 0x200), npc->y + (8 * 0x200), 0, 0, npc->direct, NULL, 0x100);
				else
					SetNpChar(103, npc->x + (8 * 0x200), npc->y + (8 * 0x200), 0, 0, npc->direct, NULL, 0x100);

				npc->ani_no = 1;
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			if (++npc->act_wait > 20)
			{
				npc->act_wait = 0;
				npc->act_no = 1;
				npc->ani_no = 0;
			}

			break;

		case 3:
			if (++npc->act_wait == 50 || npc->act_wait == 60)
				npc->ani_no = 3;

			if (npc->act_wait == 53 || npc->act_wait == 63)
				npc->ani_no = 2;

			if (npc->act_wait > 100)
				npc->act_no = 4;

			break;
	}

	RECT rcLeft[4] = {
		{96, 64, 120, 96},
		{120, 64, 144, 96},
		{144, 64, 168, 96},
		{168, 64, 192, 96},
	};

	RECT rcRight[4] = {
		{96, 96, 120, 128},
		{120, 96, 144, 128},
		{144, 96, 168, 128},
		{168, 96, 192, 128},
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Balrog (hover)
void ActNpc036(NPCHAR *npc)
{
	int i;
	unsigned char deg;
	int xm, ym;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->act_wait > 12)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->count1 = 3;
				npc->ani_no = 1;
			}

			break;

		case 2:
			if (++npc->act_wait > 16)
			{
				--npc->count1;
				npc->act_wait = 0;

				deg = GetArktan(npc->x - gMC.x, npc->y + (4 * 0x200) - gMC.y);
				deg += (unsigned char)Random(-0x10, 0x10);
				ym = GetSin(deg);
				xm = GetCos(deg);

				SetNpChar(11, npc->x, npc->y + 0x800, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);

				if (npc->count1 == 0)
				{
					npc->act_no = 3;
					npc->act_wait = 0;
				}
			}

			break;

		case 3:
			if (++npc->act_wait > 3)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
				npc->xm = (gMC.x - npc->x) / 100;
				npc->ym = -0x600;
				npc->ani_no = 3;
			}

			break;

		case 4:
			if (npc->ym > -0x200)
			{
				if (npc->life > 60)
				{
					npc->act_no = 5;
					npc->ani_no = 4;
					npc->ani_wait = 0;
					npc->act_wait = 0;
					npc->tgt_y = npc->y;
				}
				else
				{
					npc->act_no = 6;
				}
			}

			break;

		case 5:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
			{
				npc->ani_no = 4;
				PlaySoundObject(47, SOUND_MODE_PLAY);
			}

			if (++npc->act_wait > 100)
			{
				npc->act_no = 6;
				npc->ani_no = 3;
			}

			if (npc->y < npc->tgt_y)
				npc->ym += 0x40;
			else
				npc->ym -= 0x40;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x200)
				npc->ym = 0x200;

			break;

		case 6:
			if (npc->y + (16 * 0x200) < gMC.y)
				npc->damage = 10;
			else
				npc->damage = 0;

			if (npc->flag & 8)
			{
				npc->act_no = 7;
				npc->act_wait = 0;
				npc->ani_no = 2;
				PlaySoundObject(26, SOUND_MODE_PLAY);
				PlaySoundObject(25, SOUND_MODE_PLAY);
				SetQuake(30);
				npc->damage = 0;

				for (i = 0; i < 8; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				for (i = 0; i < 8; ++i)
					SetNpChar(33, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-0x400, 0x400), Random(-0x400, 0), 0, NULL, 0x100);
			}

			break;

		case 7:
			npc->xm = 0;

			if (++npc->act_wait > 3)
			{
				npc->act_no = 1;
				npc->act_wait = 0;
			}

			break;
	}

	if (npc->act_no != 5)
	{
		npc->ym += 0x200 / 10;

		if (npc->x < gMC.x)
			npc->direct = 2;
		else
			npc->direct = 0;
	}

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[6] = {
		{0, 0, 40, 24},
		{40, 0, 80, 24},
		{80, 0, 120, 24},
		{120, 0, 160, 24},
		{160, 48, 200, 72},
		{200, 48, 240, 72},
	};

	RECT rect_right[6] = {
		{0, 24, 40, 48},
		{40, 24, 80, 48},
		{80, 24, 120, 48},
		{120, 24, 160, 48},
		{160, 72, 200, 96},
		{200, 72, 240, 96},
	};

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Signpost
void ActNpc037(NPCHAR *npc)
{
	RECT rect[2] = {
		{192, 64, 208, 80},
		{208, 64, 224, 80},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Fireplace
void ActNpc038(NPCHAR *npc)
{
	RECT rect[4] = {
		{128, 64, 144, 80},
		{144, 64, 160, 80},
		{160, 64, 176, 80},
		{176, 64, 192, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 0;

			npc->rect = rect[npc->ani_no];
			break;

		case 10:
			npc->act_no = 11;
			SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
			// Fallthrough
		case 11:
			npc->rect.left = 0;
			npc->rect.right = 0;
			break;
	}
}

// Save sign
void ActNpc039(NPCHAR *npc)
{
	RECT rect[2] = {
		{224, 64, 240, 80},
		{240, 64, 256, 80},
	};

	if (npc->direct == 0)
		npc->ani_no = 0;
	else
		npc->ani_no = 1;

	npc->rect = rect[npc->ani_no];
}

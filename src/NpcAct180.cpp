// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "NpcAct.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Back.h"
#include "Bullet.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Flags.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "NpcHit.h"
#include "Sound.h"
#include "Triangle.h"

// Curly AI
void ActNpc180(NPCHAR *npc)
{
	int xx, yy;

	RECT rcLeft[11] = {
		{0, 96, 16, 112},
		{16, 96, 32, 112},
		{0, 96, 16, 112},
		{32, 96, 48, 112},
		{0, 96, 16, 112},
		{48, 96, 64, 112},
		{64, 96, 80, 112},
		{48, 96, 64, 112},
		{80, 96, 96, 112},
		{48, 96, 64, 112},
		{144, 96, 160, 112},
	};

	RECT rcRight[11] = {
		{0, 112, 16, 128},
		{16, 112, 32, 128},
		{0, 112, 16, 128},
		{32, 112, 48, 128},
		{0, 112, 16, 128},
		{48, 112, 64, 128},
		{64, 112, 80, 128},
		{48, 112, 64, 128},
		{80, 112, 96, 128},
		{48, 112, 64, 128},
		{144, 112, 160, 128},
	};

	if (npc->y < gMC.y - (10 * 0x10 * 0x200))
	{
		if (npc->y < 16 * 0x10 * 0x200)
		{
			npc->tgt_x = 320 * 0x10 * 0x200;
			npc->tgt_y = npc->y;
		}
		else
		{
			npc->tgt_x = 0;
			npc->tgt_y = npc->y;
		}
	}
	else
	{
		if (gCurlyShoot_wait != 0)
		{
			npc->tgt_x = gCurlyShoot_x;
			npc->tgt_y = gCurlyShoot_y;
		}
		else
		{
			npc->tgt_x = gMC.x;
			npc->tgt_y = gMC.y;
		}
	}

	if (npc->xm < 0 && npc->flag & 1)
		npc->xm = 0;
	if (npc->xm > 0 && npc->flag & 4)
		npc->xm = 0;

	switch (npc->act_no)
	{
		case 20:
			npc->x = gMC.x;
			npc->y = gMC.y;
			npc->act_no = 100;
			npc->ani_no = 0;
			SetNpChar(183, 0, 0, 0, 0, 0, npc, 0x100);

			if (GetNPCFlag(563))
				SetNpChar(182, 0, 0, 0, 0, 0, npc, 0x100);
			else
				SetNpChar(181, 0, 0, 0, 0, 0, npc, 0x100);

			break;

		case 40:
			npc->act_no = 41;
			npc->act_wait = 0;
			npc->ani_no = 10;
			// Fallthrough
		case 41:
			if (++npc->act_wait == 750)
			{
				npc->bits &= ~NPC_INTERACTABLE;
				npc->ani_no = 0;
			}

			if (npc->act_wait > 1000)
			{
				npc->act_no = 100;
				npc->ani_no = 0;
				SetNpChar(183, 0, 0, 0, 0, 0, npc, 0x100);

				if (GetNPCFlag(563))
					SetNpChar(182, 0, 0, 0, 0, 0, npc, 0x100);
				else
					SetNpChar(181, 0, 0, 0, 0, 0, npc, 0x100);
			}

			break;

		case 100:
			npc->ani_no = 0;
			npc->xm = (npc->xm * 7) / 8;
			npc->count1 = 0;

			if (npc->x > npc->tgt_x + (16 * 0x200))
			{
				npc->act_no = 200;
				npc->ani_no = 1;
				npc->direct = 0;
				npc->act_wait = Random(20, 60);
			}
			else if (npc->x < npc->tgt_x - (16 * 0x200))
			{
				npc->act_no = 300;
				npc->ani_no = 1;
				npc->direct = 2;
				npc->act_wait = Random(20, 60);
			}

			break;

		case 200:
			npc->xm -= 0x20;
			npc->direct = 0;

			if (npc->flag & 1)
				++npc->count1;
			else
				npc->count1 = 0;

			break;

		case 210:
			npc->xm -= 0x20;
			npc->direct = 0;

			if (npc->flag & 8)
				npc->act_no = 100;

			break;

		case 300:
			npc->xm += 0x20;
			npc->direct = 2;

			if (npc->flag & 4)
				++npc->count1;
			else
				npc->count1 = 0;

			break;

		case 310:
			npc->xm += 0x20;
			npc->direct = 2;

			if (npc->flag & 8)
				npc->act_no = 100;

			break;
	}

	if (gCurlyShoot_wait != 0)
		--gCurlyShoot_wait;

	if (gCurlyShoot_wait == 70)
		npc->count2 = 10;

	if (gCurlyShoot_wait == 60 && npc->flag & 8 && Random(0, 2))
	{
		npc->count1 = 0;
		npc->ym = -0x600;
		npc->ani_no = 1;
		PlaySoundObject(15, SOUND_MODE_PLAY);

		if (npc->x > npc->tgt_x)
			npc->act_no = 210;
		else
			npc->act_no = 310;
	}

	xx = npc->x - npc->tgt_x;
	yy = npc->y - npc->tgt_y;

	if (xx < 0)
		xx *= -1;

	if (npc->act_no == 100)
	{
		if (xx + (2 * 0x200) < yy)
			npc->ani_no = 5;
		else
			npc->ani_no = 0;
	}

	if (npc->act_no == 210 || npc->act_no == 310)
	{
		if (xx + (2 * 0x200) < yy)
			npc->ani_no = 6;
		else
			npc->ani_no = 1;
	}

	if (npc->act_no == 200 || npc->act_no == 300)
	{
		++npc->ani_wait;

		if (xx + (2 * 0x200) < yy)
			npc->ani_no = 6 + (npc->ani_wait / 4 % 4);
		else
			npc->ani_no = 1 + (npc->ani_wait / 4 % 4);

		if (npc->act_wait)
		{
			--npc->act_wait;

#ifdef FIX_BUGS
			// I assume this is what was intended
			if (npc->flag & 8 && npc->count1 > 10)
#else
			if (npc->flag && 8 && npc->count1 > 10)
#endif
			{
				npc->count1 = 0;
				npc->ym = -0x600;
				npc->act_no += 10;
				npc->ani_no = 1;
				PlaySoundObject(15, SOUND_MODE_PLAY);
			}
		}
		else
		{
			npc->act_no = 100;
			npc->ani_no = 0;
		}
	}

	if (npc->act_no >= 100 && npc->act_no < 500)
	{
		if (npc->x < gMC.x - (80 * 0x200) || npc->x > gMC.x + (80 * 0x200))
		{
#ifdef FIX_BUGS
			if (npc->flag & 5)
#else
			if (npc->flag && 5)
#endif
				npc->ym += 0x200 / 32;
			else
				npc->ym += 0x200 / 10;
		}
		else
		{
			npc->ym += 0x200 / 10;
		}
	}

	if (npc->xm > 0x300)
		npc->xm = 0x300;
	if (npc->xm < -0x300)
		npc->xm = -0x300;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->act_no >= 100 && !(npc->flag & 8))
	{
		switch (npc->ani_no)
		{
			case 1000:
				break;

			default:
				if (xx + (2 * 0x200) < yy)
					npc->ani_no = 6;
				else
					npc->ani_no = 1;

				break;
		}
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Curly AI Machine Gun
void ActNpc181(NPCHAR *npc)
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

	if (npc->pNpc->ani_no < 5)
	{
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
		npc->ani_no = 0;
	}
	else
	{
		if (npc->pNpc->direct == 0)
		{
			npc->direct = 0;
			npc->x = npc->pNpc->x;
		}
		else
		{
			npc->direct = 2;
			npc->x = npc->pNpc->x;
		}

		npc->y = npc->pNpc->y - (10 * 0x200);
		npc->ani_no = 1;
	}

	if (npc->pNpc->ani_no == 1 || npc->pNpc->ani_no == 3 || npc->pNpc->ani_no == 6 || npc->pNpc->ani_no == 8)
		npc->y -= 1 * 0x200;

	switch (npc->act_no)
	{
		case 0:
			if (npc->pNpc->count2 == 10)
			{
				npc->pNpc->count2 = 0;
				npc->act_no = 10;
				npc->act_wait = 0;
			}

			break;

		case 10:
			if (++npc->act_wait % 6 == 1)
			{
				if (npc->ani_no == 0)
				{
					if (npc->direct == 0)
					{
						SetBullet(12, npc->x - (4 * 0x200), npc->y + (3 * 0x200), 0);
						SetCaret(npc->x - (4 * 0x200), npc->y + (3 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
					else
					{
						SetBullet(12, npc->x + (4 * 0x200), npc->y + (3 * 0x200), 2);
						SetCaret(npc->x + (4 * 0x200), npc->y + (3 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
				}
				else
				{
					if (npc->direct == 0)
					{
						SetBullet(12, npc->x - (2 * 0x200), npc->y - (4 * 0x200), 1);
						SetCaret(npc->x - (2 * 0x200), npc->y - (4 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
					else
					{
						SetBullet(12, npc->x + (2 * 0x200), npc->y - (4 * 0x200), 1);
						SetCaret(npc->x + (2 * 0x200), npc->y - (4 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
				}
			}

			if (npc->act_wait == 60)
				npc->act_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Curly AI Polar Star
void ActNpc182(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{184, 152, 200, 168},
		{200, 152, 216, 168},
	};

	RECT rcRight[2] = {
		{184, 168, 200, 184},
		{200, 168, 216, 184},
	};

	if (npc->pNpc == NULL)
		return;

	if (npc->pNpc->ani_no < 5)
	{
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
		npc->ani_no = 0;
	}
	else
	{
		if (npc->pNpc->direct == 0)
		{
			npc->direct = 0;
			npc->x = npc->pNpc->x;
		}
		else
		{
			npc->direct = 2;
			npc->x = npc->pNpc->x;
		}

		npc->y = npc->pNpc->y - (10 * 0x200);
		npc->ani_no = 1;
	}

	if (npc->pNpc->ani_no == 1 || npc->pNpc->ani_no == 3 || npc->pNpc->ani_no == 6 || npc->pNpc->ani_no == 8)
		npc->y -= 1 * 0x200;

	switch (npc->act_no)
	{
		case 0:
			if (npc->pNpc->count2 == 10)
			{
				npc->pNpc->count2 = 0;
				npc->act_no = 10;
				npc->act_wait = 0;
			}

			break;

		case 10:
			if (++npc->act_wait % 12 == 1)
			{
				if (npc->ani_no == 0)
				{
					if (npc->direct == 0)
					{
						SetBullet(6, npc->x - (4 * 0x200), npc->y + (3 * 0x200), 0);
						SetCaret(npc->x - (4 * 0x200), npc->y + (3 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
					else
					{
						SetBullet(6, npc->x + (4 * 0x200), npc->y + (3 * 0x200), 2);
						SetCaret(npc->x + (4 * 0x200), npc->y + (3 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
				}
				else
				{
					if (npc->direct == 0)
					{
						SetBullet(6, npc->x - (2 * 0x200), npc->y - (4 * 0x200), 1);
						SetCaret(npc->x - (2 * 0x200), npc->y - (4 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
					else
					{
						SetBullet(6, npc->x + (2 * 0x200), npc->y - (4 * 0x200), 1);
						SetCaret(npc->x + (2 * 0x200), npc->y - (4 * 0x200), CARET_SHOOT, DIR_LEFT);
					}
				}
			}

			if (npc->act_wait == 60)
				npc->act_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Curly Air Tank Bubble
void ActNpc183(NPCHAR *npc)
{
	RECT rect[2] = {
		{56, 96, 80, 120},
		{80, 96, 104, 120},
	};

	if (npc->pNpc == NULL)
		return;

	switch (npc->act_no)
	{
		case 0:
			npc->x = npc->pNpc->x;
			npc->y = npc->pNpc->y;
			npc->act_no = 1;
			break;
	}

	npc->x += (npc->pNpc->x - npc->x) / 2;
	npc->y += (npc->pNpc->y - npc->y) / 2;

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	if (npc->pNpc->flag & 0x100)
		npc->rect = rect[npc->ani_no];
	else
		npc->rect.right = 0;
}

// Big Shutter
void ActNpc184(NPCHAR *npc)
{
	int i;

	RECT rc[4] = {
		{0, 64, 32, 96},
		{32, 64, 64, 96},
		{64, 64, 96, 96},
		{32, 64, 64, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->x += 8 * 0x200;
			npc->y += 8 * 0x200;
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 1;
			npc->act_wait = 0;
			npc->bits |= NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 11:
			switch (npc->direct)
			{
				case 0:
					npc->x -= 0x80;
					break;

				case 1:
					npc->y -= 0x80;
					break;

				case 2:
					npc->x += 0x80;
					break;

				case 3:
					npc->y += 0x80;
					break;
			}

			if ((++npc->act_wait % 8) == 0)
				PlaySoundObject(26, SOUND_MODE_PLAY);

			SetQuake(20);
			break;

		case 20:
			for (i = 0; i < 4; ++i)
				SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (16 * 0x200), Random(-0x155, 0x155), Random(-0x600, 0), 0, NULL, 0x100);

			npc->act_no = 1;
			break;
	}

	if (++npc->ani_wait > 10)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 3)
		npc->ani_no = 0;

	npc->rect = rc[npc->ani_no];
}

// Small Shutter
void ActNpc185(NPCHAR *npc)
{
	RECT rc = {96, 64, 112, 96};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 8 * 0x200;
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 1;
			npc->act_wait = 0;
			npc->bits |= NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 11:
			switch (npc->direct)
			{
				case 0:
					npc->x -= 0x80;
					break;

				case 1:
					npc->y -= 0x80;
					break;

				case 2:
					npc->x += 0x80;
					break;

				case 3:
					npc->y += 0x80;
					break;
			}

			++npc->act_wait;
			break;

		case 20:
			npc->y -= 24 * 0x200;
			npc->act_no = 1;
			break;
	}

	npc->rect = rc;
}

// Lift block
void ActNpc186(NPCHAR *npc)
{
	RECT rc[4] = {
		{48, 48, 64, 64},
		{64, 48, 80, 64},
		{80, 48, 96, 64},
		{64, 48, 80, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 1;
			npc->act_wait = 0;
			npc->bits |= NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 11:
			switch (npc->direct)
			{
				case 0:
					npc->x -= 0x80;
					break;

				case 1:
					npc->y -= 0x80;
					break;

				case 2:
					npc->x += 0x80;
					break;

				case 3:
					npc->y += 0x80;
					break;
			}

			++npc->act_wait;
			break;
	}

	if (++npc->ani_wait > 10)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 3)
		npc->ani_no = 0;

	npc->rect = rc[npc->ani_no];
}

// Fuzz Core
void ActNpc187(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			npc->count1 = 120;
			npc->act_wait = Random(0, 50);

			for (i = 0; i < 5; ++i)
				SetNpChar(188, 0, 0, 0, 0, 51 * i, npc, 0x100);
			// Fallthrough
		case 1:
			if (++npc->act_wait < 50)
				break;

			npc->act_wait = 0;
			npc->act_no = 2;
			npc->ym = 0x300;
			break;

		case 2:
			npc->count1 += 4;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->tgt_y < npc->y)
				npc->ym -= 0x10;
			if (npc->tgt_y > npc->y)
				npc->ym += 0x10;

			if (npc->ym > 0x355)
				npc->ym = 0x355;
			if (npc->ym < -0x355)
				npc->ym = -0x355;

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[2] = {
		{224, 104, 256, 136},
		{256, 104, 288, 136},
	};

	RECT rect_right[2] = {
		{224, 136, 256, 168},
		{256, 136, 288, 168},
	};

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Fuzz
void ActNpc188(NPCHAR *npc)
{
	unsigned char deg;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->count1 = npc->direct;
			// Fallthrough
		case 1:
			if (npc->pNpc->code_char == 187 && npc->pNpc->cond & 0x80)
			{
				deg = (npc->pNpc->count1 + npc->count1) % 0x100;
				npc->x = npc->pNpc->x + (GetSin(deg) * 20);
				npc->y = npc->pNpc->y + (GetCos(deg) * 0x20);
			}
			else
			{
				npc->xm = Random(-0x200, 0x200);
				npc->ym = Random(-0x200, 0x200);
				npc->act_no = 10;
			}

			break;

		case 10:
			if (gMC.x < npc->x)
				npc->xm -= 0x20;
			else
				npc->xm += 0x20;

			if (gMC.y < npc->y)
				npc->ym -= 0x20;
			else
				npc->ym += 0x20;

			if (npc->xm > 0x800)
				npc->xm = 0x800;
			if (npc->xm < -0x800)
				npc->xm = -0x800;

			if (npc->ym > 0x200)
				npc->ym = 0x200;
			if (npc->ym < -0x200)
				npc->ym = -0x200;

			npc->x += npc->xm;
			npc->y += npc->ym;

			break;
	}

	if (gMC.x < npc->x)
		npc->direct = 0;
	else
		npc->direct = 2;

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	RECT rect_left[2] = {
		{288, 104, 304, 120},
		{304, 104, 320, 120},
	};

	RECT rect_right[2] = {
		{288, 120, 304, 136},
		{304, 120, 320, 136},
	};

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Unused homing flame object (possibly related to the Core?)
void ActNpc189(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->xm = -0x40;
			// Fallthrough
		case 1:
			npc->y += npc->ym;

			if (++npc->act_wait > 0x100)
				npc->act_no = 10;

			break;

		case 10:
			if (gMC.x < npc->x)
				npc->xm -= 8;
			else
				npc->xm += 8;

			if (gMC.y < npc->y)
				npc->ym -= 8;
			else
				npc->ym += 8;

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

	if (gMC.x < npc->x)
		npc->direct = 0;
	else
		npc->direct = 2;

	if (++npc->ani_wait > 2)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	RECT rect[3] = {
		{224, 184, 232, 200},
		{232, 184, 240, 200},
		{240, 184, 248, 200},
	};

	npc->rect = rect[npc->ani_no];
}

// Broken robot
void ActNpc190(NPCHAR *npc)
{
	RECT rect[2] = {
		{192, 32, 208, 48},
		{208, 32, 224, 48},
	};

	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 0;
			break;

		case 10:
			PlaySoundObject(72, SOUND_MODE_PLAY);

			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x, npc->y + (Random(-8, 8) * 0x200), Random(-8, -2) * 0x200, Random(-3, 3) * 0x200, 0, NULL, 0x100);

			npc->cond = 0;
			break;

		case 20:
			if (++npc->ani_wait > 10)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	npc->rect = rect[npc->ani_no];
}

// Water level
void ActNpc191(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->tgt_y = npc->y;
			npc->ym = 0x200;
			// Fallthrough
		case 10:
			if (npc->y < npc->tgt_y)
				npc->ym += 4;
			else
				npc->ym -= 4;

			if (npc->ym < -0x100)
				npc->ym = -0x100;
			if (npc->ym > 0x100)
				npc->ym = 0x100;

			npc->y += npc->ym;
			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			// Fallthrough
		case 21:
			if (npc->y < npc->tgt_y)
				npc->ym += 4;
			else
				npc->ym -= 4;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x200)
				npc->ym = 0x200;

			npc->y += npc->ym;

			if (++npc->act_wait > 1000)
				npc->act_no = 22;

			break;

		case 22:
			if (npc->y < 0)
				npc->ym += 4;
			else
				npc->ym -= 4;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x200)
				npc->ym = 0x200;

			npc->y += npc->ym;

			if (npc->y < 64 * 0x200 || gSuperYpos != 0)
			{
				npc->act_no = 21;
				npc->act_wait = 0;
			}
			break;

		case 30:
			if (npc->y < 0)
				npc->ym += 4;
			else
				npc->ym -= 4;

			if (npc->ym < -0x200)
				npc->ym = -0x200;
			if (npc->ym > 0x100)
				npc->ym = 0x100;

			npc->y += npc->ym;
			break;
	}

	gWaterY = npc->y;

	npc->rect.right = 0;
	npc->rect.bottom = 0;
}

// Scooter
void ActNpc192(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->view.back = 16 * 0x200;
			npc->view.front = 16 * 0x200;
			npc->view.top = 8 * 0x200;
			npc->view.bottom = 8 * 0x200;
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 1;
			npc->view.top = 16 * 0x200;
			npc->view.bottom = 16 * 0x200;
			npc->y -= 5 * 0x200;
			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 1;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			// Fallthrough
		case 21:
			npc->x = npc->tgt_x + (Random(-1, 1) * 0x200);
			npc->y = npc->tgt_y + (Random(-1, 1) * 0x200);

			if (++npc->act_wait > 30)
				npc->act_no = 30;

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 1;
			npc->xm = -0x800;
			npc->x = npc->tgt_x;
			npc->y = npc->tgt_y;
			PlaySoundObject(44, SOUND_MODE_PLAY);
			// Fallthrough
		case 31:
			npc->xm += 0x20;
			npc->x += npc->xm;
			++npc->act_wait;
			npc->y = npc->tgt_y + (Random(-1, 1) * 0x200);

			if (npc->act_wait > 10)
				npc->direct = 2;

			if (npc->act_wait > 200)
				npc->act_no = 40;

			break;

		case 40:
			npc->act_no = 41;
			npc->act_wait = 2;
			npc->direct = 0;
			npc->y -= 48 * 0x200;
			npc->xm = -0x1000;
			// Fallthrough
		case 41:
			npc->x += npc->xm;
			npc->y += npc->ym;

			npc->act_wait += 2;

			if (npc->act_wait > 1200)
				npc->cond = 0;

			break;
	}

	if (npc->act_wait % 4 == 0 && npc->act_no >= 20)
	{
		PlaySoundObject(34, SOUND_MODE_PLAY);

		if (npc->direct == 0)
			SetCaret(npc->x + (10 * 0x200), npc->y + (10 * 0x200), CARET_EXHAUST, DIR_RIGHT);
		else
			SetCaret(npc->x - (10 * 0x200), npc->y + (10 * 0x200), CARET_EXHAUST, DIR_LEFT);
	}

	RECT rcLeft[2] = {
		{224, 64, 256, 80},
		{256, 64, 288, 96},
	};

	RECT rcRight[2] = {
		{224, 80, 256, 96},
		{288, 64, 320, 96},
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Scooter (broken)
void ActNpc193(NPCHAR *npc)
{
	RECT rc = {256, 96, 320, 112};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y = npc->y;	// This line probably isn't accurate to the original source code, but it produces the same assembly
			npc->x += 24 * 0x200;
			break;
	}

	npc->rect = rc;
}

// Blue robot (broken)
void ActNpc194(NPCHAR *npc)
{
	RECT rc = {192, 120, 224, 128};

	if (npc->act_no == 0)
	{
		npc->act_no = 1;
		npc->y += 4 * 0x200;
	}

	npc->rect = rc;
}

// Grate
void ActNpc195(NPCHAR *npc)
{
	RECT rc = {112, 64, 128, 80};
	npc->rect = rc;
}

// Ironhead motion wall
void ActNpc196(NPCHAR *npc)
{
	RECT rcLeft = {112, 64, 144, 80};
	RECT rcRight = {112, 80, 144, 96};

	npc->x -= 6 * 0x200;

	if (npc->x <= 19 * 0x10 * 0x200)
		npc->x += 22 * 0x10 * 0x200;

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight;
}

// Porcupine Fish
void ActNpc197(NPCHAR *npc)
{
	RECT rc[4] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
		{32, 0, 48, 16},
		{48, 0, 64, 16},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->ani_wait = 0;
			npc->ym = Random(-0x200, 0x200);
			npc->xm = 0x800;
			// Fallthrough
		case 10:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (npc->xm < 0)
			{
				npc->damage = 3;
				npc->act_no = 20;
			}

			break;

		case 20:
			npc->damage = 3;

			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			if (npc->x < 48 * 0x200)
			{
				npc->destroy_voice = 0;
				LoseNpChar(npc, TRUE);
			}

			break;
	}

	if (npc->flag & 2)
		npc->ym = 0x200;
	if (npc->flag & 8)
		npc->ym = -0x200;

	npc->xm -= 12;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rc[npc->ani_no];
}

// Ironhead projectile
void ActNpc198(NPCHAR *npc)
{
	RECT rcRight[3] = {
		{208, 48, 224, 72},
		{224, 48, 240, 72},
		{240, 48, 256, 72},
	};

	switch (npc->act_no)
	{
		case 0:
			if (++npc->act_wait > 20)
			{
				npc->act_no = 1;
				npc->xm = 0;
				npc->ym = 0;
				npc->count1 = 0;
			}

			break;

		case 1:
			npc->xm += 0x20;
			break;
	}

	if (++npc->ani_wait > 0)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rcRight[npc->ani_no];

	if (++npc->count1 > 100)
		npc->cond = 0;

	if (npc->count1 % 4 == 1)
		PlaySoundObject(46, SOUND_MODE_PLAY);
}

// Water/wind particles
void ActNpc199(NPCHAR *npc)
{
	RECT rect[5] = {
		{72, 16, 74, 18},
		{74, 16, 76, 18},
		{76, 16, 78, 18},
		{78, 16, 80, 18},
		{80, 16, 82, 18},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = Random(0, 2);

			switch (npc->direct)
			{
				case 0:
					npc->xm = -1;
					break;

				case 1:
					npc->ym = -1;
					break;

				case 2:
					npc->xm = 1;
					break;

				case 3:
					npc->ym = 1;
					break;
			}

			npc->xm *= (Random(4, 8) * 0x200) / 2;
			npc->ym *= (Random(4, 8) * 0x200) / 2;
			break;
	}

	if (++npc->ani_wait > 6)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 4)
	{
		npc->cond = 0;
	#ifdef FIX_MAJOR_BUGS
		return;	// The code below will use 'ani_no' to access 'rect', even though it's now too high
	#endif
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rect[npc->ani_no];
}

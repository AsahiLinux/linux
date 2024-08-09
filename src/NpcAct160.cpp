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
#include "MyChar.h"
#include "MycParam.h"
#include "NpChar.h"
#include "NpcHit.h"
#include "Sound.h"
#include "Triangle.h"

// Puu Black
void ActNpc160(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SOLID_SOFT;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->x < gMC.x)
				npc->direct = 2;
			else
				npc->direct = 0;

			npc->ym = 0xA00;

			if (npc->y < 128 * 0x200)
			{
				++npc->count1;
			}
			else
			{
				npc->bits &= ~NPC_IGNORE_SOLIDITY;
				npc->act_no = 2;
			}

			break;

		case 2:
			npc->ym = 0xA00;

			if (npc->flag & 8)
			{
				DeleteNpCharCode(161, TRUE);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				npc->act_no = 3;
				npc->act_wait = 0;
				SetQuake(30);
				PlaySoundObject(26, SOUND_MODE_PLAY);
				PlaySoundObject(72, SOUND_MODE_PLAY);
			}

			if (npc->y < gMC.y && gMC.flag & 8)
				npc->damage = 20;
			else
				npc->damage = 0;

			break;

		case 3:
			npc->damage = 20;	// Overwritten by the following line
			npc->damage = 0;

			if (++npc->act_wait > 24)
			{
				npc->act_no = 4;
				npc->count1 = 0;
				npc->count2 = 0;
			}

			break;

		case 4:
			gSuperXpos = npc->x;
			gSuperYpos = npc->y;

			if (npc->shock % 2 == 1)
			{
				SetNpChar(161, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-0x600, 0x600), Random(-0x600, 0x600), 0, NULL, 0x100);

				if (++npc->count1 > 30)
				{
					npc->count1 = 0;
					npc->act_no = 5;
					npc->ym = -0xC00;
					npc->bits |= NPC_IGNORE_SOLIDITY;
				}
			}

			break;

		case 5:
			gSuperXpos = npc->x;
			gSuperYpos = npc->y;

			if (++npc->count1 > 60)
			{
				npc->count1 = 0;
				npc->act_no = 6;
			}

			break;

		case 6:
			gSuperXpos = gMC.x;
			gSuperYpos = 400 * 0x10 * 0x200;

			if (++npc->count1 > 110)
			{
				npc->count1 = 10;
				npc->x = gMC.x;
				npc->y = 0;
				npc->ym = 0x5FF;
				npc->act_no = 1;
			}

			break;
	}

	npc->y += npc->ym;

	switch (npc->act_no)
	{
		case 0:
		case 1:
			npc->ani_no = 3;
			break;

		case 2:
			npc->ani_no = 3;
			break;

		case 3:
			npc->ani_no = 2;
			break;

		case 4:
			npc->ani_no = 0;
			break;

		case 5:
			npc->ani_no = 3;
			break;

		case 6:
			npc->ani_no = 3;
			break;
	}

	RECT rect_left[4] = {
		{0, 0, 40, 24},
		{40, 0, 80, 24},
		{80, 0, 120, 24},
		{120, 0, 160, 24},
	};

	RECT rect_right[4] = {
		{0, 24, 40, 48},
		{40, 24, 80, 48},
		{80, 24, 120, 48},
		{120, 24, 160, 48},
	};

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Puu Black projectile
void ActNpc161(NPCHAR *npc)
{
	npc->exp = 0;

	if (npc->x < gSuperXpos)
		npc->xm += 0x40;
	else
		npc->xm -= 0x40;

	if (npc->y < gSuperYpos)
		npc->ym += 0x40;
	else
		npc->ym -= 0x40;

	if (npc->xm < -4605)
		npc->xm = -4605;
	if (npc->xm > 4605)
		npc->xm = 4605;

	if (npc->ym < -4605)
		npc->ym = -4605;
	if (npc->ym > 4605)
		npc->ym = 4605;

	if (npc->life < 100)
	{
		npc->bits &= ~NPC_SHOOTABLE;
		npc->bits &= ~NPC_INVULNERABLE;
		npc->damage = 0;
		npc->ani_no = 2;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->ani_no < 2)
	{
		if (Random(0, 10) == 2)
			npc->ani_no = 0;
		else
			npc->ani_no = 1;
	}

	RECT rect[3] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
	};

	npc->rect = rect[npc->ani_no];
}

// Puu Black (dead)
void ActNpc162(NPCHAR *npc)
{
	int i;

	RECT rect_left = {40, 0, 80, 24};
	RECT rect_right = {40, 24, 80, 48};
	RECT rect_end = {0, 0, 0, 0};

	switch (npc->act_no)
	{
		case 0:
			DeleteNpCharCode(161, TRUE);
			PlaySoundObject(72, SOUND_MODE_PLAY);

			for (i = 0; i < 10; ++i)
				SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-0x600, 0x600), Random(-0x600, 0x600), 0, NULL, 0x100);

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->direct == 0)
				npc->rect = rect_left;
			else
				npc->rect = rect_right;

			npc->count1 = 0;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			++npc->count1;

			if (npc->count1 % 4 == 0)
				SetNpChar(161, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), 0, 0, 0, NULL, 0x100);

			if (npc->count1 > 160)
			{
				npc->count1 = 0;
				npc->act_no = 2;
				npc->tgt_y = npc->y;
			}

			break;

		case 2:
			SetQuake(2);

			++npc->count1;

			if (npc->count1 <= 240)
			{
				if (npc->direct == 0)
					npc->rect = rect_left;
				else
					npc->rect = rect_right;

				npc->rect.top += npc->count1 / 8;
				npc->y = npc->tgt_y + ((npc->count1 / 8) * 0x200);
				npc->rect.left -= (npc->count1 / 2) % 2;
			}
			else
			{
				npc->rect = rect_end;

				npc->count1 = 0;
				npc->act_no = 3;
			}

			if (npc->count1 % 3 == 2)
				SetNpChar(161, npc->x + (Random(-12, 12) * 0x200), npc->y - (12 * 0x200), Random(-0x200, 0x200), 0x100, 0, NULL, 0x100);

			if (npc->count1 % 4 == 2)
				PlaySoundObject(21, SOUND_MODE_PLAY);

			break;

		case 3:
			if (++npc->count1 < 60)
				break;

			DeleteNpCharCode(161, TRUE);
			npc->cond = 0;

			break;

	}

	gSuperXpos = npc->x;
	gSuperYpos = -1000 * 0x200;
}

// Dr Gero
void ActNpc163(NPCHAR *npc)
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

// Nurse Hasumi
void ActNpc164(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{224, 0, 240, 16},
		{240, 0, 256, 16},
	};

	RECT rcRight[2] = {
		{224, 16, 240, 32},
		{240, 16, 256, 32},
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

// Curly (collapsed)
void ActNpc165(NPCHAR *npc)
{
	RECT rcRight[2] = {
		{192, 96, 208, 112},
		{208, 96, 224, 112},
	};

	RECT rcLeft = {144, 96, 160, 112};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 10 * 0x200;
			// Fallthrough
		case 1:
			if (npc->direct == 2 && gMC.x > npc->x - (32 * 0x200) && gMC.x < npc->x + (32 * 0x200) && gMC.y > npc->y - (16 * 0x200) && gMC.y < npc->y + (16 * 0x200))
				npc->ani_no = 1;
			else
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft;
	else
		npc->rect = rcRight[npc->ani_no];
}

// Chaba
void ActNpc166(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{144, 104, 184, 128},
		{184, 104, 224, 128},
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

	npc->rect = rcLeft[npc->ani_no];
}

// Professor Booster (falling)
void ActNpc167(NPCHAR *npc)
{
	int i;

	RECT rect[3] = {
		{304, 0, 320, 16},
		{304, 16, 320, 32},
		{0, 0, 0, 0},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 1;
			break;

		case 10:
			npc->ani_no = 0;

			npc->ym += 0x40;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			npc->y += npc->ym;
			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			npc->ani_no = 0;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 21:
			if (++npc->ani_no > 2)
				npc->ani_no = 1;

			if (++npc->act_wait > 100)
			{
				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

				npc->cond = 0;
			}

			break;
	}

	npc->rect = rect[npc->ani_no];
}

// Boulder
void ActNpc168(NPCHAR *npc)
{
	RECT rect = {264, 56, 320, 96};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->tgt_x = npc->x;
			// Fallthrough
		case 11:
			++npc->act_wait;
			npc->x = npc->tgt_x;

			if (npc->act_wait / 3 % 2)
				npc->x += 1 * 0x200;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			npc->ym = -0x400;
			npc->xm = 0x100;
			PlaySoundObject(25, SOUND_MODE_PLAY);
			// Fallthrough
		case 21:
			npc->ym += 0x10;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->act_wait && npc->flag & 8)
			{
				PlaySoundObject(35, SOUND_MODE_PLAY);
				SetQuake(40);
				npc->act_no = 0;
			}

			if (npc->act_wait == 0)
				++npc->act_wait;

			break;
	}

	npc->rect = rect;
}

// Balrog (missile)
void ActNpc169(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->act_wait = 30;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 1:
			if (--npc->act_wait != 0)
				break;

			npc->act_no = 2;
			++npc->count1;
			break;

		case 2:
			npc->act_no = 3;
			npc->act_wait = 0;
			npc->ani_no = 1;
			npc->ani_wait = 0;
			// Fallthrough
		case 3:
			++npc->ani_wait;

			if (npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;

				if (npc->ani_no == 2 || npc->ani_no == 4)
					PlaySoundObject(23, SOUND_MODE_PLAY);
			}

			if (npc->ani_no > 4)
				npc->ani_no = 1;

			if (npc->direct == 0)
				npc->xm -= 0x20;
			else
				npc->xm += 0x20;

			if (npc->act_wait >= 8 && npc->x - (12 * 0x200) < gMC.x && npc->x + (12 * 0x200) > gMC.x && npc->y - (12 * 0x200) < gMC.y && npc->y + (8 * 0x200) > gMC.y)
			{
				npc->act_no = 10;
				npc->ani_no = 5;
				gMC.cond |= 2;
				DamageMyChar(5);
				break;
			}

			++npc->act_wait;

			if (npc->act_wait > 75)
			{
				npc->act_no = 9;
				npc->ani_no = 0;
				break;
			}

			if (npc->flag & 5)
			{
				if (npc->count2 < 5)
				{
					++npc->count2;
				}
				else
				{
					npc->act_no = 4;
					npc->act_wait = 0;
					npc->ani_no = 7;
					npc->ym = -0x5FF;
				}
			}
			else
			{
				npc->count2 = 0;
			}

			if (npc->count1 % 2 == 0 && npc->act_wait > 25)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
				npc->ani_no = 7;
				npc->ym = -0x5FF;
				break;
			}

			break;

		case 4:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			++npc->act_wait;

			if (npc->act_wait < 30 && npc->act_wait % 6 == 1)
			{
				PlaySoundObject(39, SOUND_MODE_PLAY);
				SetNpChar(170, npc->x, npc->y, 0, 0, npc->direct, NULL, 0x100);
			}

			if (npc->flag & 8)
			{
				npc->act_no = 9;
				npc->ani_no = 8;
				SetQuake(30);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			if (npc->act_wait >= 8 && npc->x - (12 * 0x200) < gMC.x && npc->x + (12 * 0x200) > gMC.x && npc->y - (12 * 0x200) < gMC.y && npc->y + (8 * 0x200) > gMC.y)
			{
				npc->act_no = 10;
				npc->ani_no = 5;
				gMC.cond |= 2;
				DamageMyChar(10);
			}

			break;

		case 9:
			npc->xm = (npc->xm * 4) / 5;

			if (npc->xm != 0)
				break;

			npc->act_no = 0;
			break;

		case 10:
			gMC.x = npc->x;
			gMC.y = npc->y;

			npc->xm = (npc->xm * 4) / 5;

			if (npc->xm != 0)
				break;

			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_no = 5;
			npc->ani_wait = 0;
			break;

		case 11:
			gMC.x = npc->x;
			gMC.y = npc->y;

			++npc->ani_wait;

			if (npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 6)
				npc->ani_no = 5;

			++npc->act_wait;

			if (npc->act_wait > 100)
				npc->act_no = 20;

			break;

		case 20:
			PlaySoundObject(25, SOUND_MODE_PLAY);
			gMC.cond &= ~2;

			if (npc->direct == 0)
			{
				gMC.x += 4 * 0x200;
				gMC.y -= 8 * 0x200;
				gMC.xm = 0x5FF;
				gMC.ym = -0x200;
				gMC.direct = 2;
				npc->direct = 2;
			}
			else
			{
				gMC.x -= 4 * 0x200;
				gMC.y -= 8 * 0x200;
				gMC.xm = -0x5FF;
				gMC.ym = -0x200;
				gMC.direct = 0;
				npc->direct = 0;
			}

			npc->act_no = 21;
			npc->act_wait = 0;
			npc->ani_no = 7;
			// Fallthrough
		case 21:
			++npc->act_wait;

			if (npc->act_wait < 50)
				break;

			npc->act_no = 0;
			break;
	}

	npc->ym += 0x20;

	if (npc->xm < -0x300)
		npc->xm = -0x300;
	if (npc->xm > 0x300)
		npc->xm = 0x300;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rect_left[9] = {
		{0, 0, 40, 24},
		{0, 48, 40, 72},
		{0, 0, 40, 24},
		{40, 48, 80, 72},
		{0, 0, 40, 24},
		{80, 48, 120, 72},
		{120, 48, 160, 72},
		{120, 0, 160, 24},
		{80, 0, 120, 24},
	};

	RECT rect_right[9] = {
		{0, 24, 40, 48},
		{0, 72, 40, 96},
		{0, 24, 40, 48},
		{40, 72, 80, 96},
		{0, 24, 40, 48},
		{80, 72, 120, 96},
		{120, 72, 160, 96},
		{120, 24, 160, 48},
		{80, 24, 120, 48},
	};

	if (npc->direct == 0)
		npc->rect = rect_left[npc->ani_no];
	else
		npc->rect = rect_right[npc->ani_no];
}

// Balrog missile
void ActNpc170(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{112, 96, 128, 104},
		{128, 96, 144, 104},
	};

	RECT rcRight[2] = {
		{112, 104, 128, 112},
		{128, 104, 144, 112},
	};

	BOOL bHit = FALSE;

	if (npc->direct == 0 && npc->flag & 1)
		bHit = TRUE;
	if (npc->direct == 2 && npc->flag & 4)
		bHit = TRUE;

	if (bHit)
	{
		PlaySoundObject(44, SOUND_MODE_PLAY);
		SetDestroyNpChar(npc->x, npc->y, 0, 3);
		VanishNpChar(npc);
		return;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
				npc->xm = Random(1, 2) * 0x200;
			else
				npc->xm = Random(-2, -1) * 0x200;

			npc->ym = Random(-2, 0) * 0x200;
			// Fallthrough
		case 1:
			++npc->count1;

			if (npc->direct == 0)
			{
				npc->xm -= 0x20;

				if (npc->count1 % 3 == 1)
					SetCaret(npc->x + (8 * 0x200), npc->y, CARET_EXHAUST, DIR_RIGHT);
			}
			else
			{
				npc->xm += 0x20;

				if (npc->count1 % 3 == 1)
					SetCaret(npc->x - (8 * 0x200), npc->y, CARET_EXHAUST, DIR_LEFT);
			}

			if (npc->count1 < 50)
			{
				if (npc->y < gMC.y)
					npc->ym += 0x20;
				else
					npc->ym -= 0x20;
			}
			else
			{
				npc->ym = 0;
			}

			if (++npc->ani_no > 1)
				npc->ani_no = 0;

			break;
	}

	if (npc->xm < -0x400)
		npc->xm = -0x600;
	if (npc->xm > 0x400)
		npc->xm = 0x600;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Fire Whirrr
void ActNpc171(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{120, 48, 152, 80},
		{152, 48, 184, 80},
	};

	RECT rcRight[2] = {
		{184, 48, 216, 80},
		{216, 48, 248, 80},
	};

	if (npc->x > gMC.x)
		npc->direct = 0;
	else
		npc->direct = 2;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = Random(0, 50);
			npc->tgt_y = npc->y;
			// Fallthrough
		case 1:
			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->act_no = 10;
				npc->ym = 0x200;
			}
			// Fallthrough
		case 10:
			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (npc->y < npc->tgt_y)
				npc->ym += 0x10;
			else
				npc->ym -= 0x10;

			if (npc->ym > 0x200)
				npc->ym = 0x200;
			if (npc->ym < -0x200)
				npc->ym = -0x200;

			npc->y += npc->ym;

			if (npc->direct == 0)
			{
				if (gMC.y < npc->y + (80 * 0x200) && gMC.y > npc->y - (80 * 0x200) && gMC.x < npc->x && gMC.x > npc->x - (160 * 0x200))
					++npc->count1;
			}
			else
			{
				if (gMC.y < npc->y + (80 * 0x200) && gMC.y > npc->y - (80 * 0x200) && gMC.x < npc->x + (160 * 0x200) && gMC.x > npc->x)
					++npc->count1;
			}

			if (npc->count1 > 20)
			{
				SetNpChar(172, npc->x, npc->y, 0, 0, npc->direct, NULL, 0x100);
				npc->count1 = -100;
				gCurlyShoot_wait = Random(80, 100);
				gCurlyShoot_x = npc->x;
				gCurlyShoot_y = npc->y;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Fire Whirr projectile
void ActNpc172(NPCHAR *npc)
{
	RECT rect[3] = {
		{248, 48, 264, 80},
		{264, 48, 280, 80},
		{280, 48, 296, 80},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 0;

			if (npc->direct == 0)
				npc->x -= 1 * 0x200;
			else
				npc->x += 1 * 0x200;

			if (npc->flag & 1 || npc->flag & 4)
			{
				SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
				VanishNpChar(npc);
				return;
			}

			break;
	}

	npc->rect = rect[npc->ani_no];
}

// Gaudi (armoured)
void ActNpc173(NPCHAR *npc)
{
	unsigned char deg;
	int xm, ym;

	RECT rcLeft[4] = {
		{0, 128, 24, 152},
		{24, 128, 48, 152},
		{48, 128, 72, 152},
		{72, 128, 96, 152},
	};

	RECT rcRight[4] = {
		{0, 152, 24, 176},
		{24, 152, 48, 176},
		{48, 152, 72, 176},
		{72, 152, 96, 176},
	};

	if (npc->x > gMC.x + (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->x < gMC.x - (((WINDOW_WIDTH / 2) + 160) * 0x200) || npc->y > gMC.y + (((WINDOW_HEIGHT / 2) + 120) * 0x200) || npc->y < gMC.y - (((WINDOW_HEIGHT / 2) + 120) * 0x200))
		return;

	switch (npc->act_no)
	{
		case 0:
			npc->tgt_x = npc->x;
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->ani_no = 0;
			npc->xm = 0;

			if (npc->act_wait < 5)
			{
				++npc->act_wait;
			}
			else
			{
				if (npc->x - (192 * 0x200) < gMC.x && npc->x + (192 * 0x200) > gMC.x && npc->y - (160 * 0x200) < gMC.y && npc->y + (160 * 0x200) > gMC.y)
				{
					npc->act_no = 10;
					npc->act_wait = 0;
					npc->ani_no = 1;
				}
			}

			break;

		case 10:
			if (++npc->act_wait > 3)
			{
				if (++npc->count1 == 3)
				{
					PlaySoundObject(30, SOUND_MODE_PLAY);
					npc->count1 = 0;
					npc->act_no = 25;
					npc->act_wait = 0;
					npc->ani_no = 2;
					npc->ym = -0x600;

					if (npc->x < npc->tgt_x)
						npc->xm = 0x80;
					else
						npc->xm = -0x80;
				}
				else
				{
					PlaySoundObject(30, SOUND_MODE_PLAY);
					npc->act_no = 20;
					npc->ani_no = 2;
					npc->ym = -0x200;

					if (npc->x < npc->tgt_x)
						npc->xm = 0x200;
					else
						npc->xm = -0x200;
				}
			}

			break;

		case 20:
			++npc->act_wait;

			if (npc->flag & 8)
			{
				PlaySoundObject(23, SOUND_MODE_PLAY);
				npc->ani_no = 1;
				npc->act_no = 30;
				npc->act_wait = 0;
			}

			break;

		case 25:
			if (++npc->act_wait == 30 || npc->act_wait == 40)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-6, 6);
				ym = GetSin(deg) * 3;
				xm = GetCos(deg) * 3;

				SetNpChar(174, npc->x, npc->y, xm, ym, 0, NULL, 0x100);

				PlaySoundObject(39, SOUND_MODE_PLAY);
				npc->ani_no = 3;

				gCurlyShoot_wait = Random(80, 100);
				gCurlyShoot_x = npc->x;
				gCurlyShoot_y = npc->y;
			}

			if (npc->act_wait == 35 || npc->act_wait == 45)
				npc->ani_no = 2;

			if (npc->flag & 8)
			{
				PlaySoundObject(23, SOUND_MODE_PLAY);
				npc->ani_no = 1;
				npc->act_no = 30;
				npc->act_wait = 0;
			}

			break;

		case 30:
			npc->xm = 7 * npc->xm / 8;

			if (++npc->act_wait > 3)
			{
				npc->ani_no = 0;
				npc->act_no = 1;
				npc->act_wait = 0;
			}

			break;
	}

	npc->ym += 51;

	if (gMC.x < npc->x)
		npc->direct = 0;
	else
		npc->direct = 2;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;
	if (npc->ym < -0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->life <= 985)
	{
		SetDestroyNpChar(npc->x, npc->y, 0, 2);
		npc->code_char = 154;
		npc->act_no = 0;
	}
}

// Armoured-Gaudi projectile
void ActNpc174(NPCHAR *npc)
{
	BOOL bHit;

	switch (npc->act_no)
	{
		case 0:
			if (npc->direct == 2)
				npc->act_no = 2;
			// Fallthrough
		case 1:
			npc->x += npc->xm;
			npc->y += npc->ym;

			bHit = FALSE;

			if (npc->flag & 1)
			{
				bHit = TRUE;
				npc->xm = 0x200;
			}

			if (npc->flag & 4)
			{
				bHit = TRUE;
				npc->xm = -0x200;
			}

			if (npc->flag & 2)
			{
				bHit = TRUE;
				npc->ym = 0x200;
			}

			if (npc->flag & 8)
			{
				bHit = TRUE;
				npc->ym = -0x200;
			}

			if (bHit)
			{
				npc->act_no = 2;
				++npc->count1;
				PlaySoundObject(31, SOUND_MODE_PLAY);
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

	RECT rect_left[3] = {
		{120, 80, 136, 96},
		{136, 80, 152, 96},
		{152, 80, 168, 96},
	};

	if (++npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rect_left[npc->ani_no];
}

// Gaudi egg
void ActNpc175(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{168, 80, 192, 104},
		{192, 80, 216, 104},
	};

	RECT rcRight[2] = {
		{216, 80, 240, 104},
		{240, 80, 264, 104},
	};

	if (npc->act_no < 3 && npc->life < 90)
	{
		LoseNpChar(npc, FALSE);
		npc->act_no = 10;
		npc->ani_no = 1;
		npc->bits &= ~NPC_SHOOTABLE;
		npc->damage = 0;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 0;
			npc->act_no = 1;
			break;
	}

	if (npc->direct == 0)
		npc->ym += 0x20;
	else
		npc->ym -= 0x20;

	if (npc->ym < -0x5FF)
		npc->ym = -0x5FF;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// BuyoBuyo Base
void ActNpc176(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{96, 128, 128, 144},
		{128, 128, 160, 144},
		{160, 128, 192, 144},
	};

	RECT rcRight[3] = {
		{96, 144, 128, 160},
		{128, 144, 160, 160},
		{160, 144, 192, 160},
	};

	if (npc->act_no < 3 && npc->life < 940)
	{
		LoseNpChar(npc, FALSE);
		npc->act_no = 10;
		npc->ani_no = 2;
		npc->bits &= ~NPC_SHOOTABLE;
		npc->damage = 0;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 1:
			if (npc->direct == 0)
			{
				if (npc->x < gMC.x + (160 * 0x200) && npc->x > gMC.x - (160 * 0x200) && npc->y < gMC.y + (160 * 0x200) && npc->y > gMC.y - (16 * 0x200))
					++npc->count1;
			}
			else
			{
				if (npc->x < gMC.x + (160 * 0x200) && npc->x > gMC.x - (160 * 0x200) && npc->y < gMC.y + (16 * 0x200) && npc->y > gMC.y - (160 * 0x200))
					++npc->count1;
			}

			if (npc->count1 > 10)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
			}

			break;

		case 2:
			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (++npc->act_wait > 10)
			{
				if (++npc->count2 > 2)
				{
					npc->count2 = 0;
					npc->count1 = -90;
				}
				else
				{
					npc->count1 = -10;
				}

				if (npc->direct == 0)
					SetNpChar(177, npc->x, npc->y - (8 * 0x200), 0, 0, 0, NULL, 0x100);
				else
					SetNpChar(177, npc->x, npc->y + (8 * 0x200), 0, 0, 2, NULL, 0x100);

				PlaySoundObject(39, SOUND_MODE_PLAY);

				npc->act_no = 0;
				npc->ani_no = 0;

				gCurlyShoot_wait = Random(80, 100);
				gCurlyShoot_x = npc->x;
				gCurlyShoot_y = npc->y;
			}

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// BuyoBuyo
void ActNpc177(NPCHAR *npc)
{
	RECT rc[2] = {
		{192, 128, 208, 144},
		{208, 128, 224, 144},
	};

	if (npc->flag & 0xFF)
	{
		SetCaret(npc->x, npc->y, CARET_SHOOT, DIR_LEFT);
		npc->cond = 0;
		return;
	}

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
				npc->ym = -0x600;
			else
				npc->ym = 0x600;
			// Fallthrough
		case 1:
			if (npc->y < gMC.y + (16 * 0x200) && npc->y > gMC.y - (16 * 0x200))
			{
				npc->act_no = 10;

				npc->tgt_x = npc->x;
				npc->tgt_y = npc->y;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				npc->xm = ((Random(0, 1) * 0x200) - 0x100) * 2;
				npc->ym = ((Random(0, 1) * 0x200) - 0x100) * 2;
			}

			break;

		case 10:
			if (npc->x < npc->tgt_x)
				npc->xm += 0x20;
			else
				npc->xm -= 0x20;

			if (npc->y < npc->tgt_y)
				npc->ym += 0x20;
			else
				npc->ym -= 0x20;

			if (++npc->act_wait > 300)
			{
				SetCaret(npc->x, npc->y, CARET_SHOOT, DIR_LEFT);
				npc->cond = 0;
				return;
			}

			if (npc->direct == 0)
				npc->tgt_x -= 1 * 0x200;
			else
				npc->tgt_x += 1 * 0x200;

			break;
	}

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

	if (++npc->ani_wait > 6)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rc[npc->ani_no];
}

// Core blade projectile
void ActNpc178(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}

	if (npc->flag & 0x100)
	{
		npc->y += npc->ym / 2;
		npc->x += npc->xm / 2;
	}
	else
	{
		npc->y += npc->ym;
		npc->x += npc->xm;
	}

	RECT rect_left[3] = {
		{0, 224, 16, 240},
		{16, 224, 32, 240},
		{32, 224, 48, 240},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rect_left[npc->ani_no];

	if (++npc->count1 > 150)
	{
		VanishNpChar(npc);
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
	}
}

// Core wisp projectile
void ActNpc179(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		npc->cond = 0;
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
	}

	npc->xm -= 0x20;
	npc->ym = 0;

	if (npc->xm < -0x400)
		npc->xm = -0x400;

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[3] = {
		{48, 224, 72, 240},
		{72, 224, 96, 240},
		{96, 224, 120, 240},
	};

	if (++npc->ani_wait > 1)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rect_left[npc->ani_no];

	if (++npc->count1 > 300)
	{
		VanishNpChar(npc);
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
	}
}

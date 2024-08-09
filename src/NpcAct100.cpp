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
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"

// Grate
void ActNpc100(NPCHAR *npc)
{
	RECT rc[2] = {
		{272, 48, 288, 64},
		{272, 48, 288, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->y += 16 * 0x200;
			npc->act_no = 1;
			break;
	}

	if (npc->direct == 0)
		npc->rect = rc[0];
	else
		npc->rect = rc[1];
}

// Malco computer screen
void ActNpc101(NPCHAR *npc)
{
	RECT rect[3] = {
		{240, 136, 256, 152},
		{240, 136, 256, 152},
		{256, 136, 272, 152},
	};

	if (++npc->ani_wait > 3)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Malco computer wave
void ActNpc102(NPCHAR *npc)
{
	RECT rect[4] = {
		{208, 120, 224, 136},
		{224, 120, 240, 136},
		{240, 120, 256, 136},
		{256, 120, 272, 136},
	};

	if (npc->act_no == 0)
	{
		npc->act_no = 1;
		npc->y += 8 * 0x200;
	}

	if (++npc->ani_wait > 0)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 3)
		npc->ani_no = 0;

	npc->rect = rect[npc->ani_no];
}

// Mannan projectile
void ActNpc103(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{192, 96, 208, 120},
		{208, 96, 224, 120},
		{224, 96, 240, 120},
	};

	RECT rcRight[3] = {
		{192, 120, 208, 144},
		{208, 120, 224, 144},
		{224, 120, 240, 144},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (npc->direct == 0)
				npc->xm -= 0x20;
			else
				npc->xm += 0x20;

			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 0;

			break;
	}

	npc->x += npc->xm;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (++npc->count1 > 100)
		npc->cond = 0;

	if (npc->count1 % 4 == 1)
		PlaySoundObject(46, SOUND_MODE_PLAY);
}

// Frog
void ActNpc104(NPCHAR *npc)
{
	BOOL bJump;

	RECT rcLeft[3] = {
		{0, 112, 32, 144},
		{32, 112, 64, 144},
		{64, 112, 96, 144},
	};

	RECT rcRight[3] = {
		{0, 144, 32, 176},
		{32, 144, 64, 176},
		{64, 144, 96, 176},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;

			if (npc->direct == 4)
			{
				if (Random(0, 1))
					npc->direct = 0;
				else
					npc->direct = 2;

				npc->bits |= NPC_IGNORE_SOLIDITY;
				npc->ani_no = 2;
				npc->act_no = 3;
				break;
			}

			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 1:
			++npc->act_wait;

			if (Random(0, 50) == 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->ani_wait = 0;
				break;
			}

			break;

		case 2:
			++npc->act_wait;

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (npc->act_wait > 18)
			{
				npc->act_no = 1;
				npc->act_no = 1;	// Duplicate line
			}

			break;

		case 3:
			if (++npc->act_wait > 40)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->flag & 8)
			{
				npc->act_no = 0;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			break;

		case 10:
			npc->act_no = 11;
			// Fallthrough
		case 11:
			if (npc->flag & 1 && npc->xm < 0)
			{
				npc->xm *= -1;
				npc->direct = 2;
			}

			if (npc->flag & 4 && npc->xm > 0)
			{
				npc->xm *= -1;
				npc->direct = 0;
			}

			if (npc->flag & 8)
			{
				npc->act_no = 0;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			break;
	}

	bJump = FALSE;

	if (npc->act_no < 10 && npc->act_no != 3 && npc->act_wait > 10)
	{
		if (npc->shock)
			bJump = TRUE;

		if (npc->x < gMC.x - (160 * 0x200) || npc->x > gMC.x + (160 * 0x200) || npc->y < gMC.y - (64 * 0x200) || npc->y > gMC.y + (64 * 0x200))
		{
			// This blank space is needed for the function to produce the same assembly.
			// Chances are there used to be some commented-out code here.
		}
		else
		{
			if (Random(0, 50) == 2)
				bJump = TRUE;
		}
	}

	if (bJump)
	{
		if (npc->x < gMC.x)
			npc->direct = 2;
		else
			npc->direct = 0;

		npc->act_no = 10;
		npc->ani_no = 2;
		npc->ym = -0x5FF;

		if (!(gMC.cond & 2))
			PlaySoundObject(30, SOUND_MODE_PLAY);

		if (npc->direct == 0)
			npc->xm = -0x200;
		else
			npc->xm = 0x200;
	}

	npc->ym += 0x80;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// "HEY!" speech bubble (low)
void ActNpc105(NPCHAR *npc)
{
	RECT rect[2] = {
		{128, 32, 144, 48},
		{128, 32, 128, 32},
	};

	if (++npc->act_wait > 30)
		npc->cond = 0;

	if (npc->act_wait < 5)
		npc->y -= 1 * 0x200;

	npc->rect = rect[npc->ani_no];
}

// "HEY!" speech bubble (high)
void ActNpc106(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			SetNpChar(105, npc->x, npc->y - (8 * 0x200), 0, 0, 0, NULL, 0x180);
			npc->act_no = 1;
			break;
	}
}

// Malco
void ActNpc107(NPCHAR *npc)
{
	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 2)
				npc->ani_no = 5;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_wait = 0;

			for (i = 0; i < 4; ++i)
				SetNpChar(4, npc->x, npc->y, Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

			// Fallthrough
		case 11:
			if (++npc->ani_wait > 1)
			{
				PlaySoundObject(43, SOUND_MODE_PLAY);
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (++npc->act_wait > 100)
				npc->act_no = 12;

			break;

		case 12:
			npc->act_no = 13;
			npc->act_wait = 0;
			npc->ani_no = 1;
			// Fallthrough
		case 13:
			if (++npc->act_wait > 50)
				npc->act_no = 14;

			break;

		case 14:
			npc->act_no = 15;
			npc->act_wait = 0;
			// Fallthrough
		case 15:
			if (npc->act_wait / 2 % 2)
			{
				npc->x += 1 * 0x200;
				PlaySoundObject(11, SOUND_MODE_PLAY);
			}
			else
			{
				npc->x -= 1 * 0x200;
			}

			if (++npc->act_wait > 50)
				npc->act_no = 16;

			break;

		case 16:
			npc->act_no = 17;
			npc->act_wait = 0;
			npc->ani_no = 2;
			PlaySoundObject(12, SOUND_MODE_PLAY);

			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x, npc->y, Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

			// Fallthrough
		case 17:
			if (++npc->act_wait > 150)
				npc->act_no = 18;

			break;

		case 18:
			npc->act_no = 19;
			npc->act_wait = 0;
			npc->ani_no = 3;
			npc->ani_wait = 0;
			// Fallthrough
		case 19:
			if (++npc->ani_wait > 3)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 4)
			{
				PlaySoundObject(11, SOUND_MODE_PLAY);
				npc->ani_no = 3;
			}

			if (++npc->act_wait > 100)
			{
				npc->act_no = 20;
				PlaySoundObject(12, SOUND_MODE_PLAY);

				for (i = 0; i < 4; ++i)
					SetNpChar(4, npc->x, npc->y, Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);
			}

			break;

		case 20:
			npc->ani_no = 4;
			break;

		case 21:
			npc->act_no = 22;
			npc->ani_no = 5;
			PlaySoundObject(51, SOUND_MODE_PLAY);
			break;

		case 100:
			npc->act_no = 101;
			npc->ani_no = 6;
			npc->ani_wait = 0;
			// Fallthrough
		case 101:
			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 9)
				npc->ani_no = 6;

			break;

		case 110:
			SetDestroyNpChar(npc->x, npc->y, 0x2000, 16);
			npc->cond = 0;
			break;
	}

	RECT rcPoweron[10] = {
		{144, 0, 160, 24},
		{160, 0, 176, 24},
		{176, 0, 192, 24},
		{192, 0, 208, 24},
		{208, 0, 224, 24},
		{224, 0, 240, 24},
		{176, 0, 192, 24},
		{192, 0, 208, 24},
		{208, 0, 224, 24},
		{192, 0, 208, 24},
	};

	npc->rect = rcPoweron[npc->ani_no];
}

// Balfrog projectile
void ActNpc108(NPCHAR *npc)
{
	if (npc->flag & 0xFF)
	{
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}

	npc->y += npc->ym;
	npc->x += npc->xm;

	RECT rect_left[3] = {
		{96, 48, 112, 64},
		{112, 48, 128, 64},
		{128, 48, 144, 64},
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
		SetCaret(npc->x, npc->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		npc->cond = 0;
	}
}

// Malco (broken)
void ActNpc109(NPCHAR *npc)
{
	int i;

	RECT rcLeft[2] = {
		{240, 0, 256, 24},
		{256, 0, 272, 24},
	};

	RECT rcRight[2] = {
		{240, 24, 256, 48},
		{256, 24, 272, 48},
	};

	switch (npc->act_no)
	{
		case 0:
			if (--npc->act_wait != 0)
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

		case 10:
			npc->act_no = 0;
			PlaySoundObject(12, SOUND_MODE_PLAY);

			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x, npc->y, Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

			break;
	}

	npc->ym += 0x40;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Puchi
void ActNpc110(NPCHAR *npc)
{
	BOOL bJump;

	RECT rcLeft[3] = {
		{96, 128, 112, 144},
		{112, 128, 128, 144},
		{128, 128, 144, 144},
	};

	RECT rcRight[3] = {
		{96, 144, 112, 160},
		{112, 144, 128, 160},
		{128, 144, 144, 160},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;

			if (npc->direct == 4)
			{
				if (Random(0, 1))
					npc->direct = 0;
				else
					npc->direct = 2;

				npc->bits |= NPC_IGNORE_SOLIDITY;
				npc->ani_no = 2;
				npc->act_no = 3;

				break;
			}

			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 1:
			++npc->act_wait;

			if (Random(0, 50) == 1)
			{
				npc->act_no = 2;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->ani_wait = 0;
				break;
			}

			break;

		case 2:
			++npc->act_wait;

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (npc->act_wait > 18)
			{
				npc->act_no = 1;
				npc->act_no = 1;	// Duplicate line
			}

			break;

		case 3:
			if (++npc->act_wait > 40)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->flag & 8)
			{
				npc->act_no = 0;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			break;

		case 10:
			npc->act_no = 11;
			// Fallthrough
		case 11:
			if (npc->flag & 1 && npc->xm < 0)
			{
				npc->xm *= -1;
				npc->direct = 2;
			}

			if (npc->flag & 4 && npc->xm > 0)
			{
				npc->xm *= -1;
				npc->direct = 0;
			}

			if (npc->flag & 8)
			{
				npc->act_no = 0;
				npc->ani_no = 0;
				npc->act_wait = 0;
			}

			break;
	}

	bJump = FALSE;

	if (npc->act_no < 10 && npc->act_no != 3 && npc->act_wait > 10)
	{
		if (npc->shock)
			bJump = TRUE;

		if (npc->x < gMC.x - (160 * 0x200) || npc->x > gMC.x + (160 * 0x200) || npc->y < gMC.y - (64 * 0x200) || npc->y > gMC.y + (64 * 0x200))
		{
			// This blank space is needed for the function to produce the same assembly.
			// Chances are there used to be some commented-out code here.
		}
		else
		{
			if (Random(0, 50) == 2)
				bJump = TRUE;
		}
	}

	if (bJump)
	{
		if (npc->x < gMC.x)
			npc->direct = 2;
		else
			npc->direct = 0;

		npc->act_no = 10;
		npc->ani_no = 2;
		npc->ym = -0x2FF;
		PlaySoundObject(6, SOUND_MODE_PLAY);

		if (npc->direct == 0)
			npc->xm = -0x100;
		else
			npc->xm = 0x100;
	}

	npc->ym += 0x80;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Quote (teleport out)
void ActNpc111(NPCHAR *npc)
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
			npc->ani_no = 0;
			npc->y -= 16 * 0x200;
			break;

		case 1:
			if (++npc->act_wait > 20)
			{
				npc->act_wait = 0;
				npc->act_no = 2;
				npc->ani_no = 1;
				npc->ym = -0x2FF;
			}

			break;

		case 2:
			if (npc->ym > 0)
				npc->hit.bottom = 16 * 0x200;

			if (npc->flag & 8)
			{
				npc->act_no = 3;
				npc->act_wait = 0;
				npc->ani_no = 0;
			}

			break;

		case 3:
			if (++npc->act_wait > 40)
			{
				npc->act_no = 4;
				npc->act_wait = 64;
				PlaySoundObject(29, SOUND_MODE_PLAY);
			}

			break;

		case 4:
			--npc->act_wait;
			npc->ani_no = 0;

			if (npc->act_wait == 0)
				npc->cond = 0;

			break;
	}

	npc->ym += 0x40;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	// Use a different sprite if the player is wearing the Mimiga Mask
	if (gMC.equip & EQUIP_MIMIGA_MASK)
	{
		npc->rect.top += 32;
		npc->rect.bottom += 32;
	}

	if (npc->act_no == 4)
	{
		npc->rect.bottom = npc->rect.top + (npc->act_wait / 4);

		if (npc->act_wait / 2 % 2)
			++npc->rect.left;
	}
}

// Quote (teleport in)
void ActNpc112(NPCHAR *npc)
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
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->x += 16 * 0x200;
			npc->y += 8 * 0x200;
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
			if (++npc->act_wait > 20)
			{
				npc->act_no = 3;
				npc->ani_no = 1;
				npc->hit.bottom = 8 * 0x200;
			}

			break;

		case 3:
			if (npc->flag & 8)
			{
				npc->act_no = 4;
				npc->act_wait = 0;
				npc->ani_no = 0;
			}

			break;
	}

	npc->ym += 0x40;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	// Use a different sprite if the player is wearing the Mimiga Mask
	if (gMC.equip & EQUIP_MIMIGA_MASK)
	{
		npc->rect.top += 32;
		npc->rect.bottom += 32;
	}

	if (npc->act_no == 1)
	{
		npc->rect.bottom = npc->rect.top + (npc->act_wait / 4);

		if (npc->act_wait / 2 % 2)
			++npc->rect.left;
	}
}

// Professor Booster
void ActNpc113(NPCHAR *npc)
{
	RECT rcLeft[7] = {
		{224, 0, 240, 16},
		{240, 0, 256, 16},
		{256, 0, 272, 16},
		{224, 0, 240, 16},
		{272, 0, 288, 16},
		{224, 0, 240, 16},
		{288, 0, 304, 16},
	};

	RECT rcRight[7] = {
		{224, 16, 240, 32},
		{240, 16, 256, 32},
		{256, 16, 272, 32},
		{224, 16, 240, 32},
		{272, 16, 288, 32},
		{224, 16, 240, 32},
		{288, 16, 304, 32},
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
				npc->x -= 1 * 0x200;
			else
				npc->x += 1 * 0x200;

			break;

		case 5:
			npc->ani_no = 6;
			break;

		case 30:
			npc->act_no = 31;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			npc->hit.bottom = 16 * 0x200;
			npc->x -= 16 * 0x200;
			npc->y += 8 * 0x200;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 31:
			if (++npc->act_wait == 64)
			{
				npc->act_no = 32;
				npc->act_wait = 0;
			}

			break;

		case 32:
			if (++npc->act_wait > 20)
			{
				npc->act_no = 33;
				npc->ani_no = 1;
				npc->hit.bottom = 8 * 0x200;
			}

			break;

		case 33:
			if (npc->flag & 8)
			{
				npc->act_no = 34;
				npc->act_wait = 0;
				npc->ani_no = 0;
			}

			break;
	}

	npc->ym += 0x40;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->act_no == 31)
	{
		npc->rect.bottom = npc->rect.top + (npc->act_wait / 4);

		if (npc->act_wait / 2 % 2)
			++npc->rect.left;
	}
}

// Press
void ActNpc114(NPCHAR *npc)
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
			if (npc->flag & 8)
			{
				// Chances are there used to be commented-out code here
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

				npc->act_no = 1;
				npc->ani_no = 0;
				npc->damage = 0;
				npc->bits |= NPC_SOLID_HARD;
			}

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;

	npc->rect = rcLeft[npc->ani_no];
}

// Ravil
void ActNpc115(NPCHAR *npc)
{
	int i;

	RECT rcLeft[6] = {
		{0, 120, 24, 144},
		{24, 120, 48, 144},
		{48, 120, 72, 144},
		{72, 120, 96, 144},
		{96, 120, 120, 144},
		{120, 120, 144, 144},
	};

	RECT rcRight[6] = {
		{0, 144, 24, 168},
		{24, 144, 48, 168},
		{48, 144, 72, 168},
		{72, 144, 96, 168},
		{96, 144, 120, 168},
		{120, 144, 144, 168},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->xm = 0;
			npc->act_no = 1;
			npc->act_wait = 0;
			npc->count1 = 0;
			// Fallthrough
		case 1:
			if (gMC.x < npc->x + (96 * 0x200) && gMC.x > npc->x - (96 * 0x200) && gMC.y < npc->y + (32 * 0x200) && gMC.y > npc->y - (96 * 0x200))
				npc->act_no = 10;

			if (npc->shock)
				npc->act_no = 10;

			break;

		case 10:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->ani_no = 1;

			if (++npc->act_wait > 20)
			{
				npc->act_wait = 0;
				npc->act_no = 20;
			}

			break;

		case 20:
			npc->damage = 0;
			npc->xm = 0;

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
			{
				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				if (npc->direct == 0)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;

				if (++npc->count1 > 2)
				{
					npc->count1 = 0;
					npc->ani_no = 4;
					npc->act_no = 21;
					npc->ym = -0x400;
					npc->xm *= 2;
					npc->damage = 5;
					PlaySoundObject(102, SOUND_MODE_PLAY);
				}
				else
				{
					npc->act_no = 21;
					npc->ym = -0x400;
					PlaySoundObject(30, SOUND_MODE_PLAY);
				}
			}

			break;

		case 21:
			if (npc->flag & 8)
			{
				PlaySoundObject(23, SOUND_MODE_PLAY);
				npc->act_no = 20;
				npc->ani_no = 1;
				npc->ani_wait = 0;
				npc->damage = 0;

				if (gMC.x > npc->x + (144 * 0x200) || gMC.x < npc->x - (144 * 0x200) || gMC.y > npc->y + (48 * 0x200) || gMC.y < npc->y - (144 * 0x200))
					npc->act_no = 0;
			}

			break;

		case 30:
			for (i = 0; i < 8; ++i)
				SetNpChar(4, npc->x + (Random(-12, 12) * 0x200), npc->y + (Random(-12, 12) * 0x200), Random(-341, 341), Random(-0x600, 0), 0, NULL, 0x100);

			npc->ani_no = 0;
			npc->act_no = 0;

			break;

		case 50:
			npc->act_no = 51;
			npc->ani_no = 4;
			npc->damage = 0;
			npc->ym = -0x200;
			npc->bits &= ~(NPC_SOLID_SOFT | NPC_SHOOTABLE);
			PlaySoundObject(51, SOUND_MODE_PLAY);
			// Fallthrough
		case 51:
			if (npc->flag & 8)
			{
				PlaySoundObject(23, SOUND_MODE_PLAY);
				npc->act_no = 52;
				npc->ani_no = 5;
				npc->xm = 0;
			}

			break;
	}

	if (npc->act_no > 50)
		npc->ym += 0x20;
	else
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

// Red petals
void ActNpc116(NPCHAR *npc)
{
	RECT rc = {272, 184, 320, 200};

	npc->rect = rc;
}

// Curly
void ActNpc117(NPCHAR *npc)
{
	RECT rcLeft[10] = {
		{0, 96, 16, 112},
		{16, 96, 32, 112},
		{0, 96, 16, 112},
		{32, 96, 48, 112},
		{0, 96, 16, 112},
		{176, 96, 192, 112},
		{112, 96, 128, 112},
		{160, 96, 176, 112},
		{144, 96, 160, 112},
		{48, 96, 64, 112},
	};

	RECT rcRight[10] = {
		{0, 112, 16, 128},
		{16, 112, 32, 128},
		{0, 112, 16, 128},
		{32, 112, 48, 128},
		{0, 112, 16, 128},
		{176, 112, 192, 128},
		{112, 112, 128, 128},
		{160, 112, 176, 128},
		{144, 112, 160, 128},
		{48, 112, 64, 128},
	};

	switch (npc->act_no)
	{
		case 0:
			if (npc->direct == 4)
			{
				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 1:
			npc->xm = 0;
			npc->ym += 0x40;
			break;

		case 3:
			if (0)
			{
				// There used to be an if-statement here that didn't do anything, but the compiler optimised it out.
				// We only know this was here because empty if-statements affect the assembly's register usage.
				// Since there's no code, we have no idea what the original condition actually was.
			}

			if (0)
			{
				// Another empty if-statement
			}

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

			npc->ym += 0x40;

			if (npc->direct == 0)
				npc->xm = -0x200;
			else
				npc->xm = 0x200;

			break;

		case 5:
			npc->act_no = 6;
			npc->ani_no = 5;
			SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
			break;

		case 6:
			if (0)
			{
				// There used to be an if-statement here that didn't do anything, but the compiler optimised it out.
				// We only know this was here because empty if-statements affect the assembly's register usage.
				// Since there's no code, we have no idea what the original condition actually was.
			}

			npc->ani_no = 5;
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 1;
			npc->ani_wait = 0;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			// Fallthrough
		case 11:
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

			if (gMC.x < npc->x + (20 * 0x200) && gMC.x > npc->x - (20 * 0x200))
				npc->act_no = 0;

			break;

		case 20:
			npc->xm = 0;
			npc->ani_no = 6;
			break;

		case 21:
			npc->xm = 0;
			npc->ani_no = 9;
			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;
			npc->ym = -0x400;
			// Fallthrough
		case 31:
			npc->ani_no = 7;

			if (npc->direct == 0)
				npc->xm = 0x200;
			else
				npc->xm = -0x200;

			npc->ym += 0x40;

			if (npc->act_wait++ != 0 && npc->flag & 8)
				npc->act_no = 32;

			break;

		case 32:
			npc->ym += 0x40;
			npc->ani_no = 8;
			npc->xm = 0;
			break;

		case 70:
			npc->act_no = 71;
			npc->act_wait = 0;
			npc->ani_no = 1;
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

			if (npc->ani_no > 4)
				npc->ani_no = 1;

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
}

// Curly (boss)
void ActNpc118(NPCHAR *npc)
{
	BOOL bUpper;

	RECT rcLeft[9] = {
		{0, 32, 32, 56},
		{32, 32, 64, 56},
		{64, 32, 96, 56},
		{96, 32, 128, 56},
		{0, 32, 32, 56},
		{128, 32, 160, 56},
		{0, 32, 32, 56},
		{0, 32, 32, 56},
		{160, 32, 192, 56},
	};

	RECT rcRight[9] = {
		{0, 56, 32, 80},
		{32, 56, 64, 80},
		{64, 56, 96, 80},
		{96, 56, 128, 80},
		{0, 56, 32, 80},
		{128, 56, 160, 80},
		{0, 56, 32, 80},
		{0, 56, 32, 80},
		{160, 56, 192, 80},
	};

	bUpper = FALSE;

	if (npc->direct == 0 && npc->x < gMC.x)
		bUpper = TRUE;
	if (npc->direct == 2 && npc->x > gMC.x)
		bUpper = TRUE;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = Random(50, 100);
			npc->ani_no = 0;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->bits |= NPC_SHOOTABLE;
			npc->bits &= ~NPC_INVULNERABLE;
			// Fallthrough
		case 11:
			if (npc->act_wait != 0)
				--npc->act_wait;
			else
				npc->act_no = 13;

			break;

		case 13:
			npc->act_no = 14;
			npc->ani_no = 3;
			npc->act_wait = Random(50, 100);

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 14:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 6)
				npc->ani_no = 3;

			if (npc->direct == 0)
				npc->xm -= 0x40;
			else
				npc->xm += 0x40;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
			}
			else
			{
				npc->bits |= NPC_SHOOTABLE;
				npc->act_no = 20;
				npc->act_wait = 0;
				PlaySoundObject(103, SOUND_MODE_PLAY);
			}

			break;

		case 20:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->xm = (npc->xm * 8) / 9;

			if (++npc->ani_no > 1)
				npc->ani_no = 0;

			if (++npc->act_wait > 50)
			{
				npc->act_no = 21;
				npc->act_wait = 0;
			}

			break;

		case 21:
			if (++npc->act_wait % 4 == 1)
			{
				if (npc->direct == 0)
				{
					if (bUpper)
					{
						npc->ani_no = 2;
						SetNpChar(123, npc->x, npc->y - (8 * 0x200), 0, 0, 1, NULL, 0x100);
					}
					else
					{
						npc->ani_no = 0;
						SetNpChar(123, npc->x - (8 * 0x200), npc->y + (4 * 0x200), 0, 0, 0, NULL, 0x100);
						npc->x += 1 * 0x200;
					}
				}
				else
				{
					if (bUpper)
					{
						npc->ani_no = 2;
						SetNpChar(123, npc->x, npc->y - (8 * 0x200), 0, 0, 1, NULL, 0x100);
					}
					else
					{
						npc->ani_no = 0;
						SetNpChar(123, npc->x + (8 * 0x200), npc->y + (4 * 0x200), 0, 0, 2, NULL, 0x100);
						npc->x -= 1 * 0x200;
					}
				}
			}

			if (npc->act_wait > 30)
				npc->act_no = 10;

			break;

		case 30:
			if (++npc->ani_no > 8)
				npc->ani_no = 7;

			if (++npc->act_wait > 30)
			{
				npc->act_no = 10;
				npc->ani_no = 0;
			}

			break;
	}

	if (npc->act_no > 10 && npc->act_no < 30 && CountArmsBullet(6))
	{
		npc->act_wait = 0;
		npc->act_no = 30;
		npc->ani_no = 7;
		npc->bits &= ~NPC_SHOOTABLE;
		npc->bits |= NPC_INVULNERABLE;
		npc->xm = 0;
	}

	npc->ym += 0x20;

	if (npc->xm > 0x1FF)
		npc->xm = 0x1FF;
	if (npc->xm < -0x1FF)
		npc->xm = -0x1FF;

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->y += npc->ym;
	npc->x += npc->xm;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Table and chair
void ActNpc119(NPCHAR *npc)
{
	RECT rc = {248, 184, 272, 200};

	npc->rect = rc;
}

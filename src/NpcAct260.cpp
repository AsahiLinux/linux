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
#include "MycParam.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Shovel Brigade (caged)
void ActNpc260(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{128, 64, 144, 80},
		{144, 64, 160, 80},
		{224, 64, 240, 80},
	};

	RECT rcRight[3] = {
		{128, 80, 144, 96},
		{144, 80, 160, 96},
		{224, 80, 240, 96},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->x += 0x200;
			npc->y -= 0x400;
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

		case 10:
			npc->act_no = 11;
			npc->ani_no = 2;
			SetNpChar(87, npc->x, npc->y - (16 * 0x200), 0, 0, 0, NULL, 0x100);

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

// Chie (caged)
void ActNpc261(NPCHAR *npc)
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
			npc->x -= 1 * 0x200;
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

// Chaco (caged)
void ActNpc262(NPCHAR *npc)
{
	RECT rcLeft[2] = {
		{128, 0, 144, 16},
		{144, 0, 160, 16},
	};

	RECT rcRight[2] = {
		{128, 16, 144, 32},
		{144, 16, 160, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->x -= 1 * 0x200;
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

// Doctor (boss)
void ActNpc263(NPCHAR *npc)
{
	int deg;
	int xm, ym;

	RECT rcLeft[9] = {
		{0, 0, 24, 32},
		{24, 0, 48, 32},
		{48, 0, 72, 32},
		{0, 0, 0, 0},
		{72, 0, 96, 32},
		{96, 0, 120, 32},
		{120, 0, 144, 32},
		{144, 0, 168, 32},
		{264, 0, 288, 32},
	};

	RECT rcRight[9] = {
		{0, 32, 24, 64},
		{24, 32, 48, 64},
		{48, 32, 72, 64},
		{0, 0, 0, 0},
		{72, 32, 96, 64},
		{96, 32, 120, 64},
		{120, 32, 144, 64},
		{144, 32, 168, 64},
		{264, 32, 288, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 8 * 0x200;
			npc->ani_no = 3;
			break;

		case 2:
			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 0;
			else
				npc->ani_no = 3;

			if (npc->act_wait > 50)
				npc->act_no = 10;

			break;

		case 10:
			npc->ym += 0x80;
			npc->bits |= NPC_SHOOTABLE;
			npc->damage = 3;

			if (npc->flag & 8)
			{
				npc->act_no = 20;
				npc->act_wait = 0;
				npc->ani_no = 0;
				npc->count2 = npc->life;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 20:
			if (++npc->act_wait < 50 && npc->life < npc->count2 - 20)
				npc->act_wait = 50;

			if (npc->act_wait == 50)
			{
				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;

				npc->ani_no = 4;
			}

			if (npc->act_wait == 80)
			{
				npc->ani_no = 5;
				PlaySoundObject(25, SOUND_MODE_PLAY);

				if (npc->direct == 0)
				{
					SetNpChar(264, npc->x - (16 * 0x200), npc->y, 0, 0, 0, NULL, 0x100);
					SetNpChar(264, npc->x - (16 * 0x200), npc->y, 0, 0, 0x400, NULL, 0x100);
				}
				else
				{
					SetNpChar(264, npc->x + (16 * 0x200), npc->y, 0, 0, 2, NULL, 0x100);
					SetNpChar(264, npc->x + (16 * 0x200), npc->y, 0, 0, 2 + 0x400, NULL, 0x100);
				}
			}

			if (npc->act_wait == 120)
				npc->ani_no = 0;

			if (npc->act_wait > 130 && npc->life < npc->count2 - 50)
				npc->act_wait = 161;

			if (npc->act_wait > 160)
			{
				npc->act_no = 100;
				npc->ani_no = 0;
			}

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;
			npc->ani_no = 6;
			npc->tgt_x = npc->x;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 31:
			if (++npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x;
			else
				npc->x = npc->tgt_x + 0x200;

			if (npc->act_wait > 50)
			{
				npc->act_no = 32;
				npc->act_wait = 0;
				npc->ani_no = 7;
				PlaySoundObject(101, SOUND_MODE_PLAY);

				for (deg = 8; deg < 0x100; deg += 0x10)
				{
					xm = GetCos(deg) * 2;
					ym = GetSin(deg) * 2;
					SetNpChar(266, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
				}
			}

			break;

		case 32:
			if (++npc->act_wait > 50)
				npc->act_no = 100;

			break;

		case 100:
			npc->act_no = 101;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->damage = 0;
			npc->act_wait = 0;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 101:
			npc->act_wait += 2;

			if (npc->act_wait > 16)
			{
				npc->act_no = 102;
				npc->act_wait = 0;
				npc->ani_no = 3;
				npc->tgt_x = Random(5, 35) * 0x200 * 0x10;
				npc->tgt_y = Random(5, 7) * 0x200 * 0x10;
			}

			break;

		case 102:
			if (++npc->act_wait > 40)
			{
				npc->act_no = 103;
				npc->act_wait = 16;
				npc->ani_no = 2;
				npc->ym = 0;
				npc->x = npc->tgt_x;
				npc->y = npc->tgt_y;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 103:
			npc->act_wait -= 2;

			if (npc->act_wait <= 0)
			{
				npc->bits |= NPC_SHOOTABLE;
				npc->damage = 3;

				if (npc->count1 < 3)
				{
					++npc->count1;
					npc->act_no = 10;
				}
				else
				{
					npc->count1 = 0;
					npc->act_no = 30;
				}
			}

			break;

		case 500:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->ani_no = 6;
			npc->ym += 0x10;

			if (npc->flag & 8)
			{
				npc->act_no = 501;
				npc->act_wait = 0;
				npc->tgt_x = npc->x;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 501:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->ani_no = 8;

			if (++npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x;
			else
				npc->x = npc->tgt_x + (1 * 0x200);

			break;
	}

	if (npc->act_no >= 10)
	{
		if (npc->act_no == 102)
		{
			gSuperXpos = npc->tgt_x;
			gSuperYpos = npc->tgt_y;
		}
		else
		{
			gSuperXpos = npc->x;
			gSuperYpos = npc->y;
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

	if (npc->act_no == 101 || npc->act_no == 103)
	{
		npc->rect.top += npc->act_wait;
		npc->rect.bottom -= npc->act_wait;
		npc->view.top = (16 - npc->act_wait) * 0x200;
	}
	else
	{
		npc->view.top = 16 * 0x200;
	}
}

// Doctor red wave (projectile)
void ActNpc264(NPCHAR *npc)
{
	RECT rc = {288, 0, 304, 16};
	unsigned char deg;

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
			npc->count1 = npc->direct / 8;
			npc->direct %= 8;
			// Fallthrough
		case 1:
			npc->count1 += 6;
			npc->count1 %= 0x100;
			deg = npc->count1;

			if (npc->act_wait < 128)
				++npc->act_wait;

			if (npc->direct == 0)
				npc->xm -= 21;
			else
				npc->xm += 21;

			npc->tgt_x += npc->xm;

			npc->x = npc->tgt_x + (GetCos(deg) * npc->act_wait) / 2 / 4;
			npc->y = npc->tgt_y + (GetSin(deg) * npc->act_wait) / 2;

			SetNpChar(265, npc->x, npc->y, 0, 0, 0, NULL, 0x100);

			break;
	}

	npc->rect = rc;
}

// Doctor red ball projectile
void ActNpc265(NPCHAR *npc)
{
	RECT rc[3] = {
		{288, 16, 304, 32},
		{288, 32, 304, 48},
		{288, 48, 304, 64},
	};

	if (++npc->ani_wait > 3)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 2)
		npc->cond = 0;
	else
		npc->rect = rc[npc->ani_no];
}

// Doctor red ball projectile (bouncing)
void ActNpc266(NPCHAR *npc)
{
	RECT rc[2] = {
		{304, 16, 320, 32},
		{304, 32, 320, 48},
	};

	if (npc->flag & 1)
		npc->xm *= -1;
	if (npc->flag & 4)
		npc->xm *= -1;

	if (npc->flag & 2)
		npc->ym = 0x200;
	if (npc->flag & 8)
		npc->ym = -0x200;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (++npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rc[npc->ani_no];

	if (++npc->act_wait % 4 == 1)
		SetNpChar(265, npc->x, npc->y, 0, 0, 0, NULL, 0x100);

	if (npc->act_wait > 250)
		VanishNpChar(npc);
}

// Muscle Doctor
void ActNpc267(NPCHAR *npc)
{
	RECT rcLeft[10] = {
		{0, 0, 0, 0},
		{0, 64, 40, 112},
		{40, 64, 80, 112},
		{80, 64, 120, 112},
		{120, 64, 160, 112},
		{160, 64, 200, 112},
		{200, 64, 240, 112},
		{240, 64, 280, 112},
		{280, 64, 320, 112},
		{0, 160, 40, 208},
	};

	RECT rcRight[10] = {
		{0, 0, 0, 0},
		{0, 112, 40, 160},
		{40, 112, 80, 160},
		{80, 112, 120, 160},
		{120, 112, 160, 160},
		{160, 112, 200, 160},
		{200, 112, 240, 160},
		{240, 112, 280, 160},
		{280, 112, 320, 160},
		{40, 160, 80, 208},
	};

	int xm, ym;
	int i;

	switch (npc->act_no)
	{
		case 0:
			if (gSuperXpos > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->direct == 0)
				npc->x = gSuperXpos - (6 * 0x200);
			else
				npc->x = gSuperXpos + (6 * 0x200);

			npc->y = gSuperYpos;
			// Fallthrough
		case 1:
			npc->act_no = 2;
			// Fallthrough
		case 2:
			npc->ym += 0x80;

			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 0;
			else
				npc->ani_no = 3;

			break;

		case 5:
			npc->act_no = 6;
			npc->ani_no = 1;
			npc->ani_wait = 0;
			// Fallthrough
		case 6:
			npc->ym += 0x80;

			if (++npc->ani_wait > 40)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

			break;

		case 7:
			npc->act_no = 8;
			npc->act_wait = 0;
			npc->ani_no = 3;
			// Fallthrough
		case 8:
			npc->ym += 0x40;

			if (++npc->act_wait > 40)
				npc->act_no = 10;

			break;

		case 10:
			npc->bits |= NPC_INVULNERABLE;
			npc->xm = 0;
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_no = 1;
			npc->ani_wait = 0;
			npc->count2 = npc->life;
			// Fallthrough
		case 11:
			npc->ym += 0x80;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->flag & 8)
			{
				if (npc->life < npc->count2 - 20)
				{
					if (gMC.flag & 8 && gMC.x > npc->x - (48 * 0x200) && gMC.x < npc->x + (48 * 0x200) && npc->ani_no != 6)
					{
						npc->ani_no = 6;
						DamageMyChar(5);
						SetQuake(10);
						PlaySoundObject(26, SOUND_MODE_PLAY);
						gMC.ym = -0x400;

						if (npc->x > gMC.x)
							gMC.xm = -0x5FF;
						else
							gMC.xm = 0x5FF;

						for (i = 0; i < 100; ++i)
							SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y + (Random(-0x10, 0x10) * 0x200), Random(-0x200, 0x200) * 3, Random(-0x200, 0x200) * 3, 3, NULL, 0xAA);
					}
				}
				else
				{
					if (++npc->ani_wait > 10)
					{
						npc->ani_wait = 0;

						if (++npc->ani_no > 2)
							npc->ani_no = 1;
					}
				}
			}
			else
			{
				npc->ani_no = 4;
			}

			if (++npc->act_wait > 30 || npc->life < npc->count2 - 20)
			{
				if (++npc->count1 > 10)
					npc->count1 = 0;

				switch (npc->count1)
				{
					case 8:
						npc->act_no = 20;
						break;

					case 2:
					case 7:
						npc->act_no = 100;
						break;

					case 3:
					case 6:
						npc->act_no = 30;
						break;

					case 1:
					case 9:
						npc->act_no = 40;
						break;

					default:
						npc->act_no = 15;
						npc->act_wait = 0;
						break;
				}
			}

			break;

		case 15:
			npc->ani_no = 3;
			++npc->act_wait;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->act_wait > 20)
			{
				npc->act_no = 16;
				npc->ani_no = 4;
				npc->ani_wait = 0;
				npc->ym = -0x600;

				if (npc->direct == 0)
					npc->xm = -0x400;
				else
					npc->xm = 0x400;
			}

			break;

		case 16:
			npc->ym += 0x40;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 4;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->ym > 0 && npc->flag & 8)
				npc->act_no = 17;

			break;

		case 17:
			npc->act_no = 18;
			npc->act_wait = 0;
			SetQuake(10);
			PlaySoundObject(26, SOUND_MODE_PLAY);
			// Fallthrough
		case 18:
			npc->ani_no = 3;
			++npc->act_wait;

			npc->xm = (npc->xm * 7) / 8;
			npc->ym += 0x80;

			if (npc->act_wait > 10)
				npc->act_no = 10;

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = 0;
			// Fallthrough
		case 21:
			++npc->act_wait;
			npc->ani_no = 6;

			if (npc->act_wait > 20 && npc->act_wait % 3 == 1)
			{
				ym = Random(-0x200, 0x200);
				xm = Random(0x100, 0x200) * 4;

				if (npc->direct == 0)
					SetNpChar(269, npc->x - 0x1000, npc->y - 0x800, -xm, ym, 0, NULL, 0x100);
				else
					SetNpChar(269, npc->x + 0x1000, npc->y - 0x800, xm, ym, 2, NULL, 0x100);

				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			if (npc->act_wait > 90)
				npc->act_no = 10;

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;
			npc->bits |= NPC_SOLID_SOFT;
			npc->bits &= ~NPC_SHOOTABLE;
			// Fallthrough
		case 31:
			npc->ani_no = 3;

			if (++npc->act_wait > 20)
			{
				npc->act_no = 32;
				npc->act_wait = 0;
				npc->ani_no = 7;
				npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
				npc->damage = 10;
				PlaySoundObject(25, SOUND_MODE_PLAY);

				if (npc->direct == 0)
					npc->xm = -0x5FF;
				else
					npc->xm = 0x5FF;
			}

			break;

		case 32:
			++npc->act_wait;
			npc->ym = 0;

			if (npc->act_wait / 2 % 2)
				npc->ani_no = 7;
			else
				npc->ani_no = 8;

			if (npc->act_wait > 30)
			{
				npc->act_wait = 0;
				npc->act_no = 18;
				npc->damage = 5;
				npc->bits &= ~(NPC_SOLID_SOFT | NPC_REAR_AND_TOP_DONT_HURT);
				npc->bits |= NPC_SHOOTABLE;
			}

			if (npc->flag & 1 || npc->flag & 4)
			{
				npc->act_no = 15;
				npc->act_wait = 0;
				npc->damage = 5;
				npc->bits &= ~(NPC_SOLID_SOFT | NPC_REAR_AND_TOP_DONT_HURT);
				npc->bits |= NPC_SHOOTABLE;
			}

			break;

		case 40:
			npc->ani_no = 3;
			++npc->act_wait;

			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (npc->act_wait > 20)
			{
				npc->act_no = 41;
				npc->ani_no = 4;
				npc->ani_wait = 0;
				npc->ym = -0x800;

				if (npc->direct == 0)
					npc->xm = -0x400;
				else
					npc->xm = 0x400;
			}

			break;

		case 41:
			npc->ym += 0x40;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 4;

			if (gMC.y > npc->y && gMC.x > npc->x - (8 * 0x200) && gMC.x < npc->x + (8 * 0x200))
			{
				npc->act_no = 16;
				npc->ym = 0x5FF;
				npc->xm = 0;
			}

			if (npc->ym > 0 && npc->flag & 8)
				npc->act_no = 17;

			break;

		case 100:
			npc->act_no = 101;
			npc->act_wait = 0;
			npc->bits &= ~(NPC_INVULNERABLE | NPC_SHOOTABLE);
			npc->damage = 0;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 101:
			npc->act_wait += 2;

			if (npc->act_wait > 28)
			{
				npc->act_no = 102;
				npc->act_wait = 0;
				npc->ani_no = 0;

				npc->tgt_x = gMC.x;
				npc->tgt_y = gMC.y - (32 * 0x200);

				if (npc->tgt_y < (64 * 0x200))
					npc->tgt_y = (64 * 0x200);

				if (npc->tgt_x < (64 * 0x200))
					npc->tgt_x = (64 * 0x200);
				if (npc->tgt_x > (576 * 0x200))
					npc->tgt_x = (576 * 0x200);
			}

			break;

		case 102:
			if (++npc->act_wait > 40)
			{
				npc->act_no = 103;
				npc->act_wait = 28;
				npc->ani_no = 4;
				npc->ym = 0;
				npc->x = npc->tgt_x;
				npc->y = npc->tgt_y;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 103:
			npc->act_wait -= 2;

			if (npc->act_wait <= 0)
			{
				npc->bits |= (NPC_INVULNERABLE | NPC_SHOOTABLE);
				npc->damage = 5;
				npc->act_no = 16;
				npc->ym = -0x200;
				npc->xm = 0;
			}

			break;

		case 500:
			DeleteNpCharCode(269, TRUE);
			npc->bits &= ~NPC_SHOOTABLE;
			npc->ani_no = 4;
			npc->ym += 0x20;
			npc->xm = 0;

			if (npc->flag & 8)
			{
				npc->act_no = 501;
				npc->act_wait = 0;
				npc->tgt_x = npc->x;

				if (npc->x > gMC.x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 501:
			npc->ani_no = 9;

			if (++npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x;
			else
				npc->x = npc->tgt_x + (1 * 0x200);

			break;

		case 510:
			npc->act_no = 511;
			npc->act_wait = 0;
			npc->ani_no = 9;
			npc->tgt_x = npc->x;
			npc->y += 16 * 0x200;
			npc->bits |= NPC_IGNORE_SOLIDITY;
			// Fallthrough
		case 511:
			SetQuake(2);

			if (++npc->act_wait % 6 == 3)
				PlaySoundObject(25, SOUND_MODE_PLAY);

			if (npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x;
			else
				npc->x = npc->tgt_x + (1 * 0x200);

			if (npc->act_wait > 352)
			{
				npc->ani_no = 0;
				npc->act_no = 0x200;
			}

			break;

		case 520:
			npc->damage = 0;
			gSuperYpos = -32 * 0x200;
			break;
	}

	if (npc->act_no >= 11 && npc->act_no < 501)
	{
		if (npc->act_no == 102)
		{
			gSuperXpos = npc->tgt_x;
			gSuperYpos = npc->tgt_y;
		}
		else
		{
			gSuperXpos = npc->x;
			gSuperYpos = npc->y;
		}
	}

	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->act_no >= 512)
	{
		// There probably used to be some commented-out code here
	}
	else if (npc->act_no < 510)
	{
		if (npc->act_no != 102 && npc->act_no != 103 && Random(0, 3) == 2)
			SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y + (Random(-8, 4) * 0x200), npc->xm, 0, 3, 0, 0x100);
	}
	else
	{
		SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y - (((336 - npc->act_wait) / 8) * 0x200), Random(-0x200, 0x200), Random(-0x200, 0) * 2, 3, NULL, 0xAA);
		SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y - (((336 - npc->act_wait) / 8) * 0x200), Random(-0x200, 0x200), Random(-0x200, 0) * 2, 3, NULL, 0xAA);
		SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y - (((336 - npc->act_wait) / 8) * 0x200), 0, 2 * Random(-0x200, 0), 3, NULL, 0xAA);
		SetNpChar(270, npc->x + (Random(-0x10, 0x10) * 0x200), npc->y - (((336 - npc->act_wait) / 8) * 0x200), 0, 2 * Random(-0x200, 0), 3, NULL, 0xAA);
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];

	if (npc->act_no == 511)
	{
		npc->rect.top += npc->act_wait / 8;
		npc->view.top = (44 - (npc->act_wait / 8)) * 0x200;
		npc->view.bottom = 4 * 0x200;
	}
	else if (npc->act_no == 101 || npc->act_no == 103)
	{
		npc->rect.top += npc->act_wait;
		npc->rect.bottom -= npc->act_wait;
		npc->view.top = (28 - npc->act_wait) * 0x200;
	}
	else
	{
		npc->view.top = 28 * 0x200;
	}
}

// Igor (enemy)
void ActNpc268(NPCHAR *npc)
{
	unsigned char deg;
	int xm, ym;

	RECT rcLeft[10] = {
		{0, 0, 40, 40},
		{40, 0, 80, 40},
		{80, 0, 120, 40},
		{0, 0, 40, 40},
		{120, 0, 160, 40},
		{0, 0, 40, 40},
		{40, 80, 80, 120},
		{0, 80, 40, 120},
		{240, 0, 280, 40},
		{280, 0, 320, 40},
	};

	RECT rcRight[10] = {
		{0, 40, 40, 80},
		{40, 40, 80, 80},
		{80, 40, 120, 80},
		{0, 40, 40, 80},
		{120, 40, 160, 80},
		{0, 40, 40, 80},
		{160, 80, 200, 120},
		{120, 80, 160, 120},
		{240, 40, 280, 80},
		{280, 40, 320, 80},
	};

	if (npc->x < gMC.x - (320 * 0x200) || npc->x > gMC.x + (320 * 0x200) || npc->y < gMC.y - (240 * 0x200) || npc->y > gMC.y + (240 * 0x200))
		npc->act_no = 1;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->y += 8 * 0x200;
			// Fallthrough
		case 1:
			if (++npc->ani_wait > 20)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

#ifdef FIX_BUGS
			if (npc->x < gMC.x + (112 * 0x200) && npc->x > gMC.x - (112 * 0x200) && npc->y < gMC.y + (48 * 0x200) && npc->y > gMC.y - (112 * 0x200))
#else
			// Instead of checking the X and Y coordinates, this checks the X coordinates twice
			if (npc->x < gMC.x + (112 * 0x200) && npc->x > gMC.x - (112 * 0x200) && npc->x < gMC.x + (48 * 0x200) && npc->x > gMC.x - (112 * 0x200))
#endif
				npc->act_no = 10;

			if (npc->shock)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_no = 0;
			npc->ani_wait = 0;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 11:
			if (npc->direct == 0)
				npc->xm = -0x200;
			else
				npc->xm = 0x200;

			if (npc->x < gMC.x + (64 * 0x200) && npc->x > gMC.x - (64 * 0x200))
			{
				npc->act_no = 20;
				npc->act_wait = 0;
			}

			if (npc->xm < 0 && npc->flag & 1)
			{
				npc->act_no = 20;
				npc->act_wait = 0;
			}

			if (npc->xm > 0 && npc->flag & 4)
			{
				npc->act_no = 20;
				npc->act_wait = 0;
			}

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 2;

			break;

		case 20:
			npc->xm = 0;
			npc->ani_no = 6;

			if (++npc->act_wait > 10)
			{
				npc->act_no = 30;
				npc->ym = -0x5FF;

				if (npc->direct == 0)
					npc->xm = -0x200;
				else
					npc->xm = 0x200;

				PlaySoundObject(108, SOUND_MODE_PLAY);
			}

			break;

		case 30:
			npc->ani_no = 7;

			if (npc->flag & 8)
			{
				npc->act_no = 40;
				npc->act_wait = 0;
				SetQuake(20);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			break;

		case 40:
			npc->xm = 0;
			npc->ani_no = 6;

			if (++npc->act_wait > 30)
				npc->act_no = 50;

			break;

		case 50:
			npc->act_no = 51;
			npc->act_wait = 0;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;
			// Fallthrough
		case 51:
			if (++npc->act_wait > 30 && npc->act_wait % 4 == 1)
			{
				if (npc->direct == 0)
					deg = 0x88;
				else
					deg = 0xF8;

				deg += (unsigned char)Random(-0x10, 0x10);
				ym = GetSin(deg) * 5;
				xm = GetCos(deg) * 5;
				SetNpChar(11, npc->x, npc->y + (4 * 0x200), xm, ym, 0, NULL, 0x100);
				PlaySoundObject(12, SOUND_MODE_PLAY);
			}

			if (npc->act_wait < 50 && npc->act_wait / 2 % 2)
				npc->ani_no = 9;
			else
				npc->ani_no = 8;

			if (npc->act_wait > 82)
			{
				npc->act_no = 10;

				if (gMC.x < npc->x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;
	}

	npc->ym += 0x200 / 10;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Red Bat (bouncing)
void ActNpc269(NPCHAR *npc)
{
	RECT rcLeft[3] = {
		{232, 0, 248, 16},
		{248, 0, 264, 16},
		{248, 16, 264, 32},
	};

	RECT rcRight[3] = {
		{232, 32, 248, 48},
		{248, 32, 264, 48},
		{248, 48, 264, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->xm2 = npc->xm;
			npc->ym2 = npc->ym;
			// Fallthrough
		case 1:
			if (npc->xm2 < 0 && npc->flag & 1)
			{
				npc->direct = 2;
				npc->xm2 *= -1;
			}
			else if (npc->xm2 > 0 && npc->flag & 4)
			{
				npc->direct = 0;
				npc->xm2 *= -1;
			}
			else if (npc->ym2 < 0 && npc->flag & 2)
			{
				npc->ym2 *= -1;
			}
			else if (npc->ym2 > 0 && npc->flag & 8)
			{
				npc->ym2 *= -1;
			}

			npc->x += npc->xm2;
			npc->y += npc->ym2;

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 0;

			break;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Doctor's blood (or """"red energy"""")
void ActNpc270(NPCHAR *npc)
{
	RECT rc[2] = {
		{170, 34, 174, 38},
		{170, 42, 174, 46},
	};

	if (npc->direct == 3 || npc->direct == 1)
	{
		if (npc->direct == 3)
			npc->ym += 0x40;
		if (npc->direct == 1)
			npc->ym -= 0x40;

		++npc->act_wait;

		if (npc->ym > 0x5FF)
			npc->ym = 0x5FF;

		npc->x += npc->xm;
		npc->y += npc->ym;

		if (npc->act_wait > 50)
			npc->cond = 0;

		if (npc->flag & 0xFF)
			npc->cond = 0;
	}
	else if (npc->direct == 2)
	{
		switch (npc->act_no)
		{
			case 0:
				npc->act_no = 1;
				npc->bits |= NPC_IGNORE_SOLIDITY;

				npc->xm = Random(-0x200, 0x200) * 3;
				npc->ym = Random(-0x200, 0x200) * 3;

				npc->count1 = Random(0x10, 0x33);
				npc->count2 = Random(0x80, 0x100);

				break;
		}

		if (npc->x < npc->pNpc->x)
			npc->xm += 0x200 / npc->count1;
		if (npc->x > npc->pNpc->x)
			npc->xm -= 0x200 / npc->count1;

		if (npc->y < npc->pNpc->y)
			npc->ym += 0x200 / npc->count1;
		if (npc->y > npc->pNpc->y)
			npc->ym -= 0x200 / npc->count1;

		if (npc->xm > npc->count2 * 2)
			npc->xm = npc->count2 * 2;
		if (npc->xm < -npc->count2 * 2)
			npc->xm = -npc->count2 * 2;

		if (npc->ym > npc->count2 * 3)
			npc->ym = npc->count2 * 3;
		if (npc->ym < -npc->count2 * 3)
			npc->ym = -npc->count2 * 3;

		npc->x += npc->xm;
		npc->y += npc->ym;
	}

	npc->rect = rc[Random(0, 1)];
}

// Ironhead block
void ActNpc271(NPCHAR *npc)
{
	int a;

	if (npc->xm < 0 && npc->x < -16 * 0x200)
	{
		VanishNpChar(npc);
		return;
	}

	if (npc->xm > 0 && npc->x > (gMap.width * 0x200 * 0x10) + (1 * 0x200 * 0x10))
	{
		VanishNpChar(npc);
		return;
	}

	if (npc->act_no == 0)
	{
		npc->act_no = 1;

		a = Random(0, 9);

		if (a == 9)
		{
			npc->rect.left = 0;
			npc->rect.right = 0x20;
			npc->rect.top = 0x40;
			npc->rect.bottom = 0x60;

			npc->view.front = 16 * 0x200;
			npc->view.back = 16 * 0x200;
			npc->view.top = 16 * 0x200;
			npc->view.bottom = 16 * 0x200;

			npc->hit.front = 12 * 0x200;
			npc->hit.back = 12 * 0x200;
			npc->hit.top = 12 * 0x200;
			npc->hit.bottom = 12 * 0x200;
		}
		else
		{
			npc->rect.left = ((a % 3) * 16) + (7 * 16);
			npc->rect.top = (a / 3) * 16;
			npc->rect.right = npc->rect.left + 16;
			npc->rect.bottom = npc->rect.top + 16;
		}

		if (npc->direct == 0)
			npc->xm = Random(0x100, 0x200) * -2;
		else
			npc->xm = Random(0x100, 0x200) * 2;

		npc->ym = Random(-0x200, 0x200);
	}

	if (npc->ym < 0 && npc->y - npc->hit.top < 8 * 0x200)
	{
		npc->ym *= -1;
		SetCaret(npc->x, npc->y - (8 * 0x200), CARET_TINY_PARTICLES, DIR_LEFT);
		SetCaret(npc->x, npc->y - (8 * 0x200), CARET_TINY_PARTICLES, DIR_LEFT);
	}

	if (npc->ym > 0 && npc->y + npc->hit.bottom > 232 * 0x200)
	{
		npc->ym *= -1;
		SetCaret(npc->x, npc->y + (8 * 0x200), CARET_TINY_PARTICLES, DIR_LEFT);
		SetCaret(npc->x, npc->y + (8 * 0x200), CARET_TINY_PARTICLES, DIR_LEFT);
	}

	npc->x += npc->xm;
	npc->y += npc->ym;
}

// Ironhead block generator
void ActNpc272(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->act_wait = Random(0, 200);
			// Fallthrough
		case 1:
			if (npc->act_wait)
			{
				--npc->act_wait;
			}
			else
			{
				npc->act_no = 0;
				SetNpChar(271, npc->x, npc->y + (Random(-32, 32) * 0x200), 0, 0, npc->direct, NULL, 0x100);
			}

			break;
	}
}

// Droll projectile
void ActNpc273(NPCHAR *npc)
{
	RECT rc[3] = {
		{248, 40, 272, 64},
		{272, 40, 296, 64},
		{296, 40, 320, 64},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 0xFF)
			{
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				VanishNpChar(npc);
				return;
			}

			if (++npc->act_wait % 5 == 0)
				PlaySoundObject(110, SOUND_MODE_PLAY);

			if (++npc->ani_no > 2)
				npc->ani_no = 0;

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Droll
void ActNpc274(NPCHAR *npc)
{
	RECT rcLeft[6] = {
		{0, 0, 32, 40},
		{32, 0, 64, 40},
		{64, 0, 96, 40},
		{64, 80, 96, 120},
		{96, 80, 128, 120},
		{96, 0, 128, 40},
	};

	RECT rcRight[6] = {
		{0, 40, 32, 80},
		{32, 40, 64, 80},
		{64, 40, 96, 80},
		{64, 120, 96, 160},
		{96, 120, 128, 160},
		{96, 40, 128, 80},
	};

	unsigned char deg;
	int xm, ym;

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

			if (npc->shock)
				npc->act_no = 10;

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
				npc->count1 = 0;

				if (npc->tgt_x > npc->x)
					npc->xm = 0x200;
				else
					npc->xm = -0x200;
			}

			break;

		case 12:
			if (npc->ym > 0)
			{
				npc->ani_no = 4;

				if (npc->count1 == 0)
				{
					++npc->count1;
					deg = GetArktan(npc->x - gMC.x, npc->y - (10 * 0x200) - gMC.y);
					ym = GetSin(deg) * 4;
					xm = GetCos(deg) * 4;
					SetNpChar(273, npc->x, npc->y - (10 * 0x200), xm, ym, 0, NULL, 0x100);
					PlaySoundObject(39, SOUND_MODE_PLAY);
				}
			}

			if (npc->ym > 0x200)
				npc->ani_no = 5;

			if (npc->flag & 8)
			{
				npc->ani_no = 2;
				npc->act_no = 13;
				npc->act_wait = 0;
				npc->xm = 0;
			}

			break;

		case 13:
			npc->xm /= 2;

			if (++npc->act_wait > 10)
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
}

// Puppy (plantation)
void ActNpc275(NPCHAR *npc)
{
	RECT rcRight[4] = {
		{272, 80, 288, 96},
		{288, 80, 304, 96},
		{272, 80, 288, 96},
		{304, 80, 320, 96},
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

			if (npc->x - (64 * 0x200) < gMC.x && npc->x + (64 * 0x200) > gMC.x && npc->y - (32 * 0x200) < gMC.y && npc->y + (16 * 0x200) > gMC.y)
			{
				if (++npc->ani_wait > 3)
				{
					npc->ani_wait = 0;
					++npc->ani_no;
				}

				if (npc->ani_no > 3)
					npc->ani_no = 2;
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

	npc->ym += 0x40;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rcRight[npc->ani_no];
}

// Red Demon
void ActNpc276(NPCHAR *npc)
{
	RECT rcLeft[9] = {
		{0, 64, 32, 104},
		{32, 64, 64, 104},
		{64, 64, 96, 104},
		{96, 64, 128, 104},
		{128, 64, 160, 104},
		{160, 64, 192, 104},
		{192, 64, 224, 104},
		{224, 64, 256, 104},
		{256, 64, 288, 104},
	};

	RECT rcRight[9] = {
		{0, 104, 32, 144},
		{32, 104, 64, 144},
		{64, 104, 96, 144},
		{96, 104, 128, 144},
		{128, 104, 160, 144},
		{160, 104, 192, 144},
		{192, 104, 224, 144},
		{224, 104, 256, 144},
		{256, 104, 288, 144},
	};

	unsigned char deg;
	int xm, ym;

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

			if (++npc->ani_wait > 20)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (npc->shock)
				npc->act_no = 10;

			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->ani_no = 3;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 11:
			switch (++npc->act_wait)
			{
				case 30:
				case 40:
				case 50:
					npc->ani_no = 4;

					deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
					ym = GetSin(deg) * 4;
					xm = GetCos(deg) * 4;

					SetNpChar(277, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
					PlaySoundObject(39, SOUND_MODE_PLAY);

					break;

				case 34:
				case 44:
				case 54:
					npc->ani_no = 3;
					break;
			}

			if (npc->act_wait > 60)
			{
				npc->act_no = 20;
				npc->act_wait = 0;
				npc->ani_no = 2;
			}

			break;

		case 20:
			if (++npc->act_wait > 20)
			{
				npc->act_no = 21;
				npc->act_wait = 0;
				npc->ani_no = 5;
				npc->ym = -0x5FF;

				if (npc->x < gMC.x)
					npc->xm = 0x100;
				else
					npc->xm = -0x100;
			}

			break;

		case 21:
			switch (++npc->act_wait)
			{
				case 30:
				case 40:
				case 50:
					npc->ani_no = 6;

					deg = GetArktan(npc->x - gMC.x, npc->y - (10 * 0x200) - gMC.y);
					ym = GetSin(deg) * 4;
					xm = GetCos(deg) * 4;

					SetNpChar(277, npc->x, npc->y - (10 * 0x200), xm, ym, 0, NULL, 0x100);
					PlaySoundObject(39, SOUND_MODE_PLAY);

					break;

				case 34:
				case 44:
					npc->ani_no = 5;
					break;
			}

			if (npc->act_wait > 53)
				npc->ani_no = 7;

			if (npc->flag & 8)
			{
				npc->act_no = 22;
				npc->act_wait = 0;
				npc->ani_no = 2;
				SetQuake(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			break;

		case 22:
			npc->xm /= 2;

			if (++npc->act_wait > 22)
				npc->act_no = 10;

			break;

		case 50:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->damage = 0;

			if (npc->flag & 8)
			{
				npc->act_no = 51;
				npc->ani_no = 2;
				SetQuake(10);
				SetExpObjects(npc->x, npc->y, 19);
				SetDestroyNpChar(npc->x, npc->y, npc->view.back, 8);
				PlaySoundObject(72, SOUND_MODE_PLAY);
			}

			break;

		case 51:
			npc->xm = (npc->xm * 7) / 8;
			npc->ani_no = 8;
			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->act_no < 50)
	{
		if (npc->x < gMC.x)
			npc->direct = 2;
		else
			npc->direct = 0;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Red Demon projectile
void ActNpc277(NPCHAR *npc)
{
	RECT rc[3] = {
		{128, 0, 152, 24},
		{152, 0, 176, 24},
		{176, 0, 200, 24},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->flag & 0xFF)
			{
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);
				VanishNpChar(npc);
				return;
			}

			if (++npc->act_wait % 5 == 0)
				PlaySoundObject(110, SOUND_MODE_PLAY);

			if (++npc->ani_no > 2)
				npc->ani_no = 0;

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Little family
void ActNpc278(NPCHAR *npc)
{
	RECT rcPapa[2] = {
		{0, 120, 8, 128},
		{8, 120, 16, 128},
	};

	RECT rcMama[2] = {
		{16, 120, 24, 128},
		{24, 120, 32, 128},
	};

	RECT rcKodomo[2] = {	 // Japanese for 'child'
		{32, 120, 40, 128},
		{40, 120, 48, 128},
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
			npc->ani_no = 0;
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
				npc->xm = -0x100;
			else
				npc->xm = 0x100;

			if (++npc->ani_wait > 4)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			if (++npc->act_wait > 0x20)
				npc->act_no = 0;

			break;
	}

	npc->ym += 0x20;
	if (npc->ym > 0x5FF)
		npc->ym = 0x5FF;

	npc->x += npc->xm;
	npc->y += npc->ym;


	switch (npc->code_event)
	{
		case 200:
			npc->rect = rcPapa[npc->ani_no];
			break;

		case 210:
			npc->rect = rcMama[npc->ani_no];
			break;

		default:
			npc->rect = rcKodomo[npc->ani_no];
			break;
	}
}

// Falling block (large)
void ActNpc279(NPCHAR *npc)
{
	RECT rc[2] = {
		{0, 16, 32, 48},
		{16, 0, 32, 16},
	};

	int i;

	switch (npc->act_no)
	{
		case 0:
			switch (npc->direct)
			{
				case 0:
					npc->act_no = 100;
					npc->bits |= NPC_INVULNERABLE;
					npc->ani_no = 0;
					break;

				case 2:
					npc->act_no = 100;
					npc->bits |= NPC_INVULNERABLE;
					npc->ani_no = 1;

					npc->view.back = 8 * 0x200;
					npc->view.front = 8 * 0x200;
					npc->view.top = 8 * 0x200;
					npc->view.bottom = 8 * 0x200;

					npc->hit.back = 8 * 0x200;
					npc->hit.front = 8 * 0x200;
					npc->hit.top = 8 * 0x200;
					npc->hit.bottom = 8 * 0x200;

					break;

				case 1:
					npc->ani_no = 0;
					npc->act_no = 10;
					break;
			}

			if (npc->direct != 1)
				break;
			// Fallthrough
		case 10:
			npc->act_no = 11;
			npc->act_wait = 16;
			// Fallthrough
		case 11:
			npc->act_wait -= 2;

			if (npc->act_wait <= 0)
			{
				npc->act_no = 100;
				npc->bits |= NPC_INVULNERABLE;
			}

			break;

		case 100:
			npc->ym += 0x40;
			if (npc->ym > 0x700)
				npc->ym = 0x700;

			if (npc->y > 128 * 0x200)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->flag & 8)
			{
				npc->ym = -0x200;
				npc->act_no = 110;
				npc->bits |= NPC_IGNORE_SOLIDITY;
				PlaySoundObject(26, SOUND_MODE_PLAY);
				SetQuake(10);

				for (i = 0; i < 4; ++i)
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

	if (gMC.y > npc->y)
		npc->damage = 10;
	else
		npc->damage = 0;

	npc->y += npc->ym;
	npc->rect = rc[npc->ani_no];

	if (npc->act_no == 11)
	{
		npc->rect.top += npc->act_wait;
		npc->rect.bottom -= npc->act_wait;
		npc->view.top = (16 - npc->act_wait) * 0x200;
	}
}

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
#include "Draw.h"
#include "Flash.h"
#include "Flags.h"
#include "Frame.h"
#include "Game.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

// Ballos
void ActNpc340(NPCHAR *npc)
{
	int i;
	int x;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->cond = 0x80;
			npc->exp = 1;
			npc->direct = 0;
			npc->y -= 6 * 0x200;
			npc->damage = 0;
			SetNpChar(341, npc->x, npc->y - (16 * 0x200), 0, 0, 0, npc, 0x100);
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 100)
				npc->act_no = 100;

			break;

		case 100:
			npc->act_no = 110;
			npc->act_wait = 0;
			npc->ani_no = 1;
			npc->ani_wait = 0;
			npc->damage = 4;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 110:
			npc->act_no = 111;
			npc->damage = 3;
			npc->tgt_x = npc->life;
			// Fallthrough
		case 111:
			if (++npc->ani_wait > 10)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->ani_no = 1;

			++npc->act_wait;

			if (npc->life < npc->tgt_x - 50 || npc->act_wait > 150)
			{
				switch (npc->count2 % 5)
				{
					case 0:
					case 1:
					case 2:
					case 3:
						npc->act_no = 200;
						break;

					case 4:
						npc->act_no = 300;
						break;
				}

				++npc->count2;
			}

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			break;

		case 200:
			npc->act_no = 201;
			npc->count1 = 0;
			// Fallthrough
		case 201:
			if (npc->xm == 0)
				npc->act_no = 202;
			else
				npc->act_no = 203;

			npc->act_wait = 0;
			npc->ani_no = 3;
			npc->damage = 3;
			++npc->count1;
			// Fallthrough
		case 202:
			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			npc->xm = 8 * npc->xm / 9;
			npc->ym = 8 * npc->ym / 9;

			if (++npc->act_wait > 20)
				npc->act_no = 210;

			break;

		case 203:
			npc->xm = 8 * npc->xm / 9;
			npc->ym = 8 * npc->ym / 9;

			if (++npc->act_wait > 20)
			{
				if (gMC.y < npc->y + (12 * 0x200))
					npc->act_no = 220;
				else
					npc->act_no = 230;
			}

			break;

		case 210:
			npc->act_no = 211;
			npc->act_wait = 0;
			npc->ani_no = 6;
			npc->ani_wait = 0;
			npc->ym = 0;
			npc->damage = 10;

			if (gMC.x < npc->x)
				npc->direct = 0;
			else
				npc->direct = 2;

			PlaySoundObject(25, SOUND_MODE_PLAY);
			// Fallthrough
		case 211:
			if (npc->direct == 0)
				npc->xm = -0x800;
			else
				npc->xm = 0x800;

			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 6;
			else
				npc->ani_no = 7;

			if (npc->direct == 0 && npc->flag & 1)
			{
				npc->act_no = 212;
				npc->act_wait = 0;
				npc->damage = 3;
				SetQuake2(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			if (npc->direct == 2 && npc->flag & 4)
			{
				npc->act_no = 212;
				npc->act_wait = 0;
				npc->damage = 3;
				SetQuake2(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			if (npc->count1 < 4 && gMC.x > npc->x - (16 * 0x200) && gMC.x < npc->x + (16 * 0x200))
				npc->act_no = 201;

			break;

		case 212:
			++npc->act_wait;
			npc->xm = 0;
			npc->ani_no = 6;

			if (npc->act_wait > 30)
			{
				if (npc->count1 > 3)
					npc->act_no = 240;
				else
					npc->act_no = 201;
			}

			break;

		case 220:
			npc->act_no = 221;
			npc->act_wait = 0;
			npc->ani_no = 8;
			npc->ani_wait = 0;
			npc->xm = 0;
			npc->damage = 10;
			npc->direct = 0;
			PlaySoundObject(25, SOUND_MODE_PLAY);
			// Fallthrough
		case 221:
			npc->ym = -0x800;

			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 8;
			else
				npc->ani_no = 9;

			if (npc->y < (48 * 0x200))
			{
				npc->y = (48 * 0x200);
				npc->ym = 0;
				npc->act_no = 222;
				npc->act_wait = 0;
				npc->damage = 3;

				for (i = 0; i < 8; ++i)
				{
					x = npc->x + (Random(-0x10, 0x10) * 0x200);
					SetNpChar(4, x, npc->y - (10 * 0x200), 0, 0, 0, NULL, 0x100);
				}

				SetNpChar(332, npc->x - (12 * 0x200), npc->y - (12 * 0x200), 0, 0, 0, NULL, 0x100);
				SetNpChar(332, npc->x + (12 * 0x200), npc->y - (12 * 0x200), 0, 0, 2, NULL, 0x100);
				SetQuake2(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			if (npc->count1 < 4 && gMC.y > npc->y - (16 * 0x200) && gMC.y < npc->y + (16 * 0x200))
				npc->act_no = 201;

			break;

		case 222:
			++npc->act_wait;
			npc->xm = 0;
			npc->ani_no = 8;

			if (npc->act_wait > 30)
			{
				if (npc->count1 > 3)
					npc->act_no = 240;
				else
					npc->act_no = 201;
			}

			break;

		case 230:
			npc->act_no = 231;
			npc->act_wait = 0;
			npc->ani_no = 8;
			npc->ani_wait = 0;
			npc->xm = 0;
			npc->damage = 10;
			npc->direct = 2;
			PlaySoundObject(25, SOUND_MODE_PLAY);
			// Fallthrough
		case 231:
			npc->ym = 0x800;

			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 8;
			else
				npc->ani_no = 9;

			if (npc->flag & 8)
			{
				npc->act_no = 232;
				npc->act_wait = 0;
				npc->damage = 3;

				if (gMC.x < npc->x)
					npc->direct = 0;
				else
					npc->direct = 2;

				for (i = 0; i < 8; ++i)
				{
					x = npc->x + (Random(-0x10, 0x10) * 0x200);
					SetNpChar(4, x, npc->y + (10 * 0x200), 0, 0, 0, NULL, 0x100);
				}

				SetNpChar(332, npc->x - (12 * 0x200), npc->y + (12 * 0x200), 0, 0, 0, NULL, 0x100);
				SetNpChar(332, npc->x + (12 * 0x200), npc->y + (12 * 0x200), 0, 0, 2, NULL, 0x100);
				SetQuake2(10);
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			if (npc->count1 < 4 && gMC.y > npc->y - (16 * 0x200) && gMC.y < npc->y + (16 * 0x200))
				npc->act_no = 201;

			break;

		case 232:
			++npc->act_wait;
			npc->xm = 0;
			npc->ani_no = 3;

			if (npc->act_wait > 30)
			{
				if (npc->count1 > 3)
					npc->act_no = 242;
				else
					npc->act_no = 201;
			}

			break;

		case 240:
			npc->act_no = 241;
			npc->direct = 0;
			// Fallthrough
		case 241:
			npc->ym += 0x80;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (++npc->ani_wait / 2 % 2)
				npc->ani_no = 4;
			else
				npc->ani_no = 5;

			if (npc->flag & 8)
			{
				npc->act_no = 242;
				npc->act_wait = 0;
				npc->ani_no = 3;

				if (gMC.x < npc->x)
					npc->direct = 0;
				else
					npc->direct = 2;
			}

			break;

		case 242:
			npc->xm = 3 * npc->xm / 4;
			npc->ani_no = 3;

			if (++npc->act_wait > 10)
				npc->act_no = 110;

			break;

		case 300:
			npc->act_no = 310;
			npc->act_wait = 0;
			npc->ym = -0x600;

			if (npc->x > 320 * 0x200)
			{
				npc->direct = 2;
				npc->tgt_x = gMC.x;
				npc->tgt_y = 176 * 0x200;
			}
			else
			{
				npc->direct = 0;
				npc->tgt_x = gMC.x;
				npc->tgt_y = 176 * 0x200;
			}

			npc->ani_wait = 0;
			// Fallthrough
		case 310:
			++npc->ani_wait;

			if (++npc->act_wait > 200 && npc->ani_wait < 20)
				npc->direct = 2;
			else
				npc->direct = 0;

			if (npc->ani_wait / 2 % 2)
				npc->ani_no = 4;
			else
				npc->ani_no = 5;

			if (npc->x < npc->tgt_x)
				npc->xm += 0x40;
			else
				npc->xm -= 0x40;

			if (npc->y < npc->tgt_y)
				npc->ym += 0x40;
			else
				npc->ym -= 0x40;

			if (npc->xm > 0x400)
				npc->xm = 0x400;
			if (npc->xm < -0x400)
				npc->xm = -0x400;

			if (npc->ym > 0x400)
				npc->ym = 0x400;
			if (npc->ym < -0x400)
				npc->ym = -0x400;

			if (npc->act_wait > 200 && npc->act_wait % 40 == 1)
			{
				npc->ani_wait = 0;
				SetNpChar(333, gMC.x, 304 * 0x200, 0, 0, 0, NULL, 0x100);
			}

			if (npc->act_wait > 480)
			{
				npc->act_no = 320;
				npc->act_wait = 0;
			}

			break;

		case 320:
			npc->xm = 0;
			npc->ym = 0;
			npc->direct = 2;

			if (++npc->act_wait == 40)
				SetFlash(0, 0, FLASH_MODE_FLASH);

			if (npc->act_wait > 50 && npc->act_wait % 10 == 1)
			{
				x = ((4 * npc->act_wait - 200) / 10 + 2) * 0x200 * 0x10;
				SetNpChar(333, x, 304 * 0x200, 0, 0, 0, NULL, 0x100);
			}

			if (npc->act_wait > 140)
				npc->act_no = 240;

			if (++npc->ani_wait / 2 % 2)
				npc->ani_no = 4;
			else
				npc->ani_no = 5;

			break;

		case 1000:
			npc->act_no = 1001;
			npc->act_wait = 0;
			npc->ani_no = 10;
			npc->tgt_x = npc->x;
			npc->xm = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			SetDestroyNpChar(npc->x, npc->y, 0x10, 0x10);
			PlaySoundObject(72, SOUND_MODE_PLAY);
			// Fallthrough
		case 1001:
			npc->ym += 0x20;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (++npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x + 0x200;
			else
				npc->x = npc->tgt_x - 0x200;

			if (npc->flag & 8)
			{
				npc->act_no = 1002;
				npc->act_wait = 0;
			}

			break;

		case 1002:
			if (++npc->act_wait > 150)
			{
				npc->act_wait = 0;
				npc->act_no = 1003;
				npc->ani_no = 3;
			}

			if (npc->act_wait / 2 % 2)
				npc->x = npc->tgt_x + 0x200;
			else
				npc->x = npc->tgt_x - 0x200;

			break;

		case 1003:
			if (++npc->act_wait > 30)
			{
				npc->act_wait = 0;
				npc->act_no = 1004;
				npc->ani_no = 3;
				npc->ym -= 0xA00;
				npc->direct = 0;
				npc->bits |= NPC_IGNORE_SOLIDITY;
			}

			break;

		case 1004:
			if (npc->y < 0)
			{
				npc->xm = 0;
				npc->ym = 0;
				npc->act_no = 1005;
				npc->act_wait = 0;
				SetFlash(0, 0, FLASH_MODE_FLASH);
				PlaySoundObject(29, SOUND_MODE_PLAY);
			}

			if (++npc->act_wait / 2 % 2)
				npc->ani_no = 8;
			else
				npc->ani_no = 9;

			break;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	RECT rcLeft[11] = {
		{0, 0, 48, 40},
		{48, 0, 96, 40},
		{96, 0, 144, 40},
		{144, 0, 192, 40},
		{192, 0, 240, 40},
		{240, 0, 288, 40},
		{0, 80, 48, 120},
		{48, 80, 96, 120},
		{96, 80, 144, 120},
		{144, 80, 192, 120},
		{192, 80, 240, 120},
	};

	RECT rcRight[11] = {
		{0, 40, 48, 80},
		{48, 40, 96, 80},
		{96, 40, 144, 80},
		{144, 40, 192, 80},
		{192, 40, 240, 80},
		{240, 40, 288, 80},
		{0, 120, 48, 160},
		{48, 120, 96, 160},
		{96, 120, 144, 160},
		{144, 120, 192, 160},
		{192, 120, 240, 160},
	};

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Ballos 1 head
void ActNpc341(NPCHAR *npc)
{
	RECT rc[3] = {
		{288, 32, 320, 48},
		{288, 48, 320, 64},
		{288, 64, 320, 80},
	};

	if (npc->pNpc->act_no == 11 && npc->pNpc->act_wait > 50)
		++npc->ani_wait;

	if (npc->ani_wait > 4)
	{
		npc->ani_wait = 0;

		if (npc->ani_no < 2)
			++npc->ani_no;
	}

	if (npc->pNpc->ani_no)
		npc->cond = 0;

	npc->rect = rc[npc->ani_no];
}

// Ballos 3 eye
void ActNpc342(NPCHAR *npc)
{
	static int flash;

	RECT rc[3] = {
		{240, 48, 280, 88},
		{240, 88, 280, 128},
		{280, 48, 320, 88},
	};

	unsigned char deg;

	if (npc->act_no < 1000 && npc->pNpc->act_no >= 1000)
		npc->act_no = 1000;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->count1 = (npc->direct & 0xFF) * 2;
			npc->direct >>= 8;
			npc->count2 = 0xC0;
			npc->damage = 14;
			// Fallthrough
		case 10:
			if (npc->count2 < 320)
				npc->count2 += 8;
			else
				npc->act_no = 11;

			break;

		case 11:
			if (npc->count2 > 304)
				npc->count2 -= 4;
			else
				npc->act_no = 12;

			break;

		case 12:
			if (npc->pNpc->act_no == 311)
				npc->act_no = 20;

			break;

		case 20:
			npc->act_no = 21;
			npc->bits |= NPC_SHOOTABLE;
			npc->life = 1000;
			// Fallthrough
		case 21:
			npc->count1 -= 2;

			if (npc->count1 < 0)
				npc->count1 += 0x200;

			if (npc->shock)
			{
				if (++flash / 2 % 2)
					npc->ani_no = 1;
				else
					npc->ani_no = 0;
			}
			else
			{
				npc->ani_no = 0;
			}

			if (npc->life < 900)
			{
				npc->act_no = 22;
				npc->bits &= ~NPC_SHOOTABLE;
				SetDestroyNpChar(npc->x, npc->y, 0x2000, 0x20);
				PlaySoundObject(71, SOUND_MODE_PLAY);
			}

			npc->pNpc->count1 = 4;

			if (npc->pNpc->act_no == 401)
				npc->act_no = 23;

			break;

		case 22:
			npc->ani_no = 2;
			npc->count1 -= 2;

			if (npc->count1 < 0)
				npc->count1 += 0x200;

			if (npc->pNpc->act_no == 401)
				npc->act_no = 23;

			break;

		case 23:
			npc->ani_no = 2;
			npc->count1 -= 4;

			if (npc->count1 < 0)
				npc->count1 += 0x200;

			if (npc->pNpc->act_no == 420)
				npc->act_no = 30;

			break;

		case 30:
			npc->act_no = 31;
			npc->life = 1000;
			npc->damage = 10;

			if (npc->direct == 0)
				npc->bits |= NPC_SHOOTABLE;

			npc->ym = 0;
			// Fallthrough
		case 31:
			++npc->count1;
			npc->count1 %= 0x200;

			if (npc->count2 > 0x100)
				--npc->count2;

			if (npc->bits & NPC_SHOOTABLE)
			{
				if (npc->shock)
				{
					if (++flash / 2 % 2)
						npc->ani_no = 1;
					else
						npc->ani_no = 0;
				}
				else
				{
					npc->ani_no = 0;
				}
			}
			else
			{
				npc->ani_no = 2;
			}

			if (npc->life < 900)
				npc->act_no = 40;

			break;

		case 40:
			npc->act_no = 41;
			npc->xm = 0;
			npc->ym = 0;
			npc->ani_no = 2;
			npc->damage = 5;
			npc->bits &= ~(NPC_IGNORE_SOLIDITY | NPC_SHOOTABLE);
			SetDestroyNpChar(npc->x, npc->y, 0x2000, 0x20);
			PlaySoundObject(71, SOUND_MODE_PLAY);
			// Fallthrough
		case 41:
			if (npc->flag & 1)
				npc->xm = 0x100;

			if (npc->flag & 4)
				npc->xm = -0x100;

			if (npc->flag & 8)
			{
				if (npc->xm == 0)
				{
					if (gMC.x < npc->x)
						npc->xm = 0x100;
					else
						npc->xm = -0x100;
				}

				npc->ym = -0x800;
				PlaySoundObject(26, SOUND_MODE_PLAY);
			}

			npc->ym += 0x20;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			break;

		case 1000:
			npc->act_no = 1001;
			npc->xm = 0;
			npc->ym = 0;
			npc->ani_no = 2;
			npc->bits &= ~(NPC_IGNORE_SOLIDITY | NPC_SHOOTABLE);
			npc->damage = 0;
			npc->count1 /= 4;
			npc->exp = 0;
			// Fallthrough
		case 1001:
			if (npc->count1 > 0)
			{
				if (--npc->count1 / 2 % 2)
					npc->ani_no = 1;
				else
					npc->ani_no = 0;
			}
			else
			{
				SetDestroyNpChar(npc->x, npc->y, 0x2000, 0x20);
				PlaySoundObject(71, SOUND_MODE_PLAY);
				VanishNpChar(npc);
				return;
			}

			break;
	}

	if (npc->act_no == 21 || npc->act_no == 22)
	{
		switch (npc->pNpc->direct)
		{
			case 0:
				if (npc->count1 == 140)
				{
					SetNpChar(4, npc->x + (8 * 0x200), npc->y + 0x1800, 0, 0, 0, NULL, 0x100);
					SetNpChar(4, npc->x - (8 * 0x200), npc->y + 0x1800, 0, 0, 0, NULL, 0x100);
					PlaySoundObject(26, SOUND_MODE_PLAY);
				}
				break;

			case 1:
				if (npc->count1 == 268)
				{
					SetNpChar(4, npc->x - (12 * 0x200), npc->y + (8 * 0x200), 0, 0, 0, NULL, 0x100);
					SetNpChar(4, npc->x - (12 * 0x200), npc->y - (8 * 0x200), 0, 0, 0, NULL, 0x100);
					PlaySoundObject(26, SOUND_MODE_PLAY);
				}
				break;

			case 2:
				if (npc->count1 == 396)
				{
					SetNpChar(4, npc->x + (8 * 0x200), npc->y - (12 * 0x200), 0, 0, 0, NULL, 0x100);
					SetNpChar(4, npc->x - (8 * 0x200), npc->y - (12 * 0x200), 0, 0, 0, NULL, 0x100);
					SetNpChar(345, npc->x - (8 * 0x200), npc->y - (12 * 0x200), 0, 0, 0, NULL, 0x100);
					PlaySoundObject(26, SOUND_MODE_PLAY);
				}
				break;

			case 3:
				if (npc->count1 == 12)
				{
					SetNpChar(4, npc->x + (12 * 0x200), npc->y + (8 * 0x200), 0, 0, 0, NULL, 0x100);
					SetNpChar(4, npc->x + (12 * 0x200), npc->y - (8 * 0x200), 0, 0, 0, NULL, 0x100);
					PlaySoundObject(26, SOUND_MODE_PLAY);
				}
				break;
		}
	}

	if (npc->act_no < 40)
	{
		deg = npc->count1 / 2;

		npc->tgt_x = npc->pNpc->x + npc->count2 * GetCos(deg) / 4;
		npc->tgt_y = npc->pNpc->y + npc->count2 * GetSin(deg) / 4;

		npc->xm = npc->tgt_x - npc->x;
		npc->ym = npc->tgt_y - npc->y;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rc[npc->ani_no];
}

// Ballos 2 cutscene
void ActNpc343(NPCHAR *npc)
{
	RECT rc = {0, 0, 120, 120};

	npc->rect = rc;

	if (++npc->act_wait > 100)
		npc->cond = 0;

	npc->x = npc->pNpc->x;
	npc->y = npc->pNpc->y;
}

// Ballos 2 eyes
void ActNpc344(NPCHAR *npc)
{
	RECT rc[2] = {
		{272, 0, 296, 16},
		{296, 0, 320, 16},
	};

	if (npc->direct == 0)
	{
		npc->rect = rc[0];
		npc->x = npc->pNpc->x - (24 * 0x200);
	}
	else
	{
		npc->rect = rc[1];
		npc->x = npc->pNpc->x + (24 * 0x200);
	}

	if (++npc->act_wait > 100)
		npc->cond = 0;

	npc->y = npc->pNpc->y - (36 * 0x200);
}

// Ballos skull projectile
void ActNpc345(NPCHAR *npc)
{
	RECT rc[4] = {
		{128, 176, 144, 192},
		{144, 176, 160, 192},
		{160, 176, 176, 192},
		{176, 176, 192, 192},
	};

	int i;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 100;
			npc->ani_no = Random(0, 16) % 4;
			// Fallthrough
		case 100:
			npc->ym += 0x40;
			if (npc->ym > 0x700)
				npc->ym = 0x700;

			if (npc->y > 128 * 0x200)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			if (npc->act_wait++ / 2 % 2)
				SetNpChar(4, npc->x, npc->y, 0, 0, 0, NULL, 0x100);

			if (npc->flag & 8)
			{
				npc->ym = -0x200;
				npc->act_no = 110;
				npc->bits |= NPC_IGNORE_SOLIDITY;
				PlaySoundObject(12, SOUND_MODE_PLAY);
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

	if (++npc->ani_wait > 8)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 3)
		npc->ani_no = 0;

	npc->y += npc->ym;

	npc->rect = rc[npc->ani_no];
}

// Ballos 4 orbiting platform
void ActNpc346(NPCHAR *npc)
{
	RECT rc = {240, 0, 272, 16};
	unsigned char deg;

	if (npc->act_no < 1000 && npc->pNpc->act_no >= 1000)
		npc->act_no = 1000;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->count1 = npc->direct * 4;
			npc->count2 = 192;
			npc->ani_no = 0;
			// Fallthrough
		case 10:
			if (npc->count2 < 448)
				npc->count2 += 8;
			else
				npc->act_no = 11;

			break;

		case 11:
			if (npc->pNpc->act_no == 411)
				npc->act_no = 20;

			break;

		case 20:
			if (--npc->count1 < 0)
				npc->count1 += 0x400;

			if (npc->pNpc->act_no == 421)
				npc->act_no = 40;
			if (npc->pNpc->act_no == 423)
				npc->act_no = 100;

			break;

		case 30:
			++npc->count1;
			npc->count1 %= 0x400;

			if (npc->pNpc->act_no == 425)
				npc->act_no = 50;
			if (npc->pNpc->act_no == 427)
				npc->act_no = 100;

			break;

		case 40:
			npc->count1 -= 2;

			if (npc->count1 < 0)
				npc->count1 += 0x400;

			if (npc->pNpc->act_no == 422)
				npc->act_no = 20;

			break;

		case 50:
			npc->count1 += 2;
			npc->count1 %= 0x400;

			if (npc->pNpc->act_no == 426)
				npc->act_no = 30;

			break;

		case 100:
			npc->ani_no = 0;

			if (npc->pNpc->act_no == 424)
				npc->act_no = 30;
			if (npc->pNpc->act_no == 428)
				npc->act_no = 20;

			break;

		case 1000:
			npc->act_no = 1001;
			npc->xm = 0;
			npc->ym = 0;
			npc->bits &= ~NPC_SOLID_HARD;
			// Fallthrough
		case 1001:
			npc->ym += 0x40;

			if (npc->y > gMap.length * 0x10 * 0x200)
				npc->cond = 0;

			break;
	}

	if (npc->act_no < 1000)
	{
		if (gMC.y > npc->y - 0x1000 && gMC.ym < 0)
			npc->bits &= ~NPC_SOLID_HARD;
		else
			npc->bits |= NPC_SOLID_HARD;

		deg = npc->count1 / 4;
		npc->tgt_x = npc->pNpc->x + npc->count2 * GetCos(deg) / 4;
		npc->tgt_y = npc->pNpc->y + (16 * 0x200) + npc->count2 * GetSin(deg) / 4;

		npc->xm = npc->tgt_x - npc->x;

		if (npc->act_no == 20 || npc->act_no == 30)
		{
			if (npc->count1 % 4 == 0)
				npc->ani_no = (npc->tgt_y - npc->y) / 4;
		}
		else if (npc->act_no == 40 || npc->act_no == 50)
		{
			 if ((npc->count1 / 2 % 2) == 0)
				npc->ani_no = (npc->tgt_y - npc->y) / 2;
		}
		else
		{
			npc->ani_no = npc->tgt_y - npc->y;
		}

		npc->ym = npc->ani_no;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	npc->rect = rc;
}

// Hoppy
void ActNpc347(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			npc->ani_no = 0;

			if (gMC.y < npc->y + (128 * 0x200) && gMC.y > npc->y - (128 * 0x200))
			{
				npc->act_no = 10;
				npc->act_wait = 0;
				npc->ani_no = 1;
			}

			break;

		case 10:
			if (++npc->act_wait == 4)
				npc->ani_no = 2;

			if (npc->act_wait > 12)
			{
				npc->act_no = 12;
				npc->xm = 0x700;
				PlaySoundObject(6, SOUND_MODE_PLAY);
				npc->ani_no = 3;
			}

			break;

		case 12:
			if (gMC.y < npc->y)
				npc->ym = -0xAA;
			else
				npc->ym = 0xAA;

			if (npc->flag & 1)
			{
				npc->act_no = 13;
				npc->act_wait = 0;
				npc->ani_no = 2;
				npc->xm = 0;
				npc->ym = 0;
				break;
			}

			npc->xm -= 42;

			if (npc->xm < -0x5FF)
				npc->xm = -0x5FF;

			npc->x += npc->xm;
			npc->y += npc->ym;
			break;

		case 13:
			++npc->act_wait;

			if (npc->act_wait == 2)
				npc->ani_no = 1;

			if (npc->act_wait == 6)
				npc->ani_no = 0;

			if (npc->act_wait > 16)
				npc->act_no = 1;

			break;
	}

	RECT rc[4] = {
		{256, 48, 272, 64},
		{272, 48, 288, 64},
		{288, 48, 304, 64},
		{304, 48, 320, 64},
	};

	npc->rect = rc[npc->ani_no];
}

// Ballos 4 spikes
void ActNpc348(NPCHAR *npc)
{
	RECT rc[2] = {
		{128, 152, 160, 176},
		{160, 152, 192, 176},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			// Fallthrough
		case 1:
			if (++npc->act_wait < 0x80)
			{
				npc->y -= 0x80;

				if (npc->act_wait / 2 % 2)
					npc->ani_no = 1;
				else
					npc->ani_no = 0;
			}
			else
			{
				npc->act_no = 10;
				npc->ani_no = 0;
				npc->damage = 2;
			}

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Statue
void ActNpc349(NPCHAR *npc)
{
	RECT rect = {0, 0, 16, 16};

	if (npc->act_no == 0)
	{
		npc->act_no = 1;

		if (npc->direct == 0)
			npc->x += 8 * 0x200;
		if (npc->direct == 2)
			npc->y += 16 * 0x200;
	}

	npc->rect = rect;
}

// Flying Bute archer
void ActNpc350(NPCHAR *npc)
{
	RECT rcLeft[7] = {
		{0, 160, 24, 184},
		{24, 160, 48, 184},
		{48, 160, 72, 184},
		{72, 160, 96, 184},
		{96, 160, 120, 184},
		{120, 160, 144, 184},
		{144, 160, 168, 184},
	};

	RECT rcRight[7] = {
		{0, 184, 24, 208},
		{24, 184, 48, 208},
		{48, 184, 72, 208},
		{72, 184, 96, 208},
		{96, 184, 120, 208},
		{120, 184, 144, 208},
		{144, 184, 168, 208},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			if (npc->direct == 0)
				npc->tgt_x = npc->x - (128 * 0x200);
			else
				npc->tgt_x = npc->x + (128 * 0x200);

			npc->tgt_y = npc->y;
			npc->ym = Random(-0x200, 0x200) * 2;
			npc->xm = Random(-0x200, 0x200) * 2;
			// Fallthrough
		case 1:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 1)
					npc->ani_no = 0;
			}

			if (npc->direct == 0)
			{
				if (npc->x < npc->tgt_x)
					npc->act_no = 20;
			}
			else
			{
				if (npc->x > npc->tgt_x)
					npc->act_no = 20;
			}

			break;

		case 20:
			npc->act_no = 21;
			npc->act_wait = Random(0, 150);
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 21:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 3)
					npc->ani_no = 2;
			}

			if (++npc->act_wait > 300)
				npc->act_no = 30;

			if (gMC.x < npc->x + (112 * 0x200) && gMC.x > npc->x - (112 * 0x200) && gMC.y < npc->y + (16 * 0x200) && gMC.y > npc->y - (16 * 0x200))
				npc->act_no = 30;

			break;

		case 30:
			npc->act_no = 31;
			npc->act_wait = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 31:
			if (++npc->ani_wait / 2 % 2)
				npc->ani_no = 3;
			else
				npc->ani_no = 4;

			if (++npc->act_wait > 30)
			{
				npc->act_no = 40;
				npc->ani_no = 5;

				if (npc->direct == 0)
					SetNpChar(312, npc->x, npc->y, -0x800, 0, 0, NULL, 0x199);
				else
					SetNpChar(312, npc->x, npc->y, 0x800, 0, 2, NULL, 0x199);
			}

			break;

		case 40:
			npc->act_no = 41;
			npc->act_wait = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 41:
			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 6)
					npc->ani_no = 5;
			}

			if (++npc->act_wait > 40)
			{
				npc->act_no = 50;
				npc->ani_no = 0;
				npc->xm = 0;
				npc->ym = 0;
			}

			break;

		case 50:
			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 1)
					npc->ani_no = 0;
			}

			if (npc->direct == 0)
				npc->xm -= 0x20;
			else
				npc->xm += 0x20;

			if (npc->x < 0 || npc->x > gMap.width * 0x200 * 0x10)
			{
				VanishNpChar(npc);
				return;
			}

			break;
	}

	if (npc->act_no < 50)
	{
		if (npc->x < npc->tgt_x)
			npc->xm += 0x2A;
		if (npc->x > npc->tgt_x)
			npc->xm -= 0x2A;

		if (npc->y < npc->tgt_y)
			npc->ym += 0x2A;
		if (npc->y > npc->tgt_y)
			npc->ym -= 0x2A;

		if (npc->xm > 0x400)
			npc->xm = 0x400;
		if (npc->xm < -0x400)
			npc->xm = -0x400;

		if (npc->ym > 0x400)
			npc->ym = 0x400;
		if (npc->ym < -0x400)
			npc->ym = -0x400;
	}

	npc->x += npc->xm;
	npc->y += npc->ym;

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

// Statue (shootable)
void ActNpc351(NPCHAR *npc)
{
	RECT rc[9] = {
		{0, 96, 32, 136},
		{32, 96, 64, 136},
		{64, 96, 96, 136},
		{96, 96, 128, 136},
		{128, 96, 160, 136},
		{0, 176, 32, 216},
		{32, 176, 64, 216},
		{64, 176, 96, 216},
		{96, 176, 128, 216},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;
			npc->ani_no = npc->direct / 10;
			npc->x += 8 * 0x200;
			npc->y += 12 * 0x200;
			break;

		case 10:
			if (GetNPCFlag(npc->code_flag))
			{
				npc->act_no = 20;
			}
			else
			{
				npc->act_no = 11;
				npc->bits |= NPC_SHOOTABLE;
			}
			// Fallthrough
		case 11:
			if (npc->life <= 900)
			{
				SetNpChar(351, npc->x - (8 * 0x200), npc->y - (12 * 0x200), 0, 0, (npc->ani_no + 4) * 10, NULL, 0);
				npc->cond |= 8;
			}

			break;

		case 20:
			npc->ani_no += 4;
			npc->act_no = 1;
			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Ending characters
void ActNpc352(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 0:
			// Set state
			npc->act_no = 1;
			npc->ani_no = 0;
			npc->count1 = npc->direct / 100;
			npc->direct %= 100;

			// Set surfaces / offset
			switch (npc->count1)
			{
				case 7:
				case 8:
				case 9:
				case 12:
				case 13:
					npc->surf = SURFACE_ID_LEVEL_SPRITESET_1;
					break;
			}

			switch (npc->count1)
			{
				case 2:
				case 4:
				case 9:
				case 12:
					npc->view.top = 16 * 0x200;
					break;
			}

			// Balrog
			if (npc->count1 == 9)
			{
				npc->view.back = 20 * 0x200;
				npc->view.front = 20 * 0x200;
				npc->x -= 1 * 0x200;
			}

			// Spawn King's sword
			if (npc->count1 == 0)
				SetNpChar(145, 0, 0, 0, 0, 2, npc, 0x100);
			// Fallthrough
		case 1:
			npc->ym += 0x40;
			if (npc->ym > 0x5FF)
				npc->ym = 0x5FF;

			if (npc->flag & 8)
			{
				npc->ym = 0;
				npc->act_no = 2;
				npc->ani_no = 1;
			}

			npc->y += npc->ym;

			break;
	}

	RECT rc[28] = {
		{304, 48, 320, 64},
		{224, 48, 240, 64},
		{32, 80, 48, 96},
		{0, 80, 16, 96},
		{224, 216, 240, 240},
		{192, 216, 208, 240},
		{48, 16, 64, 32},
		{0, 16, 16, 32},
		{112, 192, 128, 216},
		{80, 192, 96, 216},
		{304, 0, 320, 16},
		{224, 0, 240, 16},
		{176, 32, 192, 48},
		{176, 32, 192, 48},
		{240, 16, 256, 32},
		{224, 16, 240, 32},
		{208, 16, 224, 32},
		{192, 16, 208, 32},
		{280, 128, 320, 152},
		{280, 152, 320, 176},
		{32, 112, 48, 128},
		{0, 112, 16, 128},
		{80, 0, 96, 16},
		{112, 0, 128, 16},
		{16, 152, 32, 176},
		{0, 152, 16, 176},
		{48, 16, 64, 32},
		{48, 0, 64, 16}
	};

	npc->rect = rc[npc->ani_no + (npc->count1 * 2)];
}

// Bute with sword (flying)
void ActNpc353(NPCHAR *npc)
{
	RECT rc[4] = {
		{168, 160, 184, 184},
		{184, 160, 200, 184},
		{168, 184, 184, 208},
		{184, 184, 200, 208},
	};

	RECT rcLeft[2] = {
		{200, 160, 216, 176},
		{216, 160, 232, 176},
	};

	RECT rcRight[2] = {
		{200, 176, 216, 192},
		{216, 176, 232, 192},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 1;

			switch (npc->direct)
			{
				case 0:
					npc->xm = -0x600;
					break;

				case 2:
					npc->xm = 0x600;
					break;

				case 1:
					npc->ym = -0x600;
					break;

				case 3:
					npc->ym = 0x600;
					break;
			}
			// Fallthrough
		case 1:
			++npc->act_wait;

			if (npc->act_wait == 8)
				npc->bits &= ~NPC_IGNORE_SOLIDITY;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->act_wait == 0x10)
				npc->act_no = 10;

			if (++npc->ani_wait > 2)
			{
				npc->ani_wait = 0;

				if (++npc->ani_no > 3)
					npc->ani_no = 0;
			}

			npc->rect = rc[npc->ani_no];
			break;

		case 10:
			npc->act_no = 11;
			npc->ani_no = 0;
			npc->bits |= NPC_SHOOTABLE;
			npc->bits &= ~NPC_IGNORE_SOLIDITY;
			npc->damage = 5;
			npc->view.top = 8 * 0x200;
			// Fallthrough
		case 11:
			if (npc->x > gMC.x)
				npc->direct = 0;
			else
				npc->direct = 2;

			if (gMC.y - (24 * 0x200) > npc->y)
			{
				if (npc->direct == 0)
					npc->xm2 += 0x10;
				else
					npc->xm2 -= 0x10;
			}
			else
			{
				if (npc->direct == 0)
					npc->xm2 -= 0x10;
				else
					npc->xm2 += 0x10;
			}

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

				if (++npc->ani_no > 1)
					npc->ani_no = 0;
			}

			if (npc->direct == 0)
				npc->rect = rcLeft[npc->ani_no];
			else
				npc->rect = rcRight[npc->ani_no];

			break;
	}
}

// Invisible deathtrap wall
void ActNpc354(NPCHAR *npc)
{
	int i, x, y;

	switch (npc->act_no)
	{
		case 0:
			npc->hit.bottom = 280 * 0x200;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;

			if (npc->direct == 0)
				npc->x += 16 * 0x200;
			else
				npc->x -= 16 * 0x200;
			// Fallthrough
		case 11:
			if (++npc->act_wait > 100)
			{
				npc->act_wait = 0;
				SetQuake(20);
				PlaySoundObject(26, SOUND_MODE_PLAY);
				PlaySoundObject(12, SOUND_MODE_PLAY);

				if (npc->direct == 0)
					npc->x -= 16 * 0x200;
				else
					npc->x += 16 * 0x200;

				for (i = 0; i < 20; ++i)
				{
					x = (npc->x / 0x200 / 0x10);
					y = (npc->y / 0x200 / 0x10) + i;
					ChangeMapParts(x, y, 109);
				}
			}

			break;
	}
}

// Quote and Curly on Balrog's back
void ActNpc355(NPCHAR *npc)
{
	RECT rc[4] = {
		{80, 16, 96, 32},
		{80, 96, 96, 112},
		{128, 16, 144, 32},
		{208, 96, 224, 112},
	};

	switch (npc->act_no)
	{
		case 0:
			switch (npc->direct)
			{
				case 0:
					npc->surf = SURFACE_ID_MY_CHAR;
					npc->ani_no = 0;
					npc->x = npc->pNpc->x - (14 * 0x200);
					npc->y = npc->pNpc->y + (10 * 0x200);
					break;

				case 1:
					npc->surf = SURFACE_ID_NPC_REGU;
					npc->ani_no = 1;
					npc->x = npc->pNpc->x + (14 * 0x200);
					npc->y = npc->pNpc->y + (10 * 0x200);
					break;

				case 2:
					npc->surf = SURFACE_ID_MY_CHAR;
					npc->ani_no = 2;
					npc->x = npc->pNpc->x - (7 * 0x200);
					npc->y = npc->pNpc->y - (19 * 0x200);
					break;

				case 3:
					npc->surf = SURFACE_ID_NPC_REGU;
					npc->ani_no = 3;
					npc->x = npc->pNpc->x + (4 * 0x200);
					npc->y = npc->pNpc->y - (19 * 0x200);
					break;
			}

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Balrog rescue
void ActNpc356(NPCHAR *npc)
{
	RECT rcRight[2] = {
		{240, 128, 280, 152},
		{240, 152, 280, 176},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 11;
			npc->ani_wait = 0;
			npc->tgt_y = npc->y - (16 * 0x200);
			npc->tgt_x = npc->x - (6 * 0x200);
			npc->ym = 0;
			SetNpChar(355, 0, 0, 0, 0, 3, npc, 0xAA);
			SetNpChar(355, 0, 0, 0, 0, 2, npc, 0xAA);
			// Fallthrough
		case 11:
			if (npc->x < npc->tgt_x)
				npc->xm += 8;
			else
				npc->xm -= 8;

			if (npc->y < npc->tgt_y)
				npc->ym += 8;
			else
				npc->ym -= 8;

			npc->x += npc->xm;
			npc->y += npc->ym;

			break;

		case 20:
			npc->act_no = 21;
			npc->xm = -0x400;
			npc->ym = 0x200;
			// Fallthrough
		case 21:
			++npc->ani_wait;
			npc->xm += 0x10;
			npc->ym -= 8;
			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->x > 60 * 0x10 * 0x200)
				npc->act_no = 22;

			break;

		case 22:
			npc->xm = 0;
			npc->ym = 0;
			break;
	}

	if (++npc->ani_wait > 4)
	{
		npc->ani_wait = 0;
		++npc->ani_no;
	}

	if (npc->ani_no > 1)
		npc->ani_no = 0;

	npc->rect = rcRight[npc->ani_no];
}

// Puppy ghost
void ActNpc357(NPCHAR *npc)
{
	RECT rc = {224, 136, 240, 152};

	switch (npc->act_no)
	{
		case 0:
			npc->rect = rc;
			++npc->act_wait;
			break;

		case 10:
			npc->act_wait = 0;
			npc->act_no = 11;
			PlaySoundObject(29, SOUND_MODE_PLAY);
			// Fallthrough
		case 11:
			++npc->act_wait;
			npc->rect = rc;

			if (npc->act_wait / 2 % 2)
				npc->rect.right = npc->rect.left;

			if (npc->act_wait > 50)
				npc->cond = 0;

			break;
	}

	if (npc->act_wait % 8 == 1)
		SetCaret(npc->x + (Random(-8, 8) * 0x200), npc->y + 0x1000, CARET_TINY_PARTICLES, DIR_UP);
}

// Misery (stood in the wind during the credits)
void ActNpc358(NPCHAR *npc)
{
	RECT rc[5] = {
		{208, 8, 224, 32},
		{224, 8, 240, 32},
		{240, 8, 256, 32},
		{256, 8, 272, 32},
		{272, 8, 288, 32},
	};

	switch (npc->act_no)
	{
		case 0:
			if (++npc->ani_wait > 6)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			break;

		case 10:
			if (++npc->ani_wait > 6)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 4)
				npc->ani_no = 3;

			break;
	}

	npc->rect = rc[npc->ani_no];
}

// Water droplet generator
void ActNpc359(NPCHAR *npc)
{
	int x;

	if (gMC.x < npc->x + (((WINDOW_WIDTH / 2) + 160) * 0x200) && gMC.x > npc->x - (((WINDOW_WIDTH / 2) + 160) * 0x200) && gMC.y < npc->y + (((WINDOW_HEIGHT / 2) + 200) * 0x200) && gMC.y > npc->y - (((WINDOW_HEIGHT / 2) + 40) * 0x200) && Random(0, 100) == 2)
	{
		x = npc->x + (Random(-6, 6) * 0x200);
		SetNpChar(73, x, npc->y - (7 * 0x200), 0, 0, 0, 0, 0);
	}
}

// "Thank you" message at the end of the credits
void ActNpc360(NPCHAR *npc)
{
	RECT rc = {0, 176, 48, 184};

	if (npc->act_no == 0)
	{
		++npc->act_no;
		npc->x -= 8 * 0x200;
		npc->y -= 8 * 0x200;
	}

	npc->rect = rc;
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossX.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Flash.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

static void ActBossChar03_01(NPCHAR *npc)
{
	RECT rcUp[6] = {
		{0, 0, 72, 32},
		{0, 32, 72, 64},
		{72, 0, 144, 32},
		{144, 0, 216, 32},
		{72, 32, 144, 64},
		{144, 32, 216, 64},
	};

	RECT rcDown[6] = {
		{0, 64, 72, 96},
		{0, 96, 72, 128},
		{72, 64, 144, 96},
		{144, 64, 216, 96},
		{72, 96, 144, 128},
		{144, 96, 216, 128},
	};

	switch (npc->act_no)
	{
		case 10:
			npc->ani_no = 0;
			npc->bits &= ~NPC_BOUNCY;
			break;

		case 100:
			npc->bits |= NPC_BOUNCY;
			npc->act_no = 101;
			npc->act_wait = 0;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			// Fallthrough
		case 101:
			if (++npc->act_wait > 30)
				npc->act_no = 102;

			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			npc->xm -= 0x20;
			break;

		case 102:
			npc->bits &= ~NPC_BOUNCY;
			npc->act_no = 103;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 103:
			++npc->act_wait;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			npc->xm -= 0x20;
			break;

		case 200:
			npc->bits |= NPC_BOUNCY;
			npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
			npc->act_no = 201;
			npc->act_wait = 0;
			npc->ani_no = 4;
			npc->ani_wait = 0;
			// Fallthrough
		case 201:
			if (++npc->act_wait > 30)
				npc->act_no = 202;

			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 4;

			npc->xm += 0x20;
			break;

		case 202:
			npc->bits &= ~NPC_BOUNCY;
			npc->act_no = 203;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 203:
			++npc->act_wait;

			if (++npc->ani_wait > 1)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 1)
				npc->ani_no = 0;

			npc->xm += 0x20;
			break;

		case 300:
			npc->act_no = 301;
			npc->ani_no = 4;
			npc->ani_wait = 0;
			npc->bits |= NPC_BOUNCY;
			// Fallthrough
		case 301:
			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 5)
				npc->ani_no = 4;

			npc->xm += 0x20;

			if (npc->xm > 0)
			{
				npc->xm = 0;
				npc->act_no = 10;
			}

			break;

		case 400:
			npc->act_no = 401;
			npc->ani_no = 2;
			npc->ani_wait = 0;
			npc->bits |= NPC_BOUNCY;
			// Fallthrough
		case 401:
			if (++npc->ani_wait > 0)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 3)
				npc->ani_no = 2;

			npc->xm -= 0x20;

			if (npc->xm < 0)
			{
				npc->xm = 0;
				npc->act_no = 10;
			}

			break;
	}

	if ((npc->act_no == 101 || npc->act_no == 201 || npc->act_no == 301 || npc->act_no == 401) && npc->act_wait % 2 == 1)
		PlaySoundObject(112, SOUND_MODE_PLAY);

	if ((npc->act_no == 103 || npc->act_no == 203) && npc->act_wait % 4 == 1)
		PlaySoundObject(111, SOUND_MODE_PLAY);

	if (npc->act_no >= 100 && gMC.y < npc->y + (4 * 0x200) && gMC.y > npc->y - (4 * 0x200))
	{
		npc->damage = 10;
		npc->bits |= NPC_REAR_AND_TOP_DONT_HURT;
	}
	else
	{
		npc->damage = 0;
		npc->bits &= ~NPC_REAR_AND_TOP_DONT_HURT;
	}

	if (npc->xm > 0x400)
		npc->xm = 0x400;
	if (npc->xm < -0x400)
		npc->xm = -0x400;

	npc->x += npc->xm;

	if (npc->direct == 1)
		npc->rect = rcUp[npc->ani_no];
	else
		npc->rect = rcDown[npc->ani_no];
}

static void ActBossChar03_02(NPCHAR *npc)
{
	RECT rect[4] = {
		{0, 128, 72, 160},
		{72, 128, 144, 160},
		{0, 160, 72, 192},
		{72, 160, 144, 192},
	};

	int direct;
	int x, y;

	switch (npc->act_no)
	{
		case 10:
			npc->act_no = 11;
			npc->act_wait = (npc->ani_no * 30) + 30;
			// Fallthrough
		case 11:
			if (npc->act_wait != 0)
			{
				--npc->act_wait;
				break;
			}

			switch (npc->ani_no)
			{
				case 0:
					direct = 3;
					x = -30 * 0x200;
					y = 6 * 0x200;
					break;
				case 1:
					direct = 2;
					x = 30 * 0x200;
					y = 6 * 0x200;
					break;
				case 2:
					direct = 0;
					x = -30 * 0x200;
					y = -6 * 0x200;
					break;
				case 3:
					direct = 1;
					x = 30 * 0x200;
					y = -6 * 0x200;
					break;
			}

			SetNpChar(158, npc->x + x, npc->y + y, 0, 0, direct, NULL, 0x100);
			PlaySoundObject(39, SOUND_MODE_PLAY);
			npc->act_wait = 120;

			break;
	}

	npc->x = (gBoss[0].x + gBoss[npc->count1].x) / 2;
	npc->y = (gBoss[0].y + gBoss[npc->count1].y) / 2;

	npc->rect = rect[npc->ani_no];
}

static void ActBossChar03_03(NPCHAR *npc)
{
	switch (npc->act_no)
	{
		case 10:
			npc->tgt_x += 1 * 0x200;

			if (npc->tgt_x > 32 * 0x200)
			{
				npc->tgt_x = 32 * 0x200;
				npc->act_no = 0;
				gBoss[3].act_no = 10;
				gBoss[4].act_no = 10;
				gBoss[5].act_no = 10;
				gBoss[6].act_no = 10;
			}

			break;

		case 20:
			npc->tgt_x -= 1 * 0x200;

			if (npc->tgt_x < 0)
			{
				npc->tgt_x = 0;
				npc->act_no = 0;
				gBoss[3].act_no = 0;
				gBoss[4].act_no = 0;
				gBoss[5].act_no = 0;
				gBoss[6].act_no = 0;
			}

			break;

		case 30:
			npc->tgt_x += 1 * 0x200;

			if (npc->tgt_x > 20 * 0x200)
			{
				npc->tgt_x = 20 * 0x200;
				npc->act_no = 0;
				gBoss[7].act_no = 10;
				gBoss[13].act_no = 10;
				gBoss[14].act_no = 10;
				gBoss[15].act_no = 10;
				gBoss[16].act_no = 10;
			}

			break;

		case 40:
			npc->tgt_x -= 1 * 0x200;

			if (npc->tgt_x < 0)
			{
				npc->tgt_x = 0;
				npc->act_no = 0;
				gBoss[7].act_no = 0;
				gBoss[13].act_no = 0;
				gBoss[14].act_no = 0;
				gBoss[15].act_no = 0;
				gBoss[16].act_no = 0;
			}

			break;
	}

	RECT rcLeft = {216, 96, 264, 144};
	RECT rcRight = {264, 96, 312, 144};

	if (npc->direct == 0)
	{
		npc->rect = rcLeft;
		npc->x = gBoss[0].x - (24 * 0x200) - npc->tgt_x;
		npc->y = gBoss[0].y;
	}
	else
	{
		npc->rect = rcRight;
		npc->x = gBoss[0].x + (24 * 0x200) + npc->tgt_x;
		npc->y = gBoss[0].y;
	}
}

static void ActBossChar03_04(NPCHAR *npc)
{
	int xm, ym;
	unsigned char deg;

	RECT rect[8] = {
		{0, 192, 16, 208},
		{16, 192, 32, 208},
		{32, 192, 48, 208},
		{48, 192, 64, 208},
		{0, 208, 16, 224},
		{16, 208, 32, 224},
		{32, 208, 48, 224},
		{48, 208, 64, 224},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->ani_no = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = (npc->tgt_x * 10) + 40;
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 11:
			if (npc->act_wait < 16 && npc->act_wait / 2 % 2)
				npc->ani_no = 1;
			else
				npc->ani_no = 0;

			if (npc->act_wait != 0)
			{
				--npc->act_wait;
				break;
			}

			deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
			deg += (unsigned char)Random(-2, 2);
			ym = GetSin(deg) * 3;
			xm = GetCos(deg) * 3;
			SetNpChar(156, npc->x, npc->y, xm, ym, 0, NULL, 0x100);

			PlaySoundObject(39, SOUND_MODE_PLAY);
			npc->act_wait = 40;

			break;
	}

	switch (npc->tgt_x)
	{
		case 0:
			npc->x = gBoss[0].x - (22 * 0x200);
			npc->y = gBoss[0].y - (16 * 0x200);
			break;
		case 1:
			npc->x = gBoss[0].x + (28 * 0x200);
			npc->y = gBoss[0].y - (16 * 0x200);
			break;
		case 2:
			npc->x = gBoss[0].x - (15 * 0x200);
			npc->y = gBoss[0].y + (14 * 0x200);
			break;
		case 3:
			npc->x = gBoss[0].x + (17 * 0x200);
			npc->y = gBoss[0].y + (14 * 0x200);
			break;
	}

	npc->rect = rect[npc->tgt_x + 4 * npc->ani_no];
}

static void ActBossChar03_face(NPCHAR *npc)
{
	RECT rect[3] = {
		{216, 0, 320, 48},
		{216, 48, 320, 96},
		{216, 144, 320, 192},
	};

	static unsigned char flash;

	switch (npc->act_no)
	{
		case 0:
			gBoss[0].bits &= ~NPC_SHOOTABLE;
			npc->ani_no = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = (npc->tgt_x * 10) + 40;
			gBoss[0].bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 11:
			if (gBoss[0].shock)
			{
				if (flash++ / 2 % 2)
					npc->ani_no = 1;
				else
					npc->ani_no = 0;
			}
			else
			{
				npc->ani_no = 0;
			}

			break;
	}

	gBoss[7].x = gBoss[0].x;
	gBoss[7].y = gBoss[0].y;

	if (gBoss[0].act_no <= 10)
		npc->ani_no = 2;

	npc->rect = rect[npc->ani_no];
}

void ActBossChar_MonstX(void)
{
	int i;
	NPCHAR *npc = gBoss;

	switch (npc->act_no)
	{
		case 0:
			npc->life = 1;
			npc->x = -320 * 0x200;
			break;

		case 1:
			npc->life = 700;
			npc->exp = 1;
			npc->act_no = 1;
			npc->x = 2048 * 0x200;
			npc->y = 200 * 0x200;
			npc->hit_voice = 54;
			npc->hit.front = 24 * 0x200;
			npc->hit.top = 24 * 0x200;
			npc->hit.back = 24 * 0x200;
			npc->hit.bottom = 24 * 0x200;
			npc->bits = (NPC_IGNORE_SOLIDITY | NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			npc->size = 3;
			npc->code_event = 1000;
			npc->ani_no = 0;

			gBoss[1].cond = 0x80;
			gBoss[1].size = 3;
			gBoss[1].direct = 0;
			gBoss[1].view.front = 24 * 0x200;
			gBoss[1].view.top = 24 * 0x200;
			gBoss[1].view.back = 24 * 0x200;
			gBoss[1].view.bottom = 24 * 0x200;
			gBoss[1].bits = NPC_IGNORE_SOLIDITY;

			gBoss[2] = gBoss[1];
			gBoss[2].direct = 2;

			gBoss[3].cond = 0x80;
			gBoss[3].life = 60;
			gBoss[3].size = 3;	// Redundant: this get overwritten a few lines later
			gBoss[3].hit_voice = 54;
			gBoss[3].destroy_voice = 71;
			gBoss[3].size = 2;
			gBoss[3].view.front = 8 * 0x200;
			gBoss[3].view.top = 8 * 0x200;
			gBoss[3].view.back = 8 * 0x200;
			gBoss[3].view.bottom = 8 * 0x200;
			gBoss[3].hit.front = 5 * 0x200;
			gBoss[3].hit.back = 5 * 0x200;
			gBoss[3].hit.top = 5 * 0x200;
			gBoss[3].hit.bottom = 5 * 0x200;
			gBoss[3].bits = NPC_IGNORE_SOLIDITY;
			gBoss[3].tgt_x = 0;

			gBoss[4] = gBoss[3];
			gBoss[4].tgt_x = 1;

			gBoss[5] = gBoss[3];
			gBoss[5].tgt_x = 2;
			gBoss[5].life = 100;

			gBoss[6] = gBoss[3];
			gBoss[6].tgt_x = 3;
			gBoss[6].life = 100;

			gBoss[7].cond = 0x80;
			gBoss[7].x = 2048 * 0x200;
			gBoss[7].y = 200 * 0x200;
			gBoss[7].view.front = 52 * 0x200;
			gBoss[7].view.top = 24 * 0x200;
			gBoss[7].view.back = 52 * 0x200;
			gBoss[7].view.bottom = 24 * 0x200;
			gBoss[7].hit_voice = 52;
			gBoss[7].hit.front = 8 * 0x200;
			gBoss[7].hit.top = 24 * 0x200;
			gBoss[7].hit.back = 8 * 0x200;
			gBoss[7].hit.bottom = 16 * 0x200;
			gBoss[7].bits = NPC_IGNORE_SOLIDITY;
			gBoss[7].size = 3;
			gBoss[7].ani_no = 0;

			gBoss[9].cond = 0x80;
			gBoss[9].act_no = 0;
			gBoss[9].direct = 1;
			gBoss[9].x = 1984 * 0x200;
			gBoss[9].y = 144 * 0x200;
			gBoss[9].view.front = 36 * 0x200;
			gBoss[9].view.top = 8 * 0x200;
			gBoss[9].view.back = 36 * 0x200;
			gBoss[9].view.bottom = 24 * 0x200;
			gBoss[9].hit_voice = 52;
			gBoss[9].hit.front = 28 * 0x200;
			gBoss[9].hit.top = 8 * 0x200;
			gBoss[9].hit.back = 28 * 0x200;
			gBoss[9].hit.bottom = 16 * 0x200;
			gBoss[9].bits = (NPC_SOLID_SOFT | NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_REAR_AND_TOP_DONT_HURT);
			gBoss[9].size = 3;

			gBoss[10] = gBoss[9];
			gBoss[10].x = 2112 * 0x200;

			gBoss[11] = gBoss[9];
			gBoss[11].direct = 3;
			gBoss[11].x = 1984 * 0x200;
			gBoss[11].y = 256 * 0x200;
			gBoss[11].view.top = 24 * 0x200;
			gBoss[11].view.bottom = 8 * 0x200;
			gBoss[11].hit.top = 16 * 0x200;
			gBoss[11].hit.bottom = 8 * 0x200;

			gBoss[12] = gBoss[11];
			gBoss[12].x = 2112 * 0x200;

			gBoss[13] = gBoss[9];
			gBoss[13].cond = 0x80;
			gBoss[13].view.top = 16 * 0x200;
			gBoss[13].view.bottom = 16 * 0x200;
			gBoss[13].view.front = 30 * 0x200;
			gBoss[13].view.back = 42 * 0x200;
			gBoss[13].count1 = 9;
			gBoss[13].ani_no = 0;
			gBoss[13].bits = NPC_IGNORE_SOLIDITY;

			gBoss[14] = gBoss[13];
			gBoss[14].view.front = 42 * 0x200;
			gBoss[14].view.back = 30 * 0x200;
			gBoss[14].count1 = 10;
			gBoss[14].ani_no = 1;
			gBoss[14].bits = NPC_IGNORE_SOLIDITY;

			gBoss[15] = gBoss[13];
			gBoss[15].view.top = 16 * 0x200;
			gBoss[15].view.bottom = 16 * 0x200;
			gBoss[15].count1 = 11;
			gBoss[15].ani_no = 2;
			gBoss[15].bits = NPC_IGNORE_SOLIDITY;

			gBoss[16] = gBoss[15];
			gBoss[16].view.front = 42 * 0x200;
			gBoss[16].view.back = 30 * 0x200;
			gBoss[16].count1 = 12;
			gBoss[16].ani_no = 3;
			gBoss[16].bits = NPC_IGNORE_SOLIDITY;

			npc->act_no = 2;
			break;

		case 10:
			npc->act_no = 11;
			npc->act_wait = 0;
			npc->count1 = 0;
			// Fallthrough
		case 11:
			++npc->act_wait;

			if (npc->act_wait > 100)
			{
				npc->act_wait = 0;

				if (npc->x > gMC.x)
					npc->act_no = 100;
				else
					npc->act_no = 200;
			}

			break;

		case 100:
			npc->act_wait = 0;
			npc->act_no = 101;
			++npc->count1;
			// Fallthrough
		case 101:
			++npc->act_wait;

			if (npc->act_wait == 4)
				gBoss[9].act_no = 100;
			if (npc->act_wait == 8)
				gBoss[10].act_no = 100;
			if (npc->act_wait == 10)
				gBoss[11].act_no = 100;
			if (npc->act_wait == 12)
				gBoss[12].act_no = 100;

			if (npc->act_wait > 120 && npc->count1 > 2)
				npc->act_no = 300;
			if (npc->act_wait > 121 && gMC.x > npc->x)
				npc->act_no = 200;

			break;

		case 200:
			npc->act_wait = 0;
			npc->act_no = 201;
			++npc->count1;
			// Fallthrough
		case 201:
			++npc->act_wait;

			if (npc->act_wait == 4)
				gBoss[9].act_no = 200;
			if (npc->act_wait == 8)
				gBoss[10].act_no = 200;
			if (npc->act_wait == 10)
				gBoss[11].act_no = 200;
			if (npc->act_wait == 12)
				gBoss[12].act_no = 200;

			if (npc->act_wait > 120 && npc->count1 > 2)
				npc->act_no = 400;
			if (npc->act_wait > 121 && gMC.x < npc->x)
				npc->act_no = 100;

			break;

		case 300:
			npc->act_wait = 0;
			npc->act_no = 301;
			// Fallthrough
		case 301:
			++npc->act_wait;

			if (npc->act_wait == 4)
				gBoss[9].act_no = 300;
			if (npc->act_wait == 8)
				gBoss[10].act_no = 300;
			if (npc->act_wait == 10)
				gBoss[11].act_no = 300;
			if (npc->act_wait == 12)
				gBoss[12].act_no = 300;

			if (npc->act_wait > 50)
			{
				if (gBoss[3].cond == 0 && gBoss[4].cond == 0 && gBoss[5].cond == 0 && gBoss[6].cond == 0)
					npc->act_no = 600;
				else
					npc->act_no = 500;
			}

			break;

		case 400:
			npc->act_wait = 0;
			npc->act_no = 401;
			// Fallthrough
		case 401:
			++npc->act_wait;

			if (npc->act_wait == 4)
				gBoss[9].act_no = 400;
			if (npc->act_wait == 8)
				gBoss[10].act_no = 400;
			if (npc->act_wait == 10)
				gBoss[11].act_no = 400;
			if (npc->act_wait == 12)
				gBoss[12].act_no = 400;

			if (npc->act_wait > 50)
			{
				if (gBoss[3].cond == 0 && gBoss[4].cond == 0 && gBoss[5].cond == 0 && gBoss[6].cond == 0)
					npc->act_no = 600;
				else
					npc->act_no = 500;
			}

			break;

		case 500:
			npc->act_no = 501;
			npc->act_wait = 0;
			gBoss[1].act_no = 10;
			gBoss[2].act_no = 10;
			// Fallthrough
		case 501:
			++npc->act_wait;

			if (npc->act_wait > 300)
			{
				npc->act_no = 502;
				npc->act_wait = 0;
			}

			if (gBoss[3].cond == 0 && gBoss[4].cond == 0 && gBoss[5].cond == 0 && gBoss[6].cond == 0)
			{
				npc->act_no = 502;
				npc->act_wait = 0;
			}

			break;

		case 502:
			npc->act_no = 503;
			npc->act_wait = 0;
			npc->count1 = 0;
			gBoss[1].act_no = 20;
			gBoss[2].act_no = 20;
			// Fallthrough
		case 503:	// Exactly identical to case 603
			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				if (npc->x > gMC.x)
					npc->act_no = 100;
				else
					npc->act_no = 200;
			}

			break;

		case 600:
			npc->act_no = 601;
			npc->act_wait = 0;
			npc->count2 = npc->life;
			gBoss[1].act_no = 30;
			gBoss[2].act_no = 30;
			// Fallthrough
		case 601:
			++npc->act_wait;

			if (npc->life < npc->count2 - 200 || npc->act_wait > 300)
			{
				npc->act_no = 602;
				npc->act_wait = 0;
			}

			break;

		case 602:
			npc->act_no = 603;
			npc->act_wait = 0;
			npc->count1 = 0;
			gBoss[1].act_no = 40;
			gBoss[2].act_no = 40;
			// Fallthrough

		case 603:	// Exactly identical to case 503
			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				if (npc->x > gMC.x)
					npc->act_no = 100;
				else
					npc->act_no = 200;
			}

			break;

		case 1000:
			SetQuake(2);

			if (++npc->act_wait % 8 == 0)
				PlaySoundObject(52, SOUND_MODE_PLAY);

			SetDestroyNpChar(npc->x + (Random(-72, 72) * 0x200), npc->y + (Random(-64, 64) * 0x200), 1, 1);

			if (npc->act_wait > 100)
			{
				npc->act_wait = 0;
				npc->act_no = 1001;
				SetFlash(npc->x, npc->y, FLASH_MODE_EXPLOSION);
				PlaySoundObject(35, SOUND_MODE_PLAY);
			}

			break;

		case 1001:
			SetQuake(40);

			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				for (i = 0; i < 20; ++i)
					gBoss[i].cond = 0;

				DeleteNpCharCode(158, TRUE);
				SetNpChar(159, npc->x, npc->y - (24 * 0x200), 0, 0, 0, NULL, 0);
			}

			break;
	}

	ActBossChar03_01(&gBoss[9]);
	ActBossChar03_01(&gBoss[10]);
	ActBossChar03_01(&gBoss[11]);
	ActBossChar03_01(&gBoss[12]);

	npc->x += (((gBoss[11].x + gBoss[10].x + gBoss[9].x + gBoss[12].x) / 4) - npc->x) / 0x10;

	ActBossChar03_face(&gBoss[7]);

	ActBossChar03_02(&gBoss[13]);
	ActBossChar03_02(&gBoss[14]);
	ActBossChar03_02(&gBoss[15]);
	ActBossChar03_02(&gBoss[16]);

	ActBossChar03_03(&gBoss[1]);
	ActBossChar03_03(&gBoss[2]);

	if (gBoss[3].cond)
		ActBossChar03_04(&gBoss[3]);
	if (gBoss[4].cond)
		ActBossChar03_04(&gBoss[4]);
	if (gBoss[5].cond)
		ActBossChar03_04(&gBoss[5]);
	if (gBoss[6].cond)
		ActBossChar03_04(&gBoss[6]);

	if (npc->life == 0 && npc->act_no < 1000)
	{
		npc->act_no = 1000;
		npc->act_wait = 0;
		npc->shock = 150;
		gBoss[9].act_no = 300;
		gBoss[10].act_no = 300;
		gBoss[11].act_no = 300;
		gBoss[12].act_no = 300;
	}
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossAlmo2.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Flash.h"
#include "Frame.h"
#include "Game.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

static void ActBossCharA_Head(NPCHAR *npc)
{
	RECT rect[4] = {
		{0, 0, 72, 112},
		{0, 112, 72, 224},
		{160, 0, 232, 112},
		{0, 0, 0, 0},
	};

	switch (npc->act_no)
	{
		case 10:
			npc->act_no = 11;
			npc->ani_no = 2;
			npc->bits = NPC_IGNORE_SOLIDITY;
			npc->view.front = 36 * 0x200;
			npc->view.top = 56 * 0x200;
			// Fallthrough
		case 11:
			npc->x = gBoss[0].x - (36 * 0x200);
			npc->y = gBoss[0].y;
			break;

		case 50:
			npc->act_no = 51;
			npc->act_wait = 112;
			// Fallthrough
		case 51:
			--npc->act_wait;

			if (npc->act_wait == 0)
			{
				npc->act_no = 100;
				npc->ani_no = 3;
			}

			break;

		case 100:
			npc->ani_no = 3;
			break;
	}

	npc->rect = rect[npc->ani_no];

	if (npc->act_no == 51)
		npc->rect.bottom = npc->rect.top + npc->act_wait;
}

static void ActBossCharA_Tail(NPCHAR *npc)
{
	RECT rect[3] = {
		{72, 0, 160, 112},
		{72, 112, 160, 224},
		{0, 0, 0, 0},
	};

	switch (npc->act_no)
	{
		case 10:
			npc->act_no = 11;
			npc->ani_no = 0;
			npc->bits = NPC_IGNORE_SOLIDITY;
			npc->view.front = 44 * 0x200;
			npc->view.top = 56 * 0x200;
			// Fallthrough
		case 11:
			npc->x = gBoss[0].x + (44 * 0x200);
			npc->y = gBoss[0].y;
			break;

		case 50:
			npc->act_no = 51;
			npc->act_wait = 112;
			// Fallthrough
		case 51:
			--npc->act_wait;

			if (npc->act_wait == 0)
			{
				npc->act_no = 100;
				npc->ani_no = 2;
			}

			break;

		case 100:
			npc->ani_no = 2;
			break;
	}

	npc->rect = rect[npc->ani_no];

	if (npc->act_no == 51)
		npc->rect.bottom = npc->rect.top + npc->act_wait;
}

static void ActBossCharA_Face(NPCHAR *npc)
{
	RECT rect[5] = {
		{0, 0, 0, 0},
		{160, 112, 232, 152},
		{160, 152, 232, 192},
		{160, 192, 232, 232},
		{248, 160, 320, 200},
	};

	switch (npc->act_no)
	{
		case 0:
			npc->ani_no = 0;
			break;

		case 10:
			npc->ani_no = 1;
			break;

		case 20:
			npc->ani_no = 2;
			break;

		case 30:
			npc->act_no = 31;
			npc->ani_no = 3;
			npc->act_wait = 100;
			// Fallthrough
		case 31:
			++npc->act_wait;

			if (npc->act_wait > 300)
				npc->act_wait = 0;

			if (npc->act_wait > 250 && npc->act_wait % 0x10 == 1)
				PlaySoundObject(26, SOUND_MODE_PLAY);

			if (npc->act_wait > 250 && npc->act_wait % 0x10 == 7)
			{
				SetNpChar(293, npc->x, npc->y, 0, 0, 0, NULL, 0x80);
				PlaySoundObject(101, SOUND_MODE_PLAY);
			}

			if (npc->act_wait == 200)
				PlaySoundObject(116, SOUND_MODE_PLAY);

			if (npc->act_wait > 200 && npc->act_wait % 2 != 0)
				npc->ani_no = 4;
			else
				npc->ani_no = 3;

			break;
	}

	npc->view.back = 36 * 0x200;
	npc->view.front = 36 * 0x200;
	npc->view.top = 20 * 0x200;

	npc->x = gBoss[0].x - (36 * 0x200);
	npc->y = gBoss[0].y + (4 * 0x200);

	npc->bits = NPC_IGNORE_SOLIDITY;

	npc->rect = rect[npc->ani_no];
}

static void ActBossCharA_Mini(NPCHAR *npc)
{
	RECT rect[3] = {
		{256, 0, 320, 40},
		{256, 40, 320, 80},
		{256, 80, 320, 120},
	};

	int deg;

	if (npc->cond == 0)
		return;

	npc->life = 1000;

	switch (npc->act_no)
	{
		case 0:
			npc->bits &= ~NPC_SHOOTABLE;
			break;

		case 5:
			npc->ani_no = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			++npc->count2;
			npc->count2 %= 0x100;
			break;

		case 10:
			npc->ani_no = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->count2 += 2;
			npc->count2 %= 0x100;
			break;

		case 20:
			npc->ani_no = 1;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->count2 += 2;
			npc->count2 %= 0x100;
			break;

		case 30:
			npc->ani_no = 0;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->count2 += 4;
			npc->count2 %= 0x100;
			break;

		case 200:
			npc->act_no = 201;
			npc->ani_no = 2;
			npc->xm = 0;
			npc->ym = 0;
			// Fallthrough
		case 201:
			npc->xm += 0x20;

			npc->x += npc->xm;

			if (npc->x > (gMap.width * 0x200 * 0x10) + (2 * 0x200 * 0x10))
				npc->cond = 0;

			break;
	}

	if (npc->act_no < 50)
	{
		if (npc->count1 != 0)
			deg = npc->count2 + 0x80;
		else
			deg = npc->count2 + 0x180;

		npc->x = npc->pNpc->x - (8 * 0x200) + (GetCos(deg / 2) * 0x30);
		npc->y = npc->pNpc->y + (GetSin(deg / 2) * 0x50);
	}

	npc->rect = rect[npc->ani_no];
}

static void ActBossCharA_Hit(NPCHAR *npc)
{
	switch (npc->count1)
	{
		case 0:
			npc->x = gBoss[0].x;
			npc->y = gBoss[0].y - (32 * 0x200);
			break;

		case 1:
			npc->x = gBoss[0].x + (28 * 0x200);
			npc->y = gBoss[0].y;
			break;

		case 2:
			npc->x = gBoss[0].x + (4 * 0x200);
			npc->y = gBoss[0].y + (32 * 0x200);
			break;

		case 3:
			npc->x = gBoss[0].x - (28 * 0x200);
			npc->y = gBoss[0].y + (4 * 0x200);
			break;
	}
}

void ActBossChar_Undead(void)
{
	NPCHAR *npc = gBoss;

	static unsigned char flash;
	BOOL bShock = FALSE;
	int x, y;
	int i;

	static int life;

	switch (npc->act_no)
	{
		case 1:
			npc->act_no = 10;
			npc->exp = 1;
			npc->cond = 0x80;
			npc->bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_SHOW_DAMAGE);
			npc->life = 700;
			npc->hit_voice = 114;
			npc->x = 592 * 0x200;
			npc->y = 120 * 0x200;
			npc->xm = 0;
			npc->ym = 0;
			npc->code_event = 1000;
			npc->bits |= NPC_EVENT_WHEN_KILLED;

			gBoss[3].cond = 0x80;
			gBoss[3].act_no = 0;

			gBoss[4].cond = 0x80;
			gBoss[4].act_no = 10;

			gBoss[5].cond = 0x80;
			gBoss[5].act_no = 10;

			gBoss[8].cond = 0x80;
			gBoss[8].bits = NPC_IGNORE_SOLIDITY;
			gBoss[8].view.front = 0;
			gBoss[8].view.top = 0;
			gBoss[8].hit.back = 40 * 0x200;
			gBoss[8].hit.top = 16 * 0x200;
			gBoss[8].hit.bottom = 16 * 0x200;
			gBoss[8].count1 = 0;

			gBoss[9] = gBoss[8];
			gBoss[9].hit.back = 36 * 0x200;
			gBoss[9].hit.top = 24 * 0x200;
			gBoss[9].hit.bottom = 24 * 0x200;
			gBoss[9].count1 = 1;

			gBoss[10] = gBoss[8];
			gBoss[10].hit.back = 44 * 0x200;
			gBoss[10].hit.top = 8 * 0x200;
			gBoss[10].hit.bottom = 8 * 0x200;
			gBoss[10].count1 = 2;

			gBoss[11] = gBoss[8];
			gBoss[11].cond |= 0x10;
			gBoss[11].hit.back = 20 * 0x200;
			gBoss[11].hit.top = 20 * 0x200;
			gBoss[11].hit.bottom = 20 * 0x200;
			gBoss[11].count1 = 3;

			gBoss[1].cond = 0x80;
			gBoss[1].act_no = 0;
			gBoss[1].bits = (NPC_IGNORE_SOLIDITY | NPC_SHOOTABLE);
			gBoss[1].life = 1000;
			gBoss[1].hit_voice = 54;
			gBoss[1].hit.back = 24 * 0x200;
			gBoss[1].hit.top = 16 * 0x200;
			gBoss[1].hit.bottom = 16 * 0x200;
			gBoss[1].view.front = 32 * 0x200;
			gBoss[1].view.top = 20 * 0x200;
			gBoss[1].pNpc = npc;

			gBoss[2] = gBoss[1];
			gBoss[2].count2 = 0x80;

			gBoss[6] = gBoss[1];
			gBoss[6].count1 = 1;

			gBoss[7] = gBoss[1];
			gBoss[7].count1 = 1;
			gBoss[7].count2 = 0x80;

			life = npc->life;

			break;

		case 15:
			npc->act_no = 16;
			bShock = TRUE;
			npc->direct = 0;
			gBoss[3].act_no = 10;
			gBoss[4].ani_no = 0;
			break;

		case 20:
			npc->act_no = 210;
			bShock = TRUE;
			npc->direct = 0;
			gBoss[1].act_no = 5;
			gBoss[2].act_no = 5;
			gBoss[6].act_no = 5;
			gBoss[7].act_no = 5;
			break;

		case 200:
			npc->act_no = 201;
			npc->act_wait = 0;
			gBoss[3].act_no = 0;
			gBoss[4].ani_no = 2;
			gBoss[5].ani_no = 0;
			gBoss[8].bits &= ~NPC_INVULNERABLE;
			gBoss[9].bits &= ~NPC_INVULNERABLE;
			gBoss[10].bits &= ~NPC_INVULNERABLE;
			gBoss[11].bits &= ~NPC_SHOOTABLE;
			gSuperYpos = 0;
			CutNoise();
			bShock = TRUE;
			// Fallthrough
		case 201:
			++npc->act_wait;

			if (npc->direct == 2 || npc->ani_no > 0 || npc->life < 200)
			{
				if (npc->act_wait > 200)
				{
					++npc->count1;
					PlaySoundObject(115, SOUND_MODE_PLAY);

					if (npc->life < 200)
					{
						npc->act_no = 230;
					}
					else
					{
						if (npc->count1 > 2)
							npc->act_no = 220;
						else
							npc->act_no = 210;
					}
				}
			}

			break;

		case 210:
			npc->act_no = 211;
			npc->act_wait = 0;
			gBoss[3].act_no = 10;
			gBoss[8].bits |= NPC_INVULNERABLE;
			gBoss[9].bits |= NPC_INVULNERABLE;
			gBoss[10].bits |= NPC_INVULNERABLE;
			gBoss[11].bits |= NPC_SHOOTABLE;
			life = npc->life;
			bShock = TRUE;
			// Fallthrough
		case 211:
			++flash;

			if (npc->shock != 0 && flash / 2 % 2)
			{
				gBoss[4].ani_no = 1;
				gBoss[5].ani_no = 1;
			}
			else
			{
				gBoss[4].ani_no = 0;
				gBoss[5].ani_no = 0;
			}

			++npc->act_wait;

			if (npc->act_wait % 100 == 1)
			{
				gCurlyShoot_wait = Random(80, 100);
				gCurlyShoot_x = gBoss[11].x;
				gCurlyShoot_y = gBoss[11].y;
			}

			if (npc->act_wait < 300)
			{
				if (npc->act_wait % 120 == 1)
					SetNpChar(288, npc->x - (32 * 0x200), npc->y - (16 * 0x200), 0, 0, 1, NULL, 0x20);

				if (npc->act_wait % 120 == 61)
					SetNpChar(288, npc->x - (32 * 0x200), npc->y + (16 * 0x200), 0, 0, 3, NULL, 0x20);
			}

			if (npc->life < life - 50 || npc->act_wait > 400)
				npc->act_no = 200;

			break;

		case 220:
			npc->act_no = 221;
			npc->act_wait = 0;
			npc->count1 = 0;
			gSuperYpos = 1;
			gBoss[3].act_no = 20;
			gBoss[8].bits |= NPC_INVULNERABLE;
			gBoss[9].bits |= NPC_INVULNERABLE;
			gBoss[10].bits |= NPC_INVULNERABLE;
			gBoss[11].bits |= NPC_SHOOTABLE;
			SetQuake(100);
			life = npc->life;
			bShock = TRUE;
			// Fallthrough
		case 221:
			++npc->act_wait;

			if (npc->act_wait % 40 == 1)
			{
				switch (Random(0, 3))
				{
					case 0:
						x = gBoss[1].x;
						y = gBoss[1].y;
						break;
					case 1:
						x = gBoss[2].x;
						y = gBoss[2].y;
						break;
					case 2:
						x = gBoss[6].x;
						y = gBoss[6].y;
						break;
					case 3:
						x = gBoss[7].x;
						y = gBoss[7].y;
						break;
				}

				PlaySoundObject(25, SOUND_MODE_PLAY);
				SetNpChar(285, x - (16 * 0x200), y, 0, 0, 0, NULL, 0x100);
				SetNpChar(285, x - (16 * 0x200), y, 0, 0, 0x400, NULL, 0x100);
			}

			++flash;

			if (npc->shock != 0 && flash / 2 % 2)
			{
				gBoss[4].ani_no = 1;
				gBoss[5].ani_no = 1;
			}
			else
			{
				gBoss[4].ani_no = 0;
				gBoss[5].ani_no = 0;
			}

			if (npc->life < life - 150 || npc->act_wait > 400 || npc->life < 200)
				npc->act_no = 200;

			break;

		case 230:
			npc->act_no = 231;
			npc->act_wait = 0;

			gBoss[3].act_no = 30;

			gBoss[8].bits |= NPC_INVULNERABLE;
			gBoss[9].bits |= NPC_INVULNERABLE;
			gBoss[10].bits |= NPC_INVULNERABLE;
			gBoss[11].bits |= NPC_SHOOTABLE;

			PlaySoundObject(25, SOUND_MODE_PLAY);

			SetNpChar(285, gBoss[3].x - (16 * 0x200), gBoss[3].y, 0, 0, 0, NULL, 0x100);
			SetNpChar(285, gBoss[3].x - (16 * 0x200), gBoss[3].y, 0, 0, 0x400, NULL, 0x100);
			SetNpChar(285, gBoss[3].x, gBoss[3].y - (16 * 0x200), 0, 0, 0, NULL, 0x100);
			SetNpChar(285, gBoss[3].x, gBoss[3].y - (16 * 0x200), 0, 0, 0x400, NULL, 0x100);
			SetNpChar(285, gBoss[3].x, gBoss[3].y + (16 * 0x200), 0, 0, 0, NULL, 0x100);
			SetNpChar(285, gBoss[3].x, gBoss[3].y + (16 * 0x200), 0, 0, 0x400, NULL, 0x100);

			life = npc->life;
			bShock = TRUE;
			// Fallthrough
		case 231:
			++flash;

			if (npc->shock != 0 && flash / 2 % 2)
			{
				gBoss[4].ani_no = 1;
				gBoss[5].ani_no = 1;
			}
			else
			{
				gBoss[4].ani_no = 0;
				gBoss[5].ani_no = 0;
			}

			if (++npc->act_wait % 100 == 1)
			{
				gCurlyShoot_wait = Random(80, 100);
				gCurlyShoot_x = gBoss[11].x;
				gCurlyShoot_y = gBoss[11].y;
			}

			if (npc->act_wait % 120 == 1)
				SetNpChar(288, npc->x - (32 * 0x200), npc->y - (16 * 0x200), 0, 0, 1, NULL, 0x20);

			if (npc->act_wait % 120 == 61)
				SetNpChar(288, npc->x - (32 * 0x200), npc->y + (16 * 0x200), 0, 0, 3, NULL, 0x20);

			break;

		case 500:
			CutNoise();
			npc->act_no = 501;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;
			gBoss[3].act_no = 0;
			gBoss[4].ani_no = 2;
			gBoss[5].ani_no = 0;
			gBoss[1].act_no = 5;
			gBoss[2].act_no = 5;
			gBoss[6].act_no = 5;
			gBoss[7].act_no = 5;
			SetQuake(20);

			for (i = 0; i < 100; ++i)
				SetNpChar(4, npc->x + (Random(-128, 128) * 0x200), npc->y + (Random(-64, 64) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0);

			DeleteNpCharCode(282, TRUE);
			gBoss[11].bits &= ~NPC_SHOOTABLE;

			for (i = 0; i < 12; ++i)
				gBoss[i].bits &= ~NPC_INVULNERABLE;
			// Fallthrough
		case 501:
			++npc->act_wait;

			if (npc->act_wait % 0x10 != 0)
				SetNpChar(4, npc->x + (Random(-64, 64) * 0x200), npc->y + (Random(-32, 32) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0x100);

			npc->x += 0x40;
			npc->y += 0x80;

			if (npc->act_wait > 200)
			{
				npc->act_wait = 0;
				npc->act_no = 1000;
			}

			break;

		case 1000:
			SetQuake(100);

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait % 8 == 0)
				PlaySoundObject(44, SOUND_MODE_PLAY);

			SetDestroyNpChar(gBoss[0].x + (Random(-72, 72) * 0x200), gBoss[0].y + (Random(-64, 64) * 0x200), 1, 1);

			if (gBoss[0].act_wait > 100)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 1001;
				SetFlash(gBoss[0].x, gBoss[0].y, FLASH_MODE_EXPLOSION);
				PlaySoundObject(35, SOUND_MODE_PLAY);
			}

			break;

		case 1001:
			SetQuake(40);

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait > 50)
			{
				for (i = 0; i < 20; ++i)
					gBoss[i].cond = 0;

				DeleteNpCharCode(158, TRUE);
				DeleteNpCharCode(301, TRUE);
			}

			break;
	}

	if (bShock)
	{
		SetQuake(20);

		if (npc->act_no == 201)
			gBoss[1].act_no = gBoss[2].act_no = gBoss[6].act_no = gBoss[7].act_no = 10;

		if (npc->act_no == 221)
			gBoss[1].act_no = gBoss[2].act_no = gBoss[6].act_no = gBoss[7].act_no = 20;

		if (npc->act_no == 231)
			gBoss[1].act_no = gBoss[2].act_no = gBoss[6].act_no = gBoss[7].act_no = 30;

		PlaySoundObject(26, SOUND_MODE_PLAY);

		for (i = 0; i < 8; ++i)
			SetNpChar(4, gBoss[4].x + (Random(-32, 16) * 0x200), gBoss[4].y, Random(-0x200, 0x200), Random(-0x100, 0x100), 0, NULL, 0x100);
	}

	if (npc->act_no >= 200 && npc->act_no < 300)
	{
		if (npc->x < 192 * 0x200)
			npc->direct = 2;
		if (npc->x > (gMap.width - 4) * (0x200 * 0x10))
			npc->direct = 0;

		if (npc->direct == 0)
			npc->xm -= 4;
		else
			npc->xm += 4;
	}

	switch (npc->act_no)
	{
		case 201:
		case 211:
		case 221:
		case 231:
			++npc->count2;

			if (npc->count2 == 150)
			{
				npc->count2 = 0;
				SetNpChar(282, (gMap.width * 0x200 * 0x10) + 0x40, (Random(-1, 3) + 10) * (0x200 * 0x10), 0, 0, 0, NULL, 0x30);
			}
			else if (npc->count2 == 75)
			{
				SetNpChar(282, (gMap.width * 0x200 * 0x10) + 0x40, (Random(-3, 0) + 3) * (0x200 * 0x10), 0, 0, 0, NULL, 0x30);
			}

			break;
	}

	if (npc->xm > 0x80)
		npc->xm = 0x80;
	if (npc->xm < -0x80)
		npc->xm = -0x80;

	if (npc->ym > 0x80)
		npc->ym = 0x80;
	if (npc->ym < -0x80)
		npc->ym = -0x80;

	npc->x += npc->xm;
	npc->y += npc->ym;

	ActBossCharA_Face(&gBoss[3]);

	ActBossCharA_Head(&gBoss[4]);

	ActBossCharA_Tail(&gBoss[5]);

	ActBossCharA_Mini(&gBoss[1]);
	ActBossCharA_Mini(&gBoss[2]);
	ActBossCharA_Mini(&gBoss[6]);
	ActBossCharA_Mini(&gBoss[7]);

	ActBossCharA_Hit(&gBoss[8]);
	ActBossCharA_Hit(&gBoss[9]);
	ActBossCharA_Hit(&gBoss[10]);
	ActBossCharA_Hit(&gBoss[11]);
}

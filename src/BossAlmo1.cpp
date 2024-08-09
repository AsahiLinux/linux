// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossAlmo1.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Frame.h"
#include "Game.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

void ActBossChar_Core_Face(NPCHAR *npc)
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

			npc->x = gBoss[0].x - (36 * 0x200);
			npc->y = gBoss[0].y;
			break;

		case 100:
			npc->ani_no = 3;
			break;
	}

	npc->rect = rect[npc->ani_no];

	if (npc->act_no == 51)
		npc->rect.bottom = npc->rect.top + npc->act_wait;
}

void ActBossChar_Core_Tail(NPCHAR *npc)
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

			npc->x = gBoss[0].x + (44 * 0x200);
			npc->y = gBoss[0].y;
			break;

		case 100:
			npc->ani_no = 2;
			break;
	}

	npc->rect = rect[npc->ani_no];

	if (npc->act_no == 51)
		npc->rect.bottom = npc->rect.top + npc->act_wait;
}

void ActBossChar_Core_Mini(NPCHAR *npc)
{
	RECT rect[3] = {
		{256, 0, 320, 40},
		{256, 40, 320, 80},
		{256, 80, 320, 120},
	};

	int xm, ym;
	unsigned char deg;

	npc->life = 1000;

	switch (npc->act_no)
	{
		case 10:
			npc->ani_no = 2;
			npc->bits &= ~NPC_SHOOTABLE;
			break;

		case 100:
			npc->act_no = 101;
			npc->ani_no = 2;
			npc->act_wait = 0;
			npc->tgt_x = gBoss[0].x + (Random(-0x80, 0x20) * 0x200);
			npc->tgt_y = gBoss[0].y + (Random(-0x40, 0x40) * 0x200);
			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 101:
			npc->x += (npc->tgt_x - npc->x) / 0x10;
			npc->y += (npc->tgt_y - npc->y) / 0x10;

			++npc->act_wait;

			if (npc->act_wait > 50)
				npc->ani_no = 0;

			break;

		case 120:
			npc->act_no = 121;
			npc->act_wait = 0;
			// Fallthrough
		case 121:
			++npc->act_wait;

			if (npc->act_wait / 2 % 2)
				npc->ani_no = 0;
			else
				npc->ani_no = 1;

			if (npc->act_wait > 20)
				npc->act_no = 130;

			break;

		case 130:
			npc->act_no = 131;
			npc->ani_no = 2;
			npc->act_wait = 0;
			npc->tgt_x = npc->x + (Random(24, 48) * 0x200);
			npc->tgt_y = npc->y + (Random(-4, 4) * 0x200);
			// Fallthrough
		case 131:
			npc->x += (npc->tgt_x - npc->x) / 0x10;
			npc->y += (npc->tgt_y - npc->y) / 0x10;

			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				npc->act_no = 140;
				npc->ani_no = 0;
			}

			if (npc->act_wait == 1 || npc->act_wait == 3)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				deg += (unsigned char)Random(-2, 2);
				ym = GetSin(deg) * 2;
				xm = GetCos(deg) * 2;
				SetNpChar(178, npc->x, npc->y, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			break;

		case 140:
			npc->x += (npc->tgt_x - npc->x) / 0x10;
			npc->y += (npc->tgt_y - npc->y) / 0x10;
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

	if (npc->shock != 0)
		npc->tgt_x += 2 * 0x200;

	npc->rect = rect[npc->ani_no];
}

void ActBossChar_Core_Hit(NPCHAR *npc)
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

void ActBossChar_Core(void)
{
	NPCHAR *npc = gBoss;

	static unsigned char flash;
	BOOL bShock = FALSE;
	int i;
	unsigned char deg;
	int xm, ym;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->exp = 1;
			npc->cond = 0x80;
			npc->bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_SHOW_DAMAGE);
			npc->life = 650;
			npc->hit_voice = 114;
			npc->x = 77 * 0x10 * 0x200;
			npc->y = 14 * 0x10 * 0x200;
			npc->xm = 0;
			npc->ym = 0;
			npc->code_event = 1000;
			npc->bits |= NPC_EVENT_WHEN_KILLED;

			gBoss[4].cond = 0x80;
			gBoss[4].act_no = 10;

			gBoss[5].cond = 0x80;
			gBoss[5].act_no = 10;

			gBoss[8].cond = 0x80;
			gBoss[8].bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY);
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
			gBoss[1].act_no = 10;
			gBoss[1].bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_SHOOTABLE);
			gBoss[1].life = 1000;
			gBoss[1].hit_voice = 54;
			gBoss[1].hit.back = 24 * 0x200;
			gBoss[1].hit.top = 16 * 0x200;
			gBoss[1].hit.bottom = 16 * 0x200;
			gBoss[1].view.front = 32 * 0x200;
			gBoss[1].view.top = 20 * 0x200;
			gBoss[1].x = npc->x - (8 * 0x200);
			gBoss[1].y = npc->y - (64 * 0x200);

			gBoss[2] = gBoss[1];
			gBoss[2].x = npc->x + (16 * 0x200);
			gBoss[2].y = npc->y;

			gBoss[3] = gBoss[1];
			gBoss[3].x = npc->x - (8 * 0x200);
			gBoss[3].y = npc->y + (64 * 0x200);

			gBoss[6] = gBoss[1];
			gBoss[6].x = npc->x - (48 * 0x200);
			gBoss[6].y = npc->y - (32 * 0x200);

			gBoss[7] = gBoss[1];
			gBoss[7].x = npc->x - (48 * 0x200);
			gBoss[7].y = npc->y + (32 * 0x200);
			break;

		case 200:
			npc->act_no = 201;
			npc->act_wait = 0;
			gBoss[11].bits &= ~NPC_SHOOTABLE;
			gSuperYpos = 0;
			CutNoise();
			// Fallthrough
		case 201:
			npc->tgt_x = gMC.x;
			npc->tgt_y = gMC.y;

			++npc->act_wait;

			if (npc->act_wait > 400)
			{
				++npc->count1;
				PlaySoundObject(115, SOUND_MODE_PLAY);

				if (npc->count1 > 3)
				{
					npc->count1 = 0;
					npc->act_no = 220;
					gBoss[4].ani_no = 0;
					gBoss[5].ani_no = 0;
					bShock = TRUE;
				}
				else
				{
					npc->act_no = 210;
					gBoss[4].ani_no = 0;
					gBoss[5].ani_no = 0;
					bShock = TRUE;
				}
			}

			break;

		case 210:
			npc->act_no = 211;
			npc->act_wait = 0;
			npc->count2 = npc->life;
			gBoss[11].bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 211:
			npc->tgt_x = gMC.x;
			npc->tgt_y = gMC.y;

			if (npc->shock != 0)
			{
				if (++flash / 2 % 2)
				{
					gBoss[4].ani_no = 0;
					gBoss[5].ani_no = 0;
				}
				else
				{
					gBoss[4].ani_no = 1;
					gBoss[5].ani_no = 1;
				}
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

			if (npc->act_wait < 200 && npc->act_wait % 20 == 1)
				SetNpChar(179, npc->x + (Random(-48, -16) * 0x200), npc->y + (Random(-64, 64) * 0x200), 0, 0, 0, NULL, 0x100);

			if (npc->act_wait > 400 || npc->life < npc->count2 - 200)
			{
				npc->act_no = 200;
				gBoss[4].ani_no = 2;
				gBoss[5].ani_no = 0;
				bShock = TRUE;
			}

			break;

		case 220:
			npc->act_no = 221;
			npc->act_wait = 0;
			gSuperYpos = 1;
			gBoss[11].bits |= NPC_SHOOTABLE;
			SetQuake(100);
			SetNoise(1, 1000);
			// Fallthrough
		case 221:
			++npc->act_wait;
			SetNpChar(199, gMC.x + (Random(-50, 150) * 0x200 * 2), gMC.y + (Random(-160, 160) * 0x200), 0, 0, 0, NULL, 0x100);
			gMC.xm -= 0x20;
			gMC.cond |= 0x20;

			if (npc->shock != 0)
			{
				if (++flash / 2 % 2)
				{
					gBoss[4].ani_no = 0;
					gBoss[5].ani_no = 0;
				}
				else
				{
					gBoss[4].ani_no = 1;
					gBoss[5].ani_no = 1;
				}
			}
			else
			{
				gBoss[4].ani_no = 0;
				gBoss[5].ani_no = 0;
			}

			if (npc->act_wait == 300 || npc->act_wait == 350 || npc->act_wait == 400)
			{
				deg = GetArktan(npc->x - gMC.x, npc->y - gMC.y);
				ym = GetSin(deg) * 3;
				xm = GetCos(deg) * 3;
				SetNpChar(218, npc->x - (40 * 0x200), npc->y, xm, ym, 0, NULL, 0x100);
				PlaySoundObject(101, SOUND_MODE_PLAY);
			}

			if (npc->act_wait > 400)
			{
				npc->act_no = 200;
				gBoss[4].ani_no = 2;
				gBoss[5].ani_no = 0;
				bShock = TRUE;
			}

			break;

		case 500:
			CutNoise();

			npc->act_no = 501;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;
			gBoss[4].ani_no = 2;
			gBoss[5].ani_no = 0;
			gBoss[1].act_no = 200;
			gBoss[2].act_no = 200;
			gBoss[3].act_no = 200;
			gBoss[6].act_no = 200;
			gBoss[7].act_no = 200;

			SetQuake(20);

			for (i = 0; i < 0x20; ++i)
				SetNpChar(4, npc->x + (Random(-128, 128) * 0x200), npc->y + (Random(-64, 64) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0x100);

			for (i = 0; i < 12; ++i)
				gBoss[i].bits &= ~(NPC_INVULNERABLE | NPC_SHOOTABLE);

			// Fallthrough
		case 501:
			++npc->act_wait;

			if (npc->act_wait % 16)
				SetNpChar(4, npc->x + (Random(-64, 64) * 0x200), npc->y + (Random(-32, 32) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0x100);

			if (npc->act_wait / 2 % 2)
				npc->x -= 0x200;
			else
				npc->x += 0x200;

			if (npc->x < 63 * 0x10 * 0x200)
				npc->x += 0x80;
			else
				npc->x -= 0x80;

			if (npc->y < 11 * 0x10 * 0x200)
				npc->y += 0x80;
			else
				npc->y -= 0x80;

			break;

		case 600:
			npc->act_no = 601;
			gBoss[4].act_no = 50;
			gBoss[5].act_no = 50;
			gBoss[8].bits &= ~NPC_INVULNERABLE;
			gBoss[9].bits &= ~NPC_INVULNERABLE;
			gBoss[10].bits &= ~NPC_INVULNERABLE;
			gBoss[11].bits &= ~NPC_INVULNERABLE;
			// Fallthrough
		case 601:
			++npc->act_wait;

			if (npc->act_wait / 2 % 2)
				npc->x -= 4 * 0x200;
			else
				npc->x += 4 * 0x200;

			break;
	}

	if (bShock)
	{
		SetQuake(20);

		gBoss[1].act_no = 100;
		gBoss[2].act_no = 100;
		gBoss[3].act_no = 100;
		gBoss[6].act_no = 100;
		gBoss[7].act_no = 100;

		PlaySoundObject(26, SOUND_MODE_PLAY);

		for (i = 0; i < 8; ++i)
			SetNpChar(4, gBoss[4].x + (Random(-32, 16) * 0x200), gBoss[4].y, Random(-0x200, 0x200), Random(-0x100, 0x100), 0, NULL, 0x100);
	}

	if (npc->act_no >= 200 && npc->act_no < 300)
	{
		switch (npc->act_wait)
		{
			case 80:
				gBoss[1].act_no = 120;
				break;
			case 110:
				gBoss[2].act_no = 120;
				break;
			case 140:
				gBoss[3].act_no = 120;
				break;
			case 170:
				gBoss[6].act_no = 120;
				break;
			case 200:
				gBoss[7].act_no = 120;
				break;
		}

		if (npc->x < npc->tgt_x + (10 * 0x10 * 0x200))
			npc->xm += 4;
		if (npc->x > npc->tgt_x + (10 * 0x10 * 0x200))
			npc->xm -= 4;

		if (npc->y < npc->tgt_y)
			npc->ym += 4;
		if (npc->y > npc->tgt_y)
			npc->ym -= 4;
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

	ActBossChar_Core_Face(&gBoss[4]);

	ActBossChar_Core_Tail(&gBoss[5]);

	ActBossChar_Core_Mini(&gBoss[1]);
	ActBossChar_Core_Mini(&gBoss[2]);
	ActBossChar_Core_Mini(&gBoss[3]);
	ActBossChar_Core_Mini(&gBoss[6]);
	ActBossChar_Core_Mini(&gBoss[7]);

	ActBossChar_Core_Hit(&gBoss[8]);
	ActBossChar_Core_Hit(&gBoss[9]);
	ActBossChar_Core_Hit(&gBoss[10]);
	ActBossChar_Core_Hit(&gBoss[11]);
}

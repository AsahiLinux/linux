// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossBallos.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Flash.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "MycParam.h"
#include "NpChar.h"
#include "Sound.h"

static void ActBossChar_Eye(NPCHAR *npc)
{
	RECT rcLeft[5] = {
		{272, 0, 296, 16},
		{272, 16, 296, 32},
		{272, 32, 296, 48},
		{0, 0, 0, 0},
		{240, 16, 264, 32},
	};

	RECT rcRight[5] = {
		{296, 0, 320, 16},
		{296, 16, 320, 32},
		{296, 32, 320, 48},
		{0, 0, 0, 0},
		{240, 32, 264, 48},
	};

	switch (npc->act_no)
	{
		case 100:
			npc->act_no = 101;
			npc->ani_no = 0;
			npc->ani_wait = 0;
			// Fallthrough
		case 101:
			++npc->ani_wait;

			if (npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 2)
				npc->act_no = 102;

			break;

		case 102:
			npc->ani_no = 3;
			break;

		case 200:
			npc->act_no = 201;
			npc->ani_no = 3;
			npc->ani_wait = 0;
			// Fallthrough
		case 201:
			++npc->ani_wait;

			if (npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				--npc->ani_no;
			}

			if (npc->ani_no <= 0)
				npc->act_no = 202;

			break;

		case 300:
			npc->act_no = 301;
			npc->ani_no = 4;

			if (npc->direct == 0)
				SetDestroyNpChar(npc->x - (4 * 0x200), npc->y, 0x800, 10);
			else
				SetDestroyNpChar(npc->x + (4 * 0x200), npc->y, 0x800, 10);

			break;
	}

	if (npc->direct == 0)
		npc->x = gBoss[0].x - (24 * 0x200);
	else
		npc->x = gBoss[0].x + (24 * 0x200);

	npc->y = gBoss[0].y - (36 * 0x200);

	if (npc->act_no >= 0 && npc->act_no < 300)
	{
		if (npc->ani_no != 3)
			npc->bits &= ~NPC_SHOOTABLE;
		else
			npc->bits |= NPC_SHOOTABLE;
	}

	if (npc->direct == 0)
		npc->rect = rcLeft[npc->ani_no];
	else
		npc->rect = rcRight[npc->ani_no];
}

static void ActBossChar_Body(NPCHAR *npc)
{
	RECT rc[4] = {
		{0, 0, 120, 120},
		{120, 0, 240, 120},
		{0, 120, 120, 240},
		{120, 120, 240, 240},
	};

	npc->x = gBoss[0].x;
	npc->y = gBoss[0].y;

	npc->rect = rc[npc->ani_no];
}

static void ActBossChar_HITAI(NPCHAR *npc)	// "Hitai" = "forehead" or "brow" (according to Google Translate, anyway)
{
	npc->x = gBoss[0].x;
	npc->y = gBoss[0].y - (44 * 0x200);
}

static void ActBossChar_HARA(NPCHAR *npc)	// "Hara" = "belly" or "stomach" (according to Google Translate, anyway)
{
	npc->x = gBoss[0].x;
	npc->y = gBoss[0].y;
}

void ActBossChar_Ballos(void)
{
	NPCHAR *npc = gBoss;

	static unsigned char flash;
	int i;
	int x, y;

	switch (npc->act_no)
	{
		case 0:
			// Initialize main boss
			npc->act_no = 1;
			npc->cond = 0x80;
			npc->exp = 1;
			npc->direct = 0;
			npc->x = 320 * 0x200;
			npc->y = -64 * 0x200;
			npc->hit_voice = 54;
			npc->hit.front = 32 * 0x200;
			npc->hit.top = 48 * 0x200;
			npc->hit.back = 32 * 0x200;
			npc->hit.bottom = 48 * 0x200;
			npc->bits = (NPC_IGNORE_SOLIDITY | NPC_SOLID_HARD | NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			npc->size = 3;
			npc->damage = 0;
			npc->code_event = 1000;
			npc->life = 800;

			// Initialize eyes
			gBoss[1].cond = 0x90;
			gBoss[1].direct = 0;
			gBoss[1].bits = NPC_IGNORE_SOLIDITY;
			gBoss[1].life = 10000;
			gBoss[1].view.front = 12 * 0x200;
			gBoss[1].view.top = 0;
			gBoss[1].view.back = 12 * 0x200;
			gBoss[1].view.bottom = 16 * 0x200;
			gBoss[1].hit.front = 12 * 0x200;
			gBoss[1].hit.top = 0;
			gBoss[1].hit.back = 12 * 0x200;
			gBoss[1].hit.bottom = 16 * 0x200;

			gBoss[2] = gBoss[1];
			gBoss[2].direct = 2;

			// Initialize the body
			gBoss[3].cond = 0x90;
			gBoss[3].bits = (NPC_SOLID_SOFT | NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY);
			gBoss[3].view.front = 60 * 0x200;
			gBoss[3].view.top = 60 * 0x200;
			gBoss[3].view.back = 60 * 0x200;
			gBoss[3].view.bottom = 60 * 0x200;
			gBoss[3].hit.front = 48 * 0x200;
			gBoss[3].hit.top = 24 * 0x200;
			gBoss[3].hit.back = 48 * 0x200;
			gBoss[3].hit.bottom = 32 * 0x200;

			gBoss[4].cond = 0x90;
			gBoss[4].bits = (NPC_SOLID_SOFT | NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY);
			gBoss[4].hit.front = 32 * 0x200;
			gBoss[4].hit.top = 8 * 0x200;
			gBoss[4].hit.back = 32 * 0x200;
			gBoss[4].hit.bottom = 8 * 0x200;

			gBoss[5].cond = 0x90;
			gBoss[5].bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_SOLID_HARD);
			gBoss[5].hit.front = 32 * 0x200;
			gBoss[5].hit.top = 0;
			gBoss[5].hit.back = 32 * 0x200;
			gBoss[5].hit.bottom = 48 * 0x200;
			break;

		case 100:
			npc->act_no = 101;
			npc->ani_no = 0;
			npc->x = gMC.x;
			SetNpChar(333, gMC.x, 304 * 0x200, 0, 0, 2, NULL, 0x100);
			npc->act_wait = 0;
			// Fallthrough
		case 101:
			++npc->act_wait;

			if (npc->act_wait > 30)
				npc->act_no = 102;

			break;

		case 102:
			npc->ym += 0x40;
			if (npc->ym > 0xC00)
				npc->ym = 0xC00;

			npc->y += npc->ym;

			if (npc->y > (304 * 0x200) - npc->hit.bottom)
			{
				npc->y = (304 * 0x200) - npc->hit.bottom;
				npc->ym = 0;
				npc->act_no = 103;
				npc->act_wait = 0;
				SetQuake2(30);
				PlaySoundObject(44, SOUND_MODE_PLAY);

				if (gMC.y > npc->y + (48 * 0x200) && gMC.x < npc->x + (24 * 0x200) && gMC.x > npc->x - (24 * 0x200))
					DamageMyChar(16);

				for (i = 0; i < 0x10; ++i)
				{
					x = npc->x + (Random(-40, 40) * 0x200);
					SetNpChar(4, x, npc->y + (40 * 0x200), 0, 0, 0, NULL, 0x100);
				}

				if (gMC.flag & 8)
					gMC.ym = -0x200;
			}

			break;

		case 103:
			++npc->act_wait;

			if (npc->act_wait == 50)
			{
				npc->act_no = 104;
				gBoss[1].act_no = 100;
				gBoss[2].act_no = 100;
			}

			break;

		case 200:
			npc->act_no = 201;
			npc->count1 = 0;
			// Fallthrough
		case 201:
			npc->act_no = 203;
			npc->xm = 0;
			++npc->count1;
			npc->hit.bottom = 48 * 0x200;
			npc->damage = 0;

			if (npc->count1 % 3 == 0)
				npc->act_wait = 150;
			else
				npc->act_wait = 50;
			// Fallthrough
		case 203:
			--npc->act_wait;

			if (npc->act_wait <= 0)
			{
				npc->act_no = 204;
				npc->ym = -0xC00;

				if (npc->x < gMC.x)
					npc->xm = 0x200;
				else
					npc->xm = -0x200;
			}

			break;

		case 204:
			if (npc->x < 80 * 0x200)
				npc->xm = 0x200;
			if (npc->x > 544 * 0x200)
				npc->xm = -0x200;

			npc->ym += 0x55;
			if (npc->ym > 0xC00)
				npc->ym = 0xC00;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->y > (304 * 0x200) - npc->hit.bottom)
			{
				npc->y = (304 * 0x200) - npc->hit.bottom;
				npc->ym = 0;
				npc->act_no = 201;
				npc->act_wait = 0;

				if (gMC.y > npc->y + (56 * 0x200))
					DamageMyChar(16);

				if (gMC.flag & 8)
					gMC.ym = -0x200;

				SetQuake2(30);
				PlaySoundObject(26, SOUND_MODE_PLAY);
				SetNpChar(332, npc->x - (12 * 0x200), npc->y + (52 * 0x200), 0, 0, 0, NULL, 0x100);
				SetNpChar(332, npc->x + (12 * 0x200), npc->y + (52 * 0x200), 0, 0, 2, NULL, 0x100);
				PlaySoundObject(44, SOUND_MODE_PLAY);

				for (i = 0; i < 0x10; ++i)
				{
					x = npc->x + (Random(-40, 40) * 0x200);
					SetNpChar(4, x, npc->y + (40 * 0x200), 0, 0, 0, NULL, 0x100);
				}
			}

			break;

		case 220:
			npc->act_no = 221;
			npc->life = 1200;
			gBoss[1].act_no = 200;
			gBoss[2].act_no = 200;
			npc->xm = 0;
			npc->ani_no = 0;
			npc->shock = 0;
			flash = 0;
			// Fallthrough
		case 221:
			npc->ym += 0x40;
			if (npc->ym > 0xC00)
				npc->ym = 0xC00;

			npc->y += npc->ym;

			if (npc->y > (304 * 0x200) - npc->hit.bottom)
			{
				npc->y = (304 * 0x200) - npc->hit.bottom;
				npc->ym = 0;
				npc->act_no = 222;
				npc->act_wait = 0;
				SetQuake2(30);
				PlaySoundObject(26, SOUND_MODE_PLAY);

				for (i = 0; i < 0x10; ++i)
				{
					x = npc->x + (Random(-40, 40) * 0x200);
					SetNpChar(4, x, npc->y + (40 * 0x200), 0, 0, 0, NULL, 0x100);
				}

				if (gMC.flag & 8)
					gMC.ym = -0x200;
			}

			break;

		case 300:
			npc->act_no = 301;
			npc->act_wait = 0;

			for (i = 0; i < 0x100; i += 0x40)
			{
				SetNpChar(342, npc->x, npc->y, 0, 0, i, npc, 90);
				SetNpChar(342, npc->x, npc->y, 0, 0, i + 0x220, npc, 90);
			}

			SetNpChar(343, npc->x, npc->y, 0, 0, 0, npc, 0x18);
			SetNpChar(344, npc->x - (24 * 0x200), npc->y - (36 * 0x200), 0, 0, 0, npc, 0x20);
			SetNpChar(344, npc->x + (24 * 0x200), npc->y - (36 * 0x200), 0, 0, 2, npc, 0x20);
			// Fallthrough
		case 301:
			npc->y += ((225 * 0x200) - npc->y) / 8;

			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				npc->act_no = 310;
				npc->act_wait = 0;
			}

			break;

		case 311:
			npc->direct = 0;
			npc->xm = -0x3AA;
			npc->ym = 0;
			npc->x += npc->xm;

			if (npc->x < 111 * 0x200)
			{
				npc->x = 111 * 0x200;
				npc->act_no = 312;
			}

			break;

		case 312:
			npc->direct = 1;
			npc->ym = -0x3AA;
			npc->xm = 0;
			npc->y += npc->ym;

			if (npc->y < 111 * 0x200)
			{
				npc->y = 111 * 0x200;
				npc->act_no = 313;
			}

			break;

		case 313:
			npc->direct = 2;
			npc->xm = 0x3AA;
			npc->ym = 0;
			npc->x += npc->xm;

			if (npc->x > 513 * 0x200)
			{
				npc->x = 513 * 0x200;
				npc->act_no = 314;
			}

			if (npc->count1 != 0)
				--npc->count1;

			if (npc->count1 == 0 && npc->x > 304 * 0x200 && npc->x < 336 * 0x200)
				npc->act_no = 400;

			break;

		case 314:
			npc->direct = 3;
			npc->ym = 0x3AA;
			npc->xm = 0;
			npc->y += npc->ym;

			if (npc->y > 225 * 0x200)
			{
				npc->y = 225 * 0x200;
				npc->act_no = 311;
			}

			break;

		case 400:
			npc->act_no = 401;
			npc->act_wait = 0;
			npc->xm = 0;
			npc->ym = 0;
			DeleteNpCharCode(339, FALSE);
			// Fallthrough
		case 401:
			npc->y += ((159 * 0x200) - npc->y) / 8;

			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				npc->act_wait = 0;
				npc->act_no = 410;

				for (i = 0; i < 0x100; i += 0x20)
					SetNpChar(346, npc->x, npc->y, 0, 0, i, npc, 0x50);

				SetNpChar(343, npc->x, npc->y, 0, 0, 0, npc, 0x18);
				SetNpChar(344, npc->x - (24 * 0x200), npc->y - (36 * 0x200), 0, 0, 0, npc, 0x20);
				SetNpChar(344, npc->x + (24 * 0x200), npc->y - (36 * 0x200), 0, 0, 2, npc, 0x20);
			}

			break;

		case 410:
			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				npc->act_wait = 0;
				npc->act_no = 411;
			}

			break;

		case 411:
			++npc->act_wait;

			if (npc->act_wait % 30 == 1)
			{
				x = (((npc->act_wait / 30) * 2) + 2) * 0x10 * 0x200;
				SetNpChar(348, x, 336 * 0x200, 0, 0, 0, NULL, 0x180);
			}

			if (npc->act_wait / 3 % 2)
				PlaySoundObject(26, SOUND_MODE_PLAY);

			if (npc->act_wait > 540)
				npc->act_no = 420;

			break;

		case 420:
			npc->act_no = 421;
			npc->act_wait = 0;
			npc->ani_wait = 0;
			SetQuake2(30);
			PlaySoundObject(35, SOUND_MODE_PLAY);
			gBoss[1].act_no = 102;
			gBoss[2].act_no = 102;

			for (i = 0; i < 0x100; ++i)
			{
				x = npc->x + (Random(-60, 60) * 0x200);
				y = npc->y + (Random(-60, 60) * 0x200);
				SetNpChar(4, x, y, 0, 0, 0, NULL, 0);
			}
			// Fallthrough
		case 421:
			++npc->ani_wait;

			if (npc->ani_wait > 500)
			{
				npc->ani_wait = 0;
				npc->act_no = 422;
			}

			break;

		case 422:
			++npc->ani_wait;

			if (npc->ani_wait > 200)
			{
				npc->ani_wait = 0;
				npc->act_no = 423;
			}

			break;

		case 423:
			++npc->ani_wait;

			if (npc->ani_wait > 20)
			{
				npc->ani_wait = 0;
				npc->act_no = 424;
			}

			break;

		case 424:
			++npc->ani_wait;

			if (npc->ani_wait > 200)
			{
				npc->ani_wait = 0;
				npc->act_no = 425;
			}

			break;

		case 425:
			++npc->ani_wait;

			if (npc->ani_wait > 500)
			{
				npc->ani_wait = 0;
				npc->act_no = 426;
			}

			break;

		case 426:
			++npc->ani_wait;

			if (npc->ani_wait > 200)
			{
				npc->ani_wait = 0;
				npc->act_no = 427;
			}

			break;

		case 427:
			++npc->ani_wait;

			if (npc->ani_wait > 20)
			{
				npc->ani_wait = 0;
				npc->act_no = 428;
			}

			break;

		case 428:
			++npc->ani_wait;

			if (npc->ani_wait > 200)
			{
				npc->ani_wait = 0;
				npc->act_no = 421;
			}

			break;

		case 1000:
			npc->act_no = 1001;
			npc->act_wait = 0;

			gBoss[1].act_no = 300;
			gBoss[2].act_no = 300;

#ifndef FIX_BUGS
			// This code makes absolutely no sense.
			// Luckily, it doesn't cause any bugs.
			gBoss[1].act_no &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
			gBoss[2].act_no &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
#endif

			gBoss[0].bits &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
			gBoss[3].bits &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
			gBoss[4].bits &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
			gBoss[5].bits &= ~(NPC_SOLID_SOFT | NPC_SOLID_HARD);
			// Fallthrough
		case 1001:
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait % 12 == 0)
				PlaySoundObject(44, SOUND_MODE_PLAY);

			SetDestroyNpChar(gBoss[0].x + (Random(-60, 60) * 0x200), gBoss[0].y + (Random(-60, 60) * 0x200), 1, 1);

			if (gBoss[0].act_wait > 150)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 1002;
				SetFlash(gBoss[0].x, gBoss[0].y, FLASH_MODE_EXPLOSION);
				PlaySoundObject(35, SOUND_MODE_PLAY);
			}

			break;

		case 1002:
			SetQuake2(40);

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait == 50)
			{
				gBoss[0].cond = 0;
				gBoss[1].cond = 0;
				gBoss[2].cond = 0;
				gBoss[3].cond = 0;
				gBoss[4].cond = 0;
				gBoss[5].cond = 0;

				DeleteNpCharCode(350, TRUE);
				DeleteNpCharCode(348, TRUE);
			}

			break;
	}

	if (npc->act_no > 420 && npc->act_no < 500)
	{
		gBoss[3].bits |= NPC_SHOOTABLE;
		gBoss[4].bits |= NPC_SHOOTABLE;
		gBoss[5].bits |= NPC_SHOOTABLE;

		++npc->act_wait;

		if (npc->act_wait > 300)
		{
			npc->act_wait = 0;

			if (gMC.x > npc->x)
			{
				for (i = 0; i < 8; ++i)
				{
					x = ((156 + Random(-4, 4)) * 0x200 * 0x10) / 4;
					y = (Random(8, 68) * 0x200 * 0x10) / 4;
					SetNpChar(350, x, y, 0, 0, 0, NULL, 0x100);
				}
			}
			else
			{
				for (i = 0; i < 8; ++i)
				{
					x = (Random(-4, 4) * 0x200 * 0x10) / 4;
					y = (Random(8, 68) * 0x200 * 0x10) / 4;
					SetNpChar(350, x, y, 0, 0, 2, NULL, 0x100);
				}
			}
		}

		if (npc->act_wait == 270 || npc->act_wait == 280 || npc->act_wait == 290)
		{
			SetNpChar(353, npc->x, npc->y - (52 * 0x200), 0, 0, 1, NULL, 0x100);
			PlaySoundObject(39, SOUND_MODE_PLAY);

			for (i = 0; i < 4; ++i)
				SetNpChar(4, npc->x, npc->y - (52 * 0x200), 0, 0, 0, NULL, 0x100);
		}

		if (npc->life > 500)
		{
			if (Random(0, 10) == 2)
			{
				x = npc->x + (Random(-40, 40) * 0x200);
				y = npc->y + (Random(0, 40) * 0x200);
				SetNpChar(270, x, y, 0, 0, 3, NULL, 0);
			}
		}
		else
		{
			if (Random(0, 4) == 2)
			{
				x = npc->x + (Random(-40, 40) * 0x200);
				y = npc->y + (Random(0, 40) * 0x200);
				SetNpChar(270, x, y, 0, 0, 3, NULL, 0);
			}
		}
	}

	if (npc->shock != 0)
	{
		if (++flash / 2 % 2)
			gBoss[3].ani_no = 1;
		else
			gBoss[3].ani_no = 0;
	}
	else
	{
		gBoss[3].ani_no = 0;
	}

	if (npc->act_no > 420)
		gBoss[3].ani_no += 2;

	ActBossChar_Eye(&gBoss[1]);
	ActBossChar_Eye(&gBoss[2]);
	ActBossChar_Body(&gBoss[3]);
	ActBossChar_HITAI(&gBoss[4]);
	ActBossChar_HARA(&gBoss[5]);
}

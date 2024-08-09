// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossPress.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Game.h"
#include "Map.h"
#include "NpChar.h"
#include "Sound.h"

void ActBossChar_Press(void)
{
	NPCHAR *npc = gBoss;
	static unsigned char flash;
	int i;
	int x;

	switch (npc->act_no)
	{
		case 0:
			npc->act_no = 10;
			npc->cond = 0x80;
			npc->exp = 1;
			npc->direct = 2;
			npc->x = 0;
			npc->y = 0;
			npc->view.front = 40 * 0x200;
			npc->view.top = 60 * 0x200;
			npc->view.back = 40 * 0x200;
			npc->view.bottom = 60 * 0x200;
			npc->hit_voice = 54;
			npc->hit.front = 49 * 0x200;
			npc->hit.top = 60 * 0x200;
			npc->hit.back = 40 * 0x200;
			npc->hit.bottom = 48 * 0x200;
			npc->bits = (NPC_IGNORE_SOLIDITY | NPC_SOLID_HARD | NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			npc->size = 3;
			npc->damage = 10;
			npc->code_event = 1000;
			npc->life = 700;
			break;

		case 5:
			npc->act_no = 6;
			npc->x = 0;
			npc->y = 0;
			gBoss[1].cond = 0;
			gBoss[2].cond = 0;
			break;

		case 10:
			npc->act_no = 11;
			npc->x = 160 * 0x200;
			npc->y = 74 * 0x200;
			break;

		case 20:
			npc->damage = 0;
			npc->act_no = 21;
			npc->x = 160 * 0x200;
			npc->y = 413 * 0x200;
			npc->bits &= ~NPC_SOLID_HARD;
			gBoss[1].cond = 0;
			gBoss[2].cond = 0;
			// Fallthrough
		case 21:
			if (++npc->act_wait % 0x10 == 0)
				SetDestroyNpChar(npc->x + (Random(-40, 40) * 0x200), npc->y + (Random(-60, 60) * 0x200), 1, 1);

			break;

		case 30:
			npc->act_no = 31;
			npc->ani_no = 2;
			npc->x = 160 * 0x200;
			npc->y = 64 * 0x200;
			// Fallthrough
		case 31:
			npc->y += 4 * 0x200;

			if (npc->y >= 413 * 0x200)
			{
				npc->y = 413 * 0x200;
				npc->ani_no = 0;
				npc->act_no = 20;
				PlaySoundObject(44, SOUND_MODE_PLAY);

				for (i = 0; i < 5; ++i)
				{
					x = npc->x + (Random(-40, 40) * 0x200);
					SetNpChar(4, x, npc->y + (60 * 0x200), 0, 0, 0, NULL, 0x100);
				}

			}

			break;

		case 100:
			npc->act_no = 101;
			npc->count2 = 9;
			npc->act_wait = -100;

			gBoss[1].cond = 0x80;
			gBoss[1].hit.front = 14 * 0x200;
			gBoss[1].hit.back = 14 * 0x200;
			gBoss[1].hit.top = 8 * 0x200;
			gBoss[1].hit.bottom = 8 * 0x200;
			gBoss[1].bits = (NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY);

			gBoss[2] = gBoss[1];

			gBoss[3].cond = 0x90;
			gBoss[3].bits |= NPC_SHOOTABLE;
			gBoss[3].hit.front = 6 * 0x200;
			gBoss[3].hit.back = 6 * 0x200;
			gBoss[3].hit.top = 8 * 0x200;
			gBoss[3].hit.bottom = 8 * 0x200;

			SetNpChar(325, npc->x, npc->y + (60 * 0x200), 0, 0, 0, NULL, 0x100);
			// Fallthrough
		case 101:
			if (npc->count2 > 1 && npc->life < npc->count2 * 70)
			{
				--npc->count2;

				for (i = 0; i < 5; ++i)
				{
					ChangeMapParts(i + 8, npc->count2, 0);
					SetDestroyNpChar((i + 8) * 0x200 * 0x10, npc->count2 * 0x200 * 0x10, 0, 4);
					PlaySoundObject(12, SOUND_MODE_PLAY);
				}
			}

			if (++npc->act_wait == 81 || npc->act_wait == 241)
				SetNpChar(323, 48 * 0x200, 240 * 0x200, 0, 0, 1, NULL, 0x100);

			if (npc->act_wait == 1 || npc->act_wait == 161)
				SetNpChar(323, 272 * 0x200, 240 * 0x200, 0, 0, 1, NULL, 0x100);

			if (npc->act_wait >= 300)
			{
				npc->act_wait = 0;
				SetNpChar(325, npc->x, npc->y + (60 * 0x200), 0, 0, 0, NULL, 0x100);
			}

			break;

		case 500:
			gBoss[3].bits &= ~NPC_SHOOTABLE;

			npc->act_no = 501;
			npc->act_wait = 0;
			npc->count1 = 0;

			DeleteNpCharCode(325, TRUE);
			DeleteNpCharCode(330, TRUE);
			// Fallthrough
		case 501:
			if (++npc->act_wait % 0x10 == 0)
			{
				PlaySoundObject(12, SOUND_MODE_PLAY);
				SetDestroyNpChar(npc->x + (Random(-40, 40) * 0x200), npc->y + (Random(-60, 60) * 0x200), 1, 1);
			}

			if (npc->act_wait == 95)
				npc->ani_no = 1;
			if (npc->act_wait == 98)
				npc->ani_no = 2;

			if (npc->act_wait > 100)
				npc->act_no = 510;

			break;

		case 510:
			npc->ym += 0x40;
			npc->damage = 0x7F;
			npc->y += npc->ym;

			if (npc->count1 == 0 && npc->y > 160 * 0x200)
			{
				npc->count1 = 1;
				npc->ym = -0x200;
				npc->damage = 0;

				for (i = 0; i < 7; ++i)
				{
					ChangeMapParts(i + 7, 14, 0);
					SetDestroyNpChar((i + 7) * 0x200 * 0x10, 224 * 0x200, 0, 0);
					PlaySoundObject(12, SOUND_MODE_PLAY);
				}
			}

			if (npc->y > 480 * 0x200)
				npc->act_no = 520;

			break;
	}

	gBoss[1].x = npc->x - (24 * 0x200);
	gBoss[1].y = npc->y + (52 * 0x200);

	gBoss[2].x = npc->x + (24 * 0x200);
	gBoss[2].y = npc->y + (52 * 0x200);

	gBoss[3].x = npc->x;
	gBoss[3].y = npc->y + (40 * 0x200);

	RECT rc[3] = {
		{0, 0, 80, 120},
		{80, 0, 160, 120},
		{160, 0, 240, 120},
	};

	RECT rcDamage[3] = {
		{0, 120, 80, 240},
		{80, 120, 160, 240},
		{160, 120, 240, 240},
	};

	if (npc->shock != 0)
	{
		if (++flash / 2 % 2)
			npc->rect = rc[npc->ani_no];
		else
			npc->rect = rcDamage[npc->ani_no];
	}
	else
	{
		npc->rect = rc[npc->ani_no];
	}
}

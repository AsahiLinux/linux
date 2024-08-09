// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossIronH.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"

void ActBossChar_Ironhead(void)
{
	int i;
	NPCHAR *npc = gBoss;
	static unsigned char flash;

	switch (npc->act_no)
	{
		case 0:
			npc->cond = 0x80;
			npc->exp = 1;
			npc->direct = 2;
			npc->act_no = 100;
			npc->x = 160 * 0x200;
			npc->y = 128 * 0x200;
			npc->view.front = 40 * 0x200;
			npc->view.top = 12 * 0x200;
			npc->view.back = 24 * 0x200;
			npc->view.bottom = 12 * 0x200;
			npc->hit_voice = 54;
			npc->hit.front = 16 * 0x200;
			npc->hit.top = 10 * 0x200;
			npc->hit.back = 16 * 0x200;
			npc->hit.bottom = 10 * 0x200;
			npc->bits = (NPC_IGNORE_SOLIDITY | NPC_SHOOTABLE | NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			npc->size = 3;
			npc->damage = 10;
			npc->code_event = 1000;
			npc->life = 400;
			break;

		case 100:
			npc->act_no = 101;
			npc->bits &= ~NPC_SHOOTABLE;
			npc->act_wait = 0;
			// Fallthrough
		case 101:
			++npc->act_wait;

			if (npc->act_wait > 50)
			{
				npc->act_no = 250;
				npc->act_wait = 0;
			}

			if (npc->act_wait % 4 == 0)
				SetNpChar(197, Random(15, 18) * (16 * 0x200), Random(2, 13) * (16 * 0x200), 0, 0, 0, NULL, 0x100);

			break;

		case 250:
			npc->act_no = 251;

			if (npc->direct == 2)
			{
				npc->x = 240 * 0x200;
				npc->y = gMC.y;
			}
			else
			{
				npc->x = 720 * 0x200;
				npc->y = Random(2, 13) * (16 * 0x200);
			}

			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;

			npc->ym = Random(-0x200, 0x200);
			npc->xm = Random(-0x200, 0x200);

			npc->bits |= NPC_SHOOTABLE;
			// Fallthrough
		case 251:
			if (npc->direct == 2)
			{
				npc->tgt_x += 2 * 0x200;
			}
			else
			{
				npc->tgt_x -= 1 * 0x200;

				if (npc->tgt_y < gMC.y)
					npc->tgt_y += 1 * 0x200;
				else
					npc->tgt_y -= 1 * 0x200;
			}

			if (npc->x < npc->tgt_x)
				npc->xm += 8;
			else
				npc->xm -= 8;

			if (npc->y < npc->tgt_y)
				npc->ym += 8;
			else
				npc->ym -= 8;

			if (npc->ym > 0x200)
				npc->ym = 0x200;
			if (npc->ym < -0x200)
				npc->ym = -0x200;

			npc->x += npc->xm;
			npc->y += npc->ym;

			if (npc->direct == 2)
			{
				if (npc->x > 720 * 0x200)
				{
					npc->direct = 0;
					npc->act_no = 100;
				}
			}
			else
			{
				if (npc->x < 272 * 0x200)
				{
					npc->direct = 2;
					npc->act_no = 100;
				}
			}

			if (npc->direct == 0)
			{
				++npc->act_wait;

				if (npc->act_wait == 300 || npc->act_wait == 310 || npc->act_wait == 320)
				{
					PlaySoundObject(39, SOUND_MODE_PLAY);
					SetNpChar(198, npc->x + (10 * 0x200), npc->y + (1 * 0x200), Random(-3, 0) * 0x200, Random(-3, 3) * 0x200, 2, NULL, 0x100);
				}
			}

			++npc->ani_wait;

			if (npc->ani_wait > 2)
			{
				npc->ani_wait = 0;
				++npc->ani_no;
			}

			if (npc->ani_no > 7)
				npc->ani_no = 0;

			break;

		case 1000:
			npc->bits &= ~NPC_SHOOTABLE;
			npc->ani_no = 8;
			npc->damage = 0;
			npc->act_no = 1001;
			npc->tgt_x = npc->x;
			npc->tgt_y = npc->y;
			SetQuake(20);

			for (i = 0; i < 0x20; ++i)
				SetNpChar(4, npc->x + (Random(-128, 128) * 0x200), npc->y + (Random(-64, 64) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0x100);

			DeleteNpCharCode(197, TRUE);
			DeleteNpCharCode(271, TRUE);
			DeleteNpCharCode(272, TRUE);
			// Fallthrough
		case 1001:
			npc->tgt_x -= 1 * 0x200;

			npc->x = npc->tgt_x + (Random(-1, 1) * 0x200);
			npc->y = npc->tgt_y + (Random(-1, 1) * 0x200);

			if (++npc->act_wait % 4 == 0)
				SetNpChar(4, npc->x + (Random(-128, 128) * 0x200), npc->y + (Random(-64, 64) * 0x200), Random(-128, 128) * 0x200, Random(-128, 128) * 0x200, 0, NULL, 0x100);

			break;
	}

	RECT rc[9] = {
		{0, 0, 64, 24},
		{64, 0, 128, 24},
		{128, 0, 192, 24},
		{64, 0, 128, 24},
		{0, 0, 64, 24},
		{192, 0, 256, 24},
		{256, 0, 320, 24},
		{192, 0, 256, 24},
		{256, 48, 320, 72},
	};

	RECT rcDamage[9] = {
		{0, 24, 64, 48},
		{64, 24, 128, 48},
		{128, 24, 192, 48},
		{64, 24, 128, 48},
		{0, 24, 64, 48},
		{192, 24, 256, 48},
		{256, 24, 320, 48},
		{192, 24, 256, 48},
		{256, 48, 320, 72},
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

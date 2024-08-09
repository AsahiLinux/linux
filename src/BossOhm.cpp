// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossOhm.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "Bullet.h"
#include "Flash.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"

static void ActBoss01_12(void)
{
	int i;

	RECT rcLeft[1] = {
		{80, 56, 104, 72}
	};

	RECT rcRight[1] = {
		{104, 56, 128, 72}
	};

	for (i = 1; i < 3; ++i)
	{
		gBoss[i].y = (gBoss[0].y + gBoss[i + 2].y - (8 * 0x200)) / 2;

		if (gBoss[i].direct == 0)
		{
			gBoss[i].x = gBoss[0].x - (16 * 0x200);
			gBoss[i].rect = rcLeft[gBoss[i].ani_no];
		}
		else
		{
			gBoss[i].rect = rcRight[gBoss[i].ani_no];
			gBoss[i].x = gBoss[0].x + (16 * 0x200);
		}
	}
}

static void ActBoss01_34(void)
{
	int i;

	RECT rcLeft[2] = {
		{0, 56, 40, 88},
		{40, 56, 80, 88},
	};

	RECT rcRight[2] = {
		{0, 88, 40, 120},
		{40, 88, 80, 120},
	};

	for (i = 3; i < 5; ++i)
	{
		switch (gBoss[i].act_no)
		{
			case 0:
				gBoss[i].act_no = 1;
				// Fallthrough
			case 1:
				gBoss[i].y = gBoss[0].y;

				if (i == 3)
					gBoss[i].x = gBoss[0].x - (16 * 0x200);
				if (i == 4)
					gBoss[i].x = gBoss[0].x + (16 * 0x200);

				break;

			case 3:
				gBoss[i].tgt_y = gBoss[0].y + (24 * 0x200);

				if (i == 3)
					gBoss[i].x = gBoss[0].x - (16 * 0x200);
				if (i == 4)
					gBoss[i].x = gBoss[0].x + (16 * 0x200);

				gBoss[i].y += (gBoss[i].tgt_y - gBoss[i].y) / 2;
				break;
		}

		if (gBoss[i].flag & 8 || gBoss[i].y <= gBoss[i].tgt_y)
			gBoss[i].ani_no = 0;
		else
			gBoss[i].ani_no = 1;

		if (gBoss[i].direct == 0)
			gBoss[i].rect = rcLeft[gBoss[i].ani_no];
		else
			gBoss[i].rect = rcRight[gBoss[i].ani_no];
	}
}

static void ActBoss01_5(void)
{
	switch (gBoss[5].act_no)
	{
		case 0:
			gBoss[5].bits |= (NPC_SOLID_SOFT | NPC_IGNORE_SOLIDITY);

			gBoss[5].hit.front = 20 * 0x200;
			gBoss[5].hit.top = 36 * 0x200;
			gBoss[5].hit.back = 20 * 0x200;
			gBoss[5].hit.bottom = 16 * 0x200;

			gBoss[5].act_no = 1;
			// Fallthrough
		case 1:
			gBoss[5].x = gBoss[0].x;
			gBoss[5].y = gBoss[0].y;
			break;
	}
}

void ActBossChar_Omega(void)
{
	switch (gBoss[0].act_no)
	{
		case 0:
			gBoss[0].x = 219 * 0x10 * 0x200;
			gBoss[0].y = 16 * 0x10 * 0x200;

			gBoss[0].view.front = 40 * 0x200;
			gBoss[0].view.top = 40 * 0x200;
			gBoss[0].view.back = 40 * 0x200;
			gBoss[0].view.bottom = 16 * 0x200;

			gBoss[0].tgt_x = gBoss[0].x;
			gBoss[0].tgt_y = gBoss[0].y;

			gBoss[0].hit_voice = 52;

			gBoss[0].hit.front = 8 * 0x200;
			gBoss[0].hit.top = 24 * 0x200;
			gBoss[0].hit.back = 8 * 0x200;
			gBoss[0].hit.bottom = 16 * 0x200;

			gBoss[0].bits = (NPC_IGNORE_SOLIDITY | NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			gBoss[0].size = 3;
			gBoss[0].exp = 1;
			gBoss[0].code_event = 210;
			gBoss[0].life = 400;

			gBoss[1].cond = 0x80;

			gBoss[1].view.front = 12 * 0x200;
			gBoss[1].view.top = 8 * 0x200;
			gBoss[1].view.back = 12 * 0x200;
			gBoss[1].view.bottom = 8 * 0x200;

			gBoss[1].bits = NPC_IGNORE_SOLIDITY;

			gBoss[2] = gBoss[1];

			gBoss[1].direct = 0;
			gBoss[2].direct = 2;

			gBoss[3].cond = 0x80;

			gBoss[3].view.front = 24 * 0x200;
			gBoss[3].view.top = 16 * 0x200;
			gBoss[3].view.back = 16 * 0x200;
			gBoss[3].view.bottom = 16 * 0x200;

			gBoss[3].hit_voice = 52;

			gBoss[3].hit.front = 8 * 0x200;
			gBoss[3].hit.top = 8 * 0x200;
			gBoss[3].hit.back = 8 * 0x200;
			gBoss[3].hit.bottom = 8 * 0x200;

			gBoss[3].bits = NPC_IGNORE_SOLIDITY;

			gBoss[3].x = gBoss[0].x - (16 * 0x200);
			gBoss[3].y = gBoss[0].y;
			gBoss[3].direct = 0;

			gBoss[4] = gBoss[3];

			gBoss[4].direct = 2;
			gBoss[3].x = gBoss[0].x + (16 * 0x200);
			gBoss[5].cond = 0x80;
			break;

		case 20: // Rising out of the ground
			gBoss[0].act_no = 30;
			gBoss[0].act_wait = 0;
			gBoss[0].ani_no = 0;
			// Fallthrough
		case 30:
			SetQuake(2);
			gBoss[0].y -= 1 * 0x200;

			if (++gBoss[0].act_wait % 4 == 0)
				PlaySoundObject(26, SOUND_MODE_PLAY);

			if (gBoss[0].act_wait == 48)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 40;

				if (gBoss[0].life > 280)
					break;

				gBoss[0].act_no = 110;

				gBoss[0].bits |= NPC_SHOOTABLE;
				gBoss[0].bits &= ~NPC_IGNORE_SOLIDITY;
				gBoss[3].bits &= ~NPC_IGNORE_SOLIDITY;
				gBoss[4].bits &= ~NPC_IGNORE_SOLIDITY;

				gBoss[3].act_no = 3;
				gBoss[4].act_no = 3;
				gBoss[5].hit.top = 16 * 0x200;
			}

			break;

		case 40:
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait == 48)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 50;
				gBoss[0].count1 = 0;
				gBoss[5].hit.top = 16 * 0x200;
				PlaySoundObject(102, SOUND_MODE_PLAY);
			}

			break;

		case 50: // Open mouth
			++gBoss[0].count1;

			if (gBoss[0].count1 > 2)
			{
				gBoss[0].count1 = 0;
				++gBoss[0].count2;
			}

			if (gBoss[0].count2 == 3)
			{
				gBoss[0].act_no = 60;
				gBoss[0].act_wait = 0;
				gBoss[0].bits |= NPC_SHOOTABLE;
				gBoss[0].hit.front = 16 * 0x200;
				gBoss[0].hit.back = 16 * 0x200;
			}

			break;

		case 60: // Shoot out of mouth
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait > 20 && gBoss[0].act_wait < 80 && !(gBoss[0].act_wait % 3))
			{
				if (Random(0, 9) < 8)
					SetNpChar(48, gBoss[0].x, gBoss[0].y - (16 * 0x200), Random(-0x100, 0x100), -0x10 * 0x200 / 10, 0, NULL, 0x100);
				else
					SetNpChar(48, gBoss[0].x, gBoss[0].y - (16 * 0x200), Random(-0x100, 0x100), -0x10 * 0x200 / 10, 2, NULL, 0x100);

				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			if (gBoss[0].act_wait == 200 || CountArmsBullet(6))
			{
				gBoss[0].count1 = 0;
				gBoss[0].act_no = 70;
				PlaySoundObject(102, SOUND_MODE_PLAY);
			}

			break;

		case 70: // Close mouth
			++gBoss[0].count1;

			if (gBoss[0].count1 > 2)
			{
				gBoss[0].count1 = 0;
				--gBoss[0].count2;
			}

			if (gBoss[0].count2 == 1)
				gBoss[0].damage = 20;

			if (gBoss[0].count2 == 0)
			{
				PlaySoundObject(102, SOUND_MODE_STOP);
				PlaySoundObject(12, SOUND_MODE_PLAY);

				gBoss[0].act_no = 80;
				gBoss[0].act_wait = 0;

				gBoss[0].bits &= ~NPC_SHOOTABLE;

				gBoss[0].hit.front = 24 * 0x200;
				gBoss[0].hit.back = 24 * 0x200;
				gBoss[5].hit.top = 36 * 0x200;

				gBoss[0].damage = 0;
			}

			break;

		case 80:
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait == 48)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 90;
			}

			break;

		case 90: // Go back into the ground
			SetQuake(2);
			gBoss[0].y += 1 * 0x200;

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait % 4 == 0)
				PlaySoundObject(26, SOUND_MODE_PLAY);

			if (gBoss[0].act_wait == 48)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 100;
			}

			break;

		case 100: // Move to proper position for coming out of the ground
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait == 120)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 30;

				gBoss[0].x = gBoss[0].tgt_x + (Random(-64, 64) * 0x200);
				gBoss[0].y = gBoss[0].tgt_y;
			}

			break;

		case 110:
			++gBoss[0].count1;

			if (gBoss[0].count1 > 2)
			{
				gBoss[0].count1 = 0;
				++gBoss[0].count2;
			}

			if (gBoss[0].count2 == 3)
			{
				gBoss[0].act_no = 120;
				gBoss[0].act_wait = 0;
				gBoss[0].hit.front = 16 * 0x200;
				gBoss[0].hit.back = 16 * 0x200;
			}

			break;

		case 120:
			++gBoss[0].act_wait;

			if (gBoss[0].act_wait == 50 || CountArmsBullet(6))
			{
				gBoss[0].act_no = 130;
				PlaySoundObject(102, SOUND_MODE_PLAY);
				gBoss[0].act_wait = 0;
				gBoss[0].count1 = 0;
			}

			if (gBoss[0].act_wait < 30 && gBoss[0].act_wait % 5 == 0)
			{
				SetNpChar(48, gBoss[0].x, gBoss[0].y - (16 * 0x200), Random(-341, 341), -0x10 * 0x200 / 10, 0, NULL, 0x100);
				PlaySoundObject(39, SOUND_MODE_PLAY);
			}

			break;

		case 130:
			++gBoss[0].count1;

			if (gBoss[0].count1 > 2)
			{
				gBoss[0].count1 = 0;
				--gBoss[0].count2;
			}

			if (gBoss[0].count2 == 1)
				gBoss[0].damage = 20;

			if (gBoss[0].count2 == 0)
			{
				gBoss[0].act_no = 140;
				gBoss[0].bits |= NPC_SHOOTABLE;

				gBoss[0].hit.front = 16 * 0x200;
				gBoss[0].hit.back = 16 * 0x200;

				gBoss[0].ym = -0x5FF;

				PlaySoundObject(102, SOUND_MODE_STOP);
				PlaySoundObject(12, SOUND_MODE_PLAY);
				PlaySoundObject(25, SOUND_MODE_PLAY);

				if (gBoss[0].x < gMC.x)
					gBoss[0].xm = 0x100;
				if (gBoss[0].x > gMC.x)
					gBoss[0].xm = -0x100;

				gBoss[0].damage = 0;
				gBoss[5].hit.top = 36 * 0x200;
			}

			break;

		case 140:
			if (gMC.flag & 8 && gBoss[0].ym > 0)
				gBoss[5].damage = 20;
			else
				gBoss[5].damage = 0;

			gBoss[0].ym += 0x24;
			if (gBoss[0].ym > 0x5FF)
				gBoss[0].ym = 0x5FF;

			gBoss[0].x += gBoss[0].xm;
			gBoss[0].y += gBoss[0].ym;

			if (gBoss[0].flag & 8)
			{
				gBoss[0].act_no = 110;
				gBoss[0].act_wait = 0;
				gBoss[0].count1 = 0;

				gBoss[5].hit.top = 16 * 0x200;
				gBoss[5].damage = 0;

				PlaySoundObject(26, SOUND_MODE_PLAY);
				PlaySoundObject(12, SOUND_MODE_PLAY);

				SetQuake(30);
			}

			break;

		case 150:
			SetQuake(2);

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait % 12 == 0)
				PlaySoundObject(52, SOUND_MODE_PLAY);

			SetDestroyNpChar(gBoss[0].x + (Random(-0x30, 0x30) * 0x200), gBoss[0].y + (Random(-0x30, 0x18) * 0x200), 1, 1);

			if (gBoss[0].act_wait > 100)
			{
				gBoss[0].act_wait = 0;
				gBoss[0].act_no = 160;
				SetFlash(gBoss[0].x, gBoss[0].y, FLASH_MODE_EXPLOSION);
				PlaySoundObject(35, SOUND_MODE_PLAY);
			}

			break;

		case 160:
			SetQuake(40);

			++gBoss[0].act_wait;

			if (gBoss[0].act_wait > 50)
			{
				gBoss[0].cond = 0;
				gBoss[1].cond = 0;
				gBoss[2].cond = 0;
				gBoss[3].cond = 0;
				gBoss[4].cond = 0;
				gBoss[5].cond = 0;
			}

			break;
	}

	RECT rect[4] = {
		{0, 0, 80, 56},
		{80, 0, 160, 56},
		{160, 0, 240, 56},
		{80, 0, 160, 56},
	};

	gBoss[0].rect = rect[gBoss[0].count2];

	gBoss[1].shock = gBoss[0].shock;
	gBoss[2].shock = gBoss[0].shock;
	gBoss[3].shock = gBoss[0].shock;
	gBoss[4].shock = gBoss[0].shock;

	ActBoss01_34();
	ActBoss01_12();
	ActBoss01_5();

	if (gBoss[0].life == 0 && gBoss[0].act_no < 150)
	{
		gBoss[0].act_no = 150;
		gBoss[0].act_wait = 0;
		gBoss[0].damage = 0;
		gBoss[5].damage = 0;
		DeleteNpCharCode(48, TRUE);
	}
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Boss.h"

#include <string.h>

#include "WindowsWrapper.h"

#include "BossAlmo1.h"
#include "BossAlmo2.h"
#include "BossBallos.h"
#include "BossFrog.h"
#include "BossIronH.h"
#include "BossOhm.h"
#include "BossPress.h"
#include "BossTwinD.h"
#include "BossX.h"
#include "Bullet.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Draw.h"
#include "Map.h"
#include "MyChar.h"
#include "NpChar.h"
#include "NpcHit.h"
#include "Sound.h"
#include "TextScr.h"
#include "ValueView.h"

NPCHAR gBoss[BOSS_MAX];

void InitBossChar(int code)
{
	memset(gBoss, 0, sizeof(gBoss));
	gBoss[0].cond = 0x80;
	gBoss[0].code_char = code;
}

void PutBossChar(int fx, int fy)
{
	signed char a = 0;
	int b;

	int side;

	for (b = BOSS_MAX - 1; b >= 0; --b)
	{
		if (gBoss[b].cond & 0x80)
		{
			if (gBoss[b].shock != 0)
			{
				a = ((gBoss[b].shock / 2 % 2) * 2) - 1;
			}
			else
			{
				a = 0;

				if (gBoss[b].bits & NPC_SHOW_DAMAGE && gBoss[b].damage_view != 0)
				{
					SetValueView(&gBoss[b].x, &gBoss[b].y, gBoss[b].damage_view);
					gBoss[b].damage_view = 0;
				}
			}

			if (gBoss[b].direct == 0)
				side = gBoss[b].view.front;
			else
				side = gBoss[b].view.back;

			PutBitmap3(
				&grcGame,
				((gBoss[b].x - side) / 0x200) - (fx / 0x200) + a,
				((gBoss[b].y - gBoss[b].view.top) / 0x200) - (fy / 0x200),
				&gBoss[b].rect,
				SURFACE_ID_LEVEL_SPRITESET_2);
		}
	}
}

void SetBossCharActNo(int a)
{
	gBoss[0].act_no = a;
}

void HitBossBullet(void)
{
	BOOL bHit;
	int bul;
	int bos;
	int bos_;

	for (bos = 0; bos < BOSS_MAX; ++bos)
	{
		if (!(gBoss[bos].cond & 0x80))
			continue;

		for (bul = 0; bul < BULLET_MAX; ++bul)
		{
			if (!(gBul[bul].cond & 0x80))
				continue;

			if (gBul[bul].damage == -1)
				continue;

			// Check if bullet touches boss
			bHit = FALSE;
			if (gBoss[bos].bits & NPC_SHOOTABLE
				&& gBoss[bos].x - gBoss[bos].hit.back < gBul[bul].x + gBul[bul].enemyXL
				&& gBoss[bos].x + gBoss[bos].hit.back > gBul[bul].x - gBul[bul].enemyXL
				&& gBoss[bos].y - gBoss[bos].hit.top < gBul[bul].y + gBul[bul].enemyYL
				&& gBoss[bos].y + gBoss[bos].hit.bottom > gBul[bul].y - gBul[bul].enemyYL)
				bHit = TRUE;
			else if (gBoss[bos].bits & NPC_INVULNERABLE
				&& gBoss[bos].x - gBoss[bos].hit.back < gBul[bul].x + gBul[bul].blockXL
				&& gBoss[bos].x + gBoss[bos].hit.back > gBul[bul].x - gBul[bul].blockXL
				&& gBoss[bos].y - gBoss[bos].hit.top < gBul[bul].y + gBul[bul].blockYL
				&& gBoss[bos].y + gBoss[bos].hit.bottom > gBul[bul].y - gBul[bul].blockYL)
				bHit = TRUE;

			if (bHit)
			{
				// Damage boss
				if (gBoss[bos].bits & NPC_SHOOTABLE)
				{
					if (gBoss[bos].cond & 0x10)
						bos_ = 0;
					else
						bos_ = bos;

					gBoss[bos_].life -= gBul[bul].damage;

					if (gBoss[bos_].life < 1)
					{
						gBoss[bos_].life = bos_;

						if (gMC.cond & 0x80 && gBoss[bos_].bits & NPC_EVENT_WHEN_KILLED)
						{
							StartTextScript(gBoss[bos_].code_event);
						}
						else
						{
							PlaySoundObject(gBoss[bos_].destroy_voice, SOUND_MODE_PLAY);

							switch (gBoss[bos_].size)
							{
								case 1:
									SetDestroyNpChar(gBoss[bos_].x, gBoss[bos_].y, gBoss[bos_].view.back, 4);
									break;
								case 2:
									SetDestroyNpChar(gBoss[bos_].x, gBoss[bos_].y, gBoss[bos_].view.back, 8);
									break;
								case 3:
									SetDestroyNpChar(gBoss[bos_].x, gBoss[bos_].y, gBoss[bos_].view.back, 16);
									break;
							}

							gBoss[bos_].cond = 0;
						}
					}
					else
					{
						if (gBoss[bos].shock < 14)
						{
							SetCaret(gBul[bul].x, gBul[bul].y, CARET_HURT_PARTICLES, DIR_LEFT);
							SetCaret(gBul[bul].x, gBul[bul].y, CARET_HURT_PARTICLES, DIR_LEFT);
							SetCaret(gBul[bul].x, gBul[bul].y, CARET_HURT_PARTICLES, DIR_LEFT);
							PlaySoundObject(gBoss[bos_].hit_voice, SOUND_MODE_PLAY);
						}

						gBoss[bos].shock = 8;
						gBoss[bos_].shock = 8;
						gBoss[bos_].damage_view -= gBul[bul].damage;
					}

					--gBul[bul].life;

					if (gBul[bul].life < 1)
						gBul[bul].cond = 0;
				}
				else if (gBul[bul].code_bullet == 13
					|| gBul[bul].code_bullet == 14
					|| gBul[bul].code_bullet == 15
					|| gBul[bul].code_bullet == 28
					|| gBul[bul].code_bullet == 29
					|| gBul[bul].code_bullet == 30)
				{
					--gBul[bul].life;
				}
				else
				{
					if (!(gBul[bul].bbits & 0x10))
					{
						SetCaret(gBul[bul].x, gBul[bul].y, CARET_PROJECTILE_DISSIPATION, DIR_RIGHT);
						PlaySoundObject(31, SOUND_MODE_PLAY);
						gBul[bul].cond = 0;
						continue;
					}
				}
			}
		}
	}
}

void ActBossChar_0(void)
{
	
}

BOSSFUNCTION gpBossFuncTbl[10] =
{
	ActBossChar_0,
	ActBossChar_Omega,
	ActBossChar_Frog,
	ActBossChar_MonstX,
	ActBossChar_Core,
	ActBossChar_Ironhead,
	ActBossChar_Twin,
	ActBossChar_Undead,
	ActBossChar_Press,
	ActBossChar_Ballos
};

void ActBossChar(void)
{
	int code_char;
	int bos;

	if (!(gBoss[0].cond & 0x80))
		return;

	code_char = gBoss[0].code_char;

	gpBossFuncTbl[code_char]();

	for (bos = 0; bos < BOSS_MAX; ++bos)
		if (gBoss[bos].shock)
			--gBoss[bos].shock;
}

void HitBossMap(void)
{
	int x, y;
	unsigned char atrb[16];
	int judg;
	int offx[16];
	int offy[16];
	int b, j;

	offx[0] = 0;
	offx[1] = 1;
	offx[2] = 0;
	offx[3] = 1;
	offx[4] = 2;
	offx[5] = 2;
	offx[6] = 2;
	offx[7] = 0;
	offx[8] = 1;
	offx[9] = -1;
	offx[10] = -1;
	offx[11] = -1;
	offx[12] = -1;
	offx[13] = 0;
	offx[14] = 1;
	offx[15] = 2;

	offy[0] = 0;
	offy[1] = 0;
	offy[2] = 1;
	offy[3] = 1;
	offy[4] = 0;
	offy[5] = 1;
	offy[6] = 2;
	offy[7] = 2;
	offy[8] = 2;
	offy[9] = -1;
	offy[10] = 0;
	offy[11] = 1;
	offy[12] = 2;
	offy[13] = -1;
	offy[14] = -1;
	offy[15] = -1;

	for (b = 0; b < BOSS_MAX; ++b)
	{
		if (!(gBoss[b].cond & 0x80))
			continue;

		if (gBoss[b].bits & NPC_IGNORE_SOLIDITY)
			continue;

		if (gBoss[b].size >= 3)
		{
			judg = 16;
			x = gBoss[b].x / 0x10 / 0x200;
			y = gBoss[b].y / 0x10 / 0x200;
		}
		else
		{
			judg = 4;
			x = gBoss[b].x / 0x10 / 0x200;
			y = gBoss[b].y / 0x10 / 0x200;
		}

		gBoss[b].flag = 0;
		for (j = 0; j < judg; ++j)
		{
			atrb[j] = GetAttribute(x + offx[j], y + offy[j]);

			switch (atrb[j])
			{
				case 0x44:
					if (gBoss[b].bits & NPC_IGNORE_TILE_44)
						break;
					// Fallthrough
				case 0x05:
				case 0x41:
				case 0x43:
					JadgeHitNpCharBlock(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x50:
					JudgeHitNpCharTriangleA(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x51:
					JudgeHitNpCharTriangleB(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x52:
					JudgeHitNpCharTriangleC(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x53:
					JudgeHitNpCharTriangleD(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x54:
					JudgeHitNpCharTriangleE(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x55:
					JudgeHitNpCharTriangleF(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x56:
					JudgeHitNpCharTriangleG(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x57:
					JudgeHitNpCharTriangleH(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x02:
				case 0x60:
				case 0x61:
				case 0x64:
					JadgeHitNpCharBlock(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x70:
					JudgeHitNpCharTriangleA(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x71:
					JudgeHitNpCharTriangleB(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x72:
					JudgeHitNpCharTriangleC(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x73:
					JudgeHitNpCharTriangleD(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x74:
					JudgeHitNpCharTriangleE(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x75:
					JudgeHitNpCharTriangleF(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x76:
					JudgeHitNpCharTriangleG(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
				case 0x77:
					JudgeHitNpCharTriangleH(&gBoss[b], x + offx[j], y + offy[j]);
					JudgeHitNpCharWater(&gBoss[b], x + offx[j], y + offy[j]);
					break;
			}
		}
	}
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossFrog.h"

#include <stddef.h>

#include "WindowsWrapper.h"

#include "Boss.h"
#include "CommonDefines.h"
#include "Frame.h"
#include "Game.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"
#include "Triangle.h"

enum BalfrogSprites
{
	BALFROG_SPRITE_NOTHING = 0,
	BALFROG_SPRITE_STANDING_STILL = 1,
	BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING = 2,
	BALFROG_SPRITE_MOUTH_OPEN_CROUCHING = 3,
	BALFROG_SPRITE_MOUTH_OPEN_CROUCHING_FLASHING = 4,
	BALFROG_SPRITE_JUMPING = 5,
	BALFROG_SPRITE_BALROG_WHITE = 6,
	BALFROG_SPRITE_BALROG_CROUCHING = 7,
	BALFROG_SPRITE_BALROG_JUMPING = 8
};

enum BalfrogStates
{
	BALFROG_INITIALIZE = 0,
	BALFROG_START = 10,
	BALFROG_INITIALIZE_FLICKER = 20,
	BALFROG_FLICKER = 21,
	BALFROG_WAIT = 100,
	BALFROG_INITIALIZE_HOP_1 = 101,
	BALFROG_INITIALIZE_HOP_2 = 102,
	BALFROG_HOP = 103,
	BALFROG_MIDAIR = 104,
	BALFROG_INITIALIZE_LAND = 110,
	BALFROG_LAND = 111,
	BALFROG_INITIALIZE_SHOOT = 112,
	BALFROG_SHOOT = 113,
	BALFROG_AFTER_SHOOT_WAIT = 114,
	BALFROG_INITIALIZE_LEAP_1 = 120,
	BALFROG_INITIALIZE_LEAP_2 = 121,
	BALFROG_INITIALIZE_LEAP_3 = 122,
	BALFROG_LEAP = 123,
	BALFROG_LEAP_MIDAIR = 124,
	BALFROG_DIE = 130,
	BALFROG_DIE_FLASHING = 131,
	BALFROG_REVERT = 132,
	BALFROG_NOP_START = 140,
	BALFROG_NOP = 141,
	BALFROG_GO_INTO_CEILING = 142,
	BALFROG_GONE_INTO_CEILING = 143
};

// Balfrog's mouth
static void ActBossChar02_01(void)
{
	NPCHAR *boss;
	int minus;

	if (gBoss[0].direct == DIR_LEFT)
		minus = 1;
	else
		minus = -1;

	boss = &gBoss[1];

	switch (gBoss[0].ani_no)
	{
		case BALFROG_SPRITE_NOTHING:
			boss->hit_voice = SND_BEHEMOTH_LARGE_HURT;
			boss->hit.front = 16 * 0x200;
			boss->hit.top = 16 * 0x200;
			boss->hit.back = 16 * 0x200;
			boss->hit.bottom = 16 * 0x200;
			boss->size = 3;
			boss->bits = NPC_INVULNERABLE;
			break;

		case BALFROG_SPRITE_STANDING_STILL:
			boss->x = gBoss[0].x + -24 * 0x200 * minus;
			boss->y = gBoss[0].y - 24 * 0x200;
			break;

		case BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING:
			boss->x = gBoss[0].x + -24 * 0x200 * minus;
			boss->y = gBoss[0].y - 20 * 0x200;
			break;

		case BALFROG_SPRITE_MOUTH_OPEN_CROUCHING:
		case BALFROG_SPRITE_MOUTH_OPEN_CROUCHING_FLASHING:
			boss->x = gBoss[0].x + -24 * 0x200 * minus;
			boss->y = gBoss[0].y - 16 * 0x200;
			break;

		case BALFROG_SPRITE_JUMPING:
			boss->x = gBoss[0].x + -24 * 0x200 * minus;
			boss->y = gBoss[0].y - 43 * 0x200;
			break;
	}
}

static void ActBossChar02_02(void)
{
	NPCHAR *boss = &gBoss[2];

	switch (gBoss[0].ani_no)
	{
		case BALFROG_SPRITE_NOTHING:
			boss->hit_voice = SND_BEHEMOTH_LARGE_HURT;
			boss->hit.front = 24 * 0x200;
			boss->hit.top = 16 * 0x200;
			boss->hit.back = 24 * 0x200;
			boss->hit.bottom = 16 * 0x200;
			boss->size = 3;
			boss->bits = NPC_INVULNERABLE;
			break;

		case BALFROG_SPRITE_STANDING_STILL:
		case BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING:
		case BALFROG_SPRITE_MOUTH_OPEN_CROUCHING:
		case BALFROG_SPRITE_MOUTH_OPEN_CROUCHING_FLASHING:
		case BALFROG_SPRITE_JUMPING:
			boss->x = gBoss[0].x;
			boss->y = gBoss[0].y;
			break;
	}
}

// Main boss AI
void ActBossChar_Frog(void)
{
	unsigned char deg;
	int xm, ym;
	int i;

	// Rects 1-4 are for when Balfrog is a frog, 5-8 for when he reverts into Balrog and goes into the ceiling
	RECT rcLeft[9] = {
		{0, 0, 0, 0},           // Nothing
		{0, 48, 80, 112},       // Balfrog standing still
		{0, 112, 80, 176},      // Balfrog with his mouth barely open, crouching
		{0, 176, 80, 240},      // Balfrog with his mouth open, crouching
		{160, 48, 240, 112},    // Balfrog with his mouth open, crouching, flashing
		{160, 112, 240, 200},   // Balfrog jumping
		{200, 0, 240, 24},      // Balrog completely white
		{80, 0, 120, 24},       // Balrog crouching
		{120, 0, 160, 24},      // Balrog jumping
	};

	// See above
	RECT rcRight[9] = {
		{0, 0, 0, 0},
		{80, 48, 160, 112},
		{80, 112, 160, 176},
		{80, 176, 160, 240},
		{240, 48, 320, 112},
		{240, 112, 320, 200},
		{200, 24, 240, 48},
		{80, 24, 120, 48},
		{120, 24, 160, 48},
	};

	NPCHAR *boss = gBoss;

	switch (boss->act_no)
	{
		case BALFROG_INITIALIZE:
			boss->x = 6 * (0x200 * 0x10);
			boss->y = 12 * (0x200 * 0x10) + 8 * 0x200;
			boss->direct = DIR_RIGHT;
			boss->view.front = 48 * 0x200;
			boss->view.top = 48 * 0x200;
			boss->view.back = 32 * 0x200;
			boss->view.bottom = 16 * 0x200;
			boss->hit_voice = SND_BEHEMOTH_LARGE_HURT;
			boss->hit.front = 24 * 0x200;
			boss->hit.top = 16 * 0x200;
			boss->hit.back = 24 * 0x200;
			boss->hit.bottom = 16 * 0x200;
			boss->size = 3;
			boss->exp = 1;
			boss->code_event = 1000;
			boss->bits |= (NPC_EVENT_WHEN_KILLED | NPC_SHOW_DAMAGE);
			boss->life = 300;
			break;

		case BALFROG_START:
			boss->act_no = (BALFROG_START + 1);
			boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			boss->cond = NPCCOND_ALIVE;
			boss->rect = rcRight[0];

			gBoss[1].cond = (NPCCOND_ALIVE | NPCCOND_DAMAGE_BOSS);
			gBoss[1].code_event = 1000;
			gBoss[2].cond = NPCCOND_ALIVE;

			gBoss[1].damage = 5;
			gBoss[2].damage = 5;

			for (i = 0; i < 8; ++i)
				SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + Random(-12, 12) * 0x200, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);

			break;

		case BALFROG_INITIALIZE_FLICKER:
			boss->act_no = BALFROG_FLICKER;
			boss->act_wait = 0;
			// Fallthrough
		case BALFROG_FLICKER:
			++boss->act_wait;

			if (boss->act_wait / 2 % 2)
				boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			else
				boss->ani_no = BALFROG_SPRITE_NOTHING;

			break;

		case BALFROG_WAIT:
			boss->act_no = BALFROG_INITIALIZE_HOP_1;
			boss->act_wait = 0;
			boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			boss->xm = 0;
			// Fallthrough
		case BALFROG_INITIALIZE_HOP_1:
			++boss->act_wait;

			if (boss->act_wait > 50)
			{
				boss->act_no = BALFROG_INITIALIZE_HOP_2;
				boss->ani_wait = 0;
				boss->ani_no = BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING;
			}

			break;

		case BALFROG_INITIALIZE_HOP_2:
			++boss->ani_wait;

			if (boss->ani_wait > 10)
			{
				boss->act_no = BALFROG_HOP;
				boss->ani_wait = 0;
				boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			}

			break;

		case BALFROG_HOP:
			++boss->ani_wait;

			if (boss->ani_wait > 4)
			{
				boss->act_no = BALFROG_MIDAIR;
				boss->ani_no = BALFROG_SPRITE_JUMPING;
				boss->ym = -2 * 0x200;
				PlaySoundObject(25, SOUND_MODE_PLAY);

				if (boss->direct == DIR_LEFT)
					boss->xm = -1 * 0x200;
				else
					boss->xm = 1 * 0x200;

				boss->view.top = 64 * 0x200;
				boss->view.bottom = 24 * 0x200;
			}

			break;

		case BALFROG_MIDAIR:
			if (boss->direct == DIR_LEFT && boss->flag & COLL_LEFT_WALL)
			{
				boss->direct = DIR_RIGHT;
				boss->xm = 1 * 0x200;
			}

			if (boss->direct == DIR_RIGHT && boss->flag & COLL_RIGHT_WALL)
			{
				boss->direct = DIR_LEFT;
				boss->xm = -1 * 0x200;
			}

			if (boss->flag & COLL_GROUND)
			{
				PlaySoundObject(SND_LARGE_OBJECT_HIT_GROUND, SOUND_MODE_PLAY);
				SetQuake(30);
				boss->act_no = BALFROG_WAIT;
				boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
				boss->view.top = 48 * 0x200;
				boss->view.bottom = 16 * 0x200;

				if (boss->direct == DIR_LEFT && boss->x < gMC.x)
				{
					boss->direct = DIR_RIGHT;
					boss->act_no = BALFROG_INITIALIZE_LAND;
				}

				if (boss->direct == DIR_RIGHT && boss->x > gMC.x)
				{
					boss->direct = DIR_LEFT;
					boss->act_no = BALFROG_INITIALIZE_LAND;
				}

				SetNpChar(110, Random(4, 16) * (0x200 * 0x10), Random(0, 4) * (0x200 * 0x10), 0, 0, DIR_AUTO, NULL, 0x80);

				for (i = 0; i < 4; ++i)
					SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + boss->hit.bottom, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);
			}

			break;

		case BALFROG_INITIALIZE_LAND:
			boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			boss->act_wait = 0;
			boss->act_no = BALFROG_LAND;
			// Fallthrough
		case BALFROG_LAND:
			++boss->act_wait;

			boss->xm = (boss->xm * 8) / 9;

			if (boss->act_wait > 50)
			{
				boss->ani_no = BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING;
				boss->ani_wait = 0;
				boss->act_no = BALFROG_INITIALIZE_SHOOT;
			}

			break;

		case BALFROG_INITIALIZE_SHOOT:
			++boss->ani_wait;

			if (boss->ani_wait > 4)
			{
				boss->act_no = BALFROG_SHOOT;
				boss->act_wait = 0;
				boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
				boss->count1 = 16;
				gBoss[1].bits |= NPC_SHOOTABLE;
				boss->tgt_x = boss->life;
			}

			break;

		case BALFROG_SHOOT:
			if (boss->shock != 0)
			{
				if (boss->count2++ / 2 % 2)
					boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING_FLASHING;
				else
					boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			}
			else
			{
				boss->count2 = 0;
				boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			}

			boss->xm = (boss->xm * 10) / 11;

			++boss->act_wait;

			if (boss->act_wait > 16)
			{
				boss->act_wait = 0;
				--boss->count1;

				if (boss->direct == DIR_LEFT)
					deg = GetArktan(boss->x - 2 * (0x200 * 0x10) - gMC.x, boss->y - 8 * 0x200 - gMC.y);
				else
					deg = GetArktan(boss->x + 2 * (0x200 * 0x10) - gMC.x, boss->y - 8 * 0x200 - gMC.y);

				deg += (unsigned char)Random(-0x10, 0x10);

				ym = GetSin(deg);
				xm = GetCos(deg);

				if (boss->direct == DIR_LEFT)
					SetNpChar(NPC_PROJECTILE_BALFROG_SPITBALL, boss->x - 2 * (0x200 * 0x10), boss->y - 8 * 0x200, xm, ym, DIR_LEFT, NULL, 0x100);
				else
					SetNpChar(NPC_PROJECTILE_BALFROG_SPITBALL, boss->x + 2 * (0x200 * 0x10), boss->y - 8 * 0x200, xm, ym, DIR_LEFT, NULL, 0x100);

				PlaySoundObject(SND_ENEMY_SHOOT_PROJECTILE, SOUND_MODE_PLAY);

				if (boss->count1 == 0 || boss->life < boss->tgt_x - 90)
				{
					boss->act_no = BALFROG_AFTER_SHOOT_WAIT;
					boss->act_wait = 0;
					boss->ani_no = BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING;
					boss->ani_wait = 0;
					gBoss[1].bits &= ~NPC_SHOOTABLE;
				}
			}

			break;

		case BALFROG_AFTER_SHOOT_WAIT:
			++boss->ani_wait;

			if (boss->ani_wait > 10)
			{
				if (++gBoss[1].count1 > 2)
				{
					gBoss[1].count1 = 0;
					boss->act_no = BALFROG_INITIALIZE_LEAP_1;
				}
				else
				{
					boss->act_no = BALFROG_WAIT;
				}

				boss->ani_wait = 0;
				boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			}

			break;

		case BALFROG_INITIALIZE_LEAP_1:
			boss->act_no = BALFROG_INITIALIZE_LEAP_2;
			boss->act_wait = 0;
			boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			boss->xm = 0;
			// Fallthrough
		case BALFROG_INITIALIZE_LEAP_2:
			++boss->act_wait;

			if (boss->act_wait > 50)
			{
				boss->act_no = BALFROG_INITIALIZE_LEAP_3;
				boss->ani_wait = 0;
				boss->ani_no = BALFROG_SPRITE_MOUTH_BARELY_OPEN_CROUCHING;
			}

			break;

		case BALFROG_INITIALIZE_LEAP_3:
			++boss->ani_wait;

			if (boss->ani_wait > 20)
			{
				boss->act_no = BALFROG_LEAP;
				boss->ani_wait = 0;
				boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
			}

			break;

		case BALFROG_LEAP:
			++boss->ani_wait;

			if (boss->ani_wait > 4)
			{
				boss->act_no = BALFROG_LEAP_MIDAIR;
				boss->ani_no = BALFROG_SPRITE_JUMPING;
				boss->ym = -5 * 0x200;
				boss->view.top = 64 * 0x200;
				boss->view.bottom = 24 * 0x200;
				PlaySoundObject(SND_SILLY_EXPLOSION, SOUND_MODE_PLAY);
			}

			break;

		case BALFROG_LEAP_MIDAIR:
			if (boss->flag & COLL_GROUND)
			{
				PlaySoundObject(SND_LARGE_OBJECT_HIT_GROUND, SOUND_MODE_PLAY);
				SetQuake(60);
				boss->act_no = BALFROG_WAIT;
				boss->ani_no = BALFROG_SPRITE_STANDING_STILL;
				boss->view.top = 48 * 0x200;
				boss->view.bottom = 16 * 0x200;

				for (i = 0; i < 2; ++i)
					SetNpChar(NPC_ENEMY_FROG, Random(4, 16) * (0x200 * 0x10), Random(0, 4) * (0x200 * 0x10), 0, 0, DIR_AUTO, NULL, 0x80);

				for (i = 0; i < 6; ++i)
					SetNpChar(NPC_ENEMY_PUCHI, Random(4, 16) * (0x200 * 0x10), Random(0, 4) * (0x200 * 0x10), 0, 0, DIR_AUTO, NULL, 0x80);

				for (i = 0; i < 8; ++i)
					SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + boss->hit.bottom, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);

				if (boss->direct == DIR_LEFT && boss->x < gMC.x)
				{
					boss->direct = DIR_RIGHT;
					boss->act_no = BALFROG_INITIALIZE_LAND;
				}

				if (boss->direct == DIR_RIGHT && boss->x > gMC.x)
				{
					boss->direct = DIR_LEFT;
					boss->act_no = BALFROG_INITIALIZE_LAND;
				}
			}

			break;

		case BALFROG_DIE:
			boss->act_no = BALFROG_DIE_FLASHING;
			boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			boss->act_wait = 0;
			boss->xm = 0;
			PlaySoundObject(SND_EXPLOSION, SOUND_MODE_PLAY);

			for (i = 0; i < 8; ++i)
				SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + Random(-12, 12) * 0x200, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);

			gBoss[1].cond = 0;
			gBoss[2].cond = 0;
			// Fallthrough
		case BALFROG_DIE_FLASHING:
			++boss->act_wait;

			if (boss->act_wait % 5 == 0)
				SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + Random(-12, 12) * 0x200, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);

			if (boss->act_wait / 2 % 2)
				boss->x -= 1 * 0x200;
			else
				boss->x += 1 * 0x200;

			if (boss->act_wait > 100)
			{
				boss->act_wait = 0;
				boss->act_no = BALFROG_REVERT;
			}

			break;

		case BALFROG_REVERT:
			++boss->act_wait;

			if (boss->act_wait / 2 % 2)
			{
				boss->view.front = 20 * 0x200;
				boss->view.top = 12 * 0x200;
				boss->view.back = 20 * 0x200;
				boss->view.bottom = 12 * 0x200;
				boss->ani_no = BALFROG_SPRITE_BALROG_WHITE;
			}
			else
			{
				boss->view.front = 48 * 0x200;
				boss->view.top = 48 * 0x200;
				boss->view.back = 32 * 0x200;
				boss->view.bottom = 16 * 0x200;
				boss->ani_no = BALFROG_SPRITE_MOUTH_OPEN_CROUCHING;
			}

			if (boss->act_wait % 9 == 0)
				SetNpChar(NPC_SMOKE, boss->x + Random(-12, 12) * 0x200, boss->y + Random(-12, 12) * 0x200, Random(-341, 341), Random(-3 * 0x200, 0), DIR_LEFT, NULL, 0x100);

			if (boss->act_wait > 150)
			{
				boss->act_no = BALFROG_NOP_START;
				boss->hit.bottom = 12 * 0x200;
			}

			break;

		case BALFROG_NOP_START:
			boss->act_no = BALFROG_NOP;
			// Fallthrough
		case BALFROG_NOP:
			if (boss->flag & COLL_GROUND)
			{
				boss->act_no = BALFROG_GO_INTO_CEILING;
				boss->act_wait = 0;
				boss->ani_no = BALFROG_SPRITE_BALROG_CROUCHING;
			}

			break;

		case BALFROG_GO_INTO_CEILING:
			++boss->act_wait;

			if (boss->act_wait > 30)
			{
				boss->ani_no = BALFROG_SPRITE_BALROG_JUMPING;
				boss->ym = -5 * 0x200;
				boss->bits |= NPC_IGNORE_SOLIDITY;
				boss->act_no = BALFROG_GONE_INTO_CEILING;
			}

			break;

		case BALFROG_GONE_INTO_CEILING:
			boss->ym = -5 * 0x200;

			if (boss->y < 0)
			{
				boss->cond = 0;
				PlaySoundObject(SND_LARGE_OBJECT_HIT_GROUND, SOUND_MODE_PLAY);
				SetQuake(30);
			}

			break;
	}

	boss->ym += 0x40;
	if (boss->ym > 0x5FF)
		boss->ym = 0x5FF;

	boss->x += boss->xm;
	boss->y += boss->ym;

	if (boss->direct == DIR_LEFT)
		boss->rect = rcLeft[boss->ani_no];
	else
		boss->rect = rcRight[boss->ani_no];

	ActBossChar02_01();
	ActBossChar02_02();
}

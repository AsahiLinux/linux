// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Bullet.h"

#include <stddef.h>
#include <string.h>

#include "WindowsWrapper.h"

#include "Draw.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Game.h"
#include "KeyControl.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Sound.h"

BULLET gBul[BULLET_MAX];

void InitBullet(void)
{
	// Identical to ClearBullet
	int i;

	for (i = 0; i < BULLET_MAX; ++i)
		gBul[i].cond = 0;
}

int CountArmsBullet(int arms_code)
{
	int i;
	int count = 0;

	for (i = 0; i < BULLET_MAX; ++i)
		if (gBul[i].cond & 0x80 && (gBul[i].code_bullet + 2) / 3 == arms_code)
			++count;

	return count;
}

int CountBulletNum(int bullet_code)
{
	int i;
	int count = 0;

	for (i = 0; i < BULLET_MAX; ++i)
		if (gBul[i].cond & 0x80 && gBul[i].code_bullet == bullet_code)
			++count;

	return count;
}

void DeleteBullet(int code)
{
	int i;
	int count = 0;	// Guessed name. This is unused, and was optimised out of the Linux port.

	for (i = 0; i < BULLET_MAX; ++i)
		if (gBul[i].cond & 0x80 && (gBul[i].code_bullet + 2) / 3 == code)
				gBul[i].cond = 0;
}

void ClearBullet(void)
{
	// Identical to InitBullet
	int i;

	for (i = 0; i < BULLET_MAX; ++i)
		gBul[i].cond = 0;
}

void PutBullet(int fx, int fy)
{
	int i;
	int x, y;

	for (i = 0; i < BULLET_MAX; ++i)
	{
		if (gBul[i].cond & 0x80)
		{
			switch (gBul[i].direct)
			{
				case DIR_LEFT:
					x = gBul[i].x - gBul[i].view.front;
					y = gBul[i].y - gBul[i].view.top;
					break;
				case DIR_UP:
					x = gBul[i].x - gBul[i].view.top;
					y = gBul[i].y - gBul[i].view.front;
					break;
				case DIR_RIGHT:
					x = gBul[i].x - gBul[i].view.back;
					y = gBul[i].y - gBul[i].view.top;
					break;
				case DIR_DOWN:
					x = gBul[i].x - gBul[i].view.top;
					y = gBul[i].y - gBul[i].view.back;
					break;
			}

			PutBitmap3(&grcGame, (x / 0x200) - (fx / 0x200), (y / 0x200) - (fy / 0x200), &gBul[i].rect, SURFACE_ID_BULLET);
		}
	}
}

BULLET_TABLE gBulTbl[] = {
	// Null
	{0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0}},
	// Snake
	{4, 1, 20, 36, 4, 4, 2, 2, {8, 8, 8, 8}},
	{6, 1, 23, 36, 4, 4, 2, 2, {8, 8, 8, 8}},
	{8, 1, 30, 36, 4, 4, 2, 2, {8, 8, 8, 8}},
	// Polar Star
	{1, 1, 8, 32, 6, 6, 2, 2, {8, 8, 8, 8}},
	{2, 1, 12, 32, 6, 6, 2, 2, {8, 8, 8, 8}},
	{4, 1, 16, 32, 6, 6, 2, 2, {8, 8, 8, 8}},
	// Fireball
	{2, 2, 100, 8, 8, 16, 4, 2, {8, 8, 8, 8}},
	{3, 2, 100, 8, 4, 4, 4, 2, {8, 8, 8, 8}},
	{3, 2, 100, 8, 4, 4, 4, 2, {8, 8, 8, 8}},
	// Machine Gun
	{2, 1, 20, 32, 2, 2, 2, 2, {8, 8, 8, 8}},
	{4, 1, 20, 32, 2, 2, 2, 2, {8, 8, 8, 8}},
	{6, 1, 20, 32, 2, 2, 2, 2, {8, 8, 8, 8}},
	// Missile Launcher
	{0, 10, 50, 40, 2, 2, 2, 2, {8, 8, 8, 8}},
	{0, 10, 70, 40, 4, 4, 4, 4, {8, 8, 8, 8}},
	{0, 10, 90, 40, 4, 4, 0, 0, {8, 8, 8, 8}},
	// Missile Launcher explosion
	{1, 100, 100, 20, 16, 16, 0, 0, {0, 0, 0, 0}},
	{1, 100, 100, 20, 16, 16, 0, 0, {0, 0, 0, 0}},
	{1, 100, 100, 20, 16, 16, 0, 0, {0, 0, 0, 0}},
	// Bubbler
	{1, 1, 20, 8, 2, 2, 2, 2, {4, 4, 4, 4}},
	{2, 1, 20, 8, 2, 2, 2, 2, {4, 4, 4, 4}},
	{2, 1, 20, 8, 4, 4, 4, 4, {4, 4, 4, 4}},
	// Bubbler level 3 thorns
	{3, 1, 32, 32, 2, 2, 2, 2, {4, 4, 4, 4}},
	// Blade slashes
	{0, 100, 0, 36, 8, 8, 8, 8, {12, 12, 12, 12}},
	// Falling spike that deals 127 damage
	{127, 1, 2, 4, 8, 4, 8, 4, {0, 0, 0, 0}},
	// Blade
	{15, 1, 30, 36, 8, 8, 4, 2, {8, 8, 8, 8}},
	{6, 3, 18, 36, 10, 10, 4, 2, {12, 12, 12, 12}},
	{1, 100, 30, 36, 6, 6, 4, 4, {12, 12, 12, 12}},
	// Super Missile Launcher
	{0, 10, 30, 40, 2, 2, 2, 2, {8, 8, 8, 8}},
	{0, 10, 40, 40, 4, 4, 4, 4, {8, 8, 8, 8}},
	{0, 10, 40, 40, 4, 4, 0, 0, {8, 8, 8, 8}},
	// Super Missile Launcher explosion
	{2, 100, 100, 20, 12, 12, 0, 0, {0, 0, 0, 0}},
	{2, 100, 100, 20, 12, 12, 0, 0, {0, 0, 0, 0}},
	{2, 100, 100, 20, 12, 12, 0, 0, {0, 0, 0, 0}},
	// Nemesis
	{4, 4, 20, 32, 4, 4, 3, 3, {8, 8, 24, 8}},
	{4, 2, 20, 32, 2, 2, 2, 2, {8, 8, 24, 8}},
	{1, 1, 20, 32, 2, 2, 2, 2, {8, 8, 24, 8}},
	// Spur
	{4, 4, 30, 64, 6, 6, 3, 3, {8, 8, 8, 8}},
	{8, 8, 30, 64, 6, 6, 3, 3, {8, 8, 8, 8}},
	{12, 12, 30, 64, 6, 6, 3, 3, {8, 8, 8, 8}},
	// Spur trail
	{3, 100, 30, 32, 6, 6, 3, 3, {4, 4, 4, 4}},
	{6, 100, 30, 32, 6, 6, 3, 3, {4, 4, 4, 4}},
	{11, 100, 30, 32, 6, 6, 3, 3, {4, 4, 4, 4}},
	// Curly's Nemesis
	{4, 4, 20, 32, 4, 4, 3, 3, {8, 8, 24, 8}},
	// Screen-nuke that kills all enemies
	{0, 4, 4, 4, 0, 0, 0, 0, {0, 0, 0, 0}},
	// Whimsical Star
	{1, 1, 1, 36, 1, 1, 1, 1, {1, 1, 1, 1}}
};

void SetBullet(int no, int x, int y, int dir)
{
	int i = 0;
	while (i < BULLET_MAX && gBul[i].cond & 0x80)
		++i;

	if (i >= BULLET_MAX)
		return;

	memset(&gBul[i], 0, sizeof(BULLET));
	gBul[i].code_bullet = no;
	gBul[i].cond = 0x80;
	gBul[i].direct = dir;
	gBul[i].damage = gBulTbl[no].damage;
	gBul[i].life = gBulTbl[no].life;
	gBul[i].life_count = gBulTbl[no].life_count;
	gBul[i].bbits = gBulTbl[no].bbits;
	gBul[i].enemyXL = gBulTbl[no].enemyXL * 0x200;
	gBul[i].enemyYL = gBulTbl[no].enemyYL * 0x200;
	gBul[i].blockXL = gBulTbl[no].blockXL * 0x200;
	gBul[i].blockYL = gBulTbl[no].blockYL * 0x200;
	gBul[i].view.back = gBulTbl[no].view.back * 0x200;
	gBul[i].view.front = gBulTbl[no].view.front * 0x200;
	gBul[i].view.top = gBulTbl[no].view.top * 0x200;
	gBul[i].view.bottom = gBulTbl[no].view.bottom * 0x200;
	gBul[i].x = x;
	gBul[i].y = y;
}

void ActBullet_Frontia1(BULLET *bul)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->ani_no = Random(0, 2);
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x600;
				break;
			case DIR_UP:
				bul->ym = -0x600;
				break;
			case DIR_RIGHT:
				bul->xm = 0x600;
				break;
			case DIR_DOWN:
				bul->ym = 0x600;
				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	if (++bul->ani_wait > 0)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 0;

	RECT rcLeft[4] = {
		{136, 80, 152, 80},
		{120, 80, 136, 96},
		{136, 64, 152, 80},
		{120, 64, 136, 80},
	};

	RECT rcRight[4] = {
		{120, 64, 136, 80},
		{136, 64, 152, 80},
		{120, 80, 136, 96},
		{136, 80, 152, 80},
	};

	if (bul->direct == DIR_LEFT)
		bul->rect = rcLeft[bul->ani_no];
	else
		bul->rect = rcRight[bul->ani_no];
}

void ActBullet_Frontia2(BULLET *bul, int level)
{
	static unsigned int inc;

	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->ani_no = Random(0, 2);
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x200;
				break;
			case DIR_UP:
				bul->ym = -0x200;
				break;
			case DIR_RIGHT:
				bul->xm = 0x200;
				break;
			case DIR_DOWN:
				bul->ym = 0x200;
				break;
		}

		++inc;

		switch (bul->direct)
		{
			case DIR_LEFT:
			case DIR_RIGHT:
				if (inc % 2)
					bul->ym = 0x400;
				else
					bul->ym = -0x400;

				break;

			case DIR_UP:
			case DIR_DOWN:
				if (inc % 2)
					bul->xm = 0x400;
				else
					bul->xm = -0x400;

				break;
		}
	}
	else
	{
		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm -= 0x80;
				break;
			case DIR_UP:
				bul->ym -= 0x80;
				break;
			case DIR_RIGHT:
				bul->xm += 0x80;
				break;
			case DIR_DOWN:
				bul->ym += 0x80;
				break;
		}

		switch (bul->direct)
		{
			case DIR_LEFT:
			case DIR_RIGHT:
				if (bul->count1 % 5 == 2)
				{
					if (bul->ym < 0)
						bul->ym = 0x400;
					else
						bul->ym = -0x400;
				}

				break;

			case DIR_UP:
			case DIR_DOWN:
				if (bul->count1 % 5 == 2)
				{
					if (bul->xm < 0)
						bul->xm = 0x400;
					else
						bul->xm = -0x400;
				}

				break;
		}

		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	if (++bul->ani_wait > 0)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 2)
		bul->ani_no = 0;

	RECT rect[3] = {
		{192, 16, 208, 32},
		{208, 16, 224, 32},
		{224, 16, 240, 32},
	};

	bul->rect = rect[bul->ani_no];

	if (level == 2)
		SetNpChar(129, bul->x, bul->y, 0, -0x200, bul->ani_no, NULL, 0x100);
	else
		SetNpChar(129, bul->x, bul->y, 0, -0x200, bul->ani_no + 3, NULL, 0x100);
}

void ActBullet_PoleStar(BULLET *bul, int level)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		// Set speed
		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x1000;
				break;
			case DIR_UP:
				bul->ym = -0x1000;
				break;
			case DIR_RIGHT:
				bul->xm = 0x1000;
				break;
			case DIR_DOWN:
				bul->ym = 0x1000;
				break;
		}

		// Set hitbox
		switch (level)
		{
			case 1:
				switch (bul->direct)
				{
					case DIR_LEFT:
						bul->enemyYL = 0x400;
						break;
					case DIR_UP:
						bul->enemyXL = 0x400;
						break;
					case DIR_RIGHT:
						bul->enemyYL = 0x400;
						break;
					case DIR_DOWN:
						bul->enemyXL = 0x400;
						break;
				}
				break;
			case 2:
				switch (bul->direct)
				{
					case DIR_LEFT:
						bul->enemyYL = 0x800;
						break;
					case DIR_UP:
						bul->enemyXL = 0x800;
						break;
					case DIR_RIGHT:
						bul->enemyYL = 0x800;
						break;
					case DIR_DOWN:
						bul->enemyXL = 0x800;
						break;
				}
				break;
		}
	}
	else
	{
		// Move
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	RECT rect1[2] = {
		{128, 32, 144, 48},
		{144, 32, 160, 48},
	};

	RECT rect2[2] = {
		{160, 32, 176, 48},
		{176, 32, 192, 48},
	};

	RECT rect3[2] = {
		{128, 48, 144, 64},
		{144, 48, 160, 64},
	};

	//Set framerect
	switch (level)
	{
		case 1:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect1[1];
			else
				bul->rect = rect1[0];

			break;

		case 2:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect2[1];
			else
				bul->rect = rect2[0];

			break;

		case 3:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect3[1];
			else
				bul->rect = rect3[0];

			break;
	}
}

void ActBullet_FireBall(BULLET *bul, int level)
{
	BOOL bBreak;

	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	bBreak = FALSE;
	if (bul->flag & 2 && bul->flag & 8)
		bBreak = TRUE;
	if (bul->flag & 1 && bul->flag & 4)
		bBreak = TRUE;

	if (bul->direct == DIR_LEFT && bul->flag & 1)
		bul->direct = DIR_RIGHT;
	if (bul->direct == DIR_RIGHT && bul->flag & 4)
		bul->direct = DIR_LEFT;

	if (bBreak)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		PlaySoundObject(28, SOUND_MODE_PLAY);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x400;
				break;

			case DIR_UP:
				bul->xm = gMC.xm;

				if (gMC.xm < 0)
					bul->direct = DIR_LEFT;
				else
					bul->direct = DIR_RIGHT;

				if (gMC.direct == DIR_LEFT)
					bul->xm -= 0x80;
				else
					bul->xm += 0x80;

				bul->ym = -0x5FF;
				break;

			case DIR_RIGHT:
				bul->xm = 0x400;
				break;

			case DIR_DOWN:
				bul->xm = gMC.xm;

				if (gMC.xm < 0)
					bul->direct = DIR_LEFT;
				else
					bul->direct = DIR_RIGHT;

				bul->ym = 0x5FF;

				break;
		}
	}
	else
	{
		if (bul->flag & 8)
			bul->ym = -0x400;
		else if (bul->flag & 1)
			bul->xm = 0x400;
		else if (bul->flag & 4)
			bul->xm = -0x400;

		bul->ym += 85;
		if (bul->ym > 0x3FF)
			bul->ym = 0x3FF;

		bul->x += bul->xm;
		bul->y += bul->ym;

		if (bul->flag & 0xD)
			PlaySoundObject(34, SOUND_MODE_PLAY);
	}

	RECT rect_left1[4] = {
		{128, 0, 144, 16},
		{144, 0, 160, 16},
		{160, 0, 176, 16},
		{176, 0, 192, 16},
	};

	RECT rect_right1[4] = {
		{128, 16, 144, 32},
		{144, 16, 160, 32},
		{160, 16, 176, 32},
		{176, 16, 192, 32},
	};

	RECT rect_left2[3] = {
		{192, 16, 208, 32},
		{208, 16, 224, 32},
		{224, 16, 240, 32},
	};

	RECT rect_right2[3] = {
		{224, 16, 240, 32},
		{208, 16, 224, 32},
		{192, 16, 208, 32},
	};

	++bul->ani_no;

	if (level == 1)
	{
		if (bul->ani_no > 3)
			bul->ani_no = 0;

		if (bul->direct == DIR_LEFT)
			bul->rect = rect_left1[bul->ani_no];
		else
			bul->rect = rect_right1[bul->ani_no];
	}
	else
	{
		if (bul->ani_no > 2)
			bul->ani_no = 0;

		if (bul->direct == DIR_LEFT)
			bul->rect = rect_left2[bul->ani_no];
		else
			bul->rect = rect_right2[bul->ani_no];

		if (level == 2)
			SetNpChar(129, bul->x, bul->y, 0, -0x200, bul->ani_no, NULL, 0x100);
		else
			SetNpChar(129, bul->x, bul->y, 0, -0x200, bul->ani_no + 3, NULL, 0x100);
	}
}

void ActBullet_MachineGun(BULLET *bul, int level)
{
	int move;

	RECT rect1[4] = {
		{64, 0, 80, 16},
		{80, 0, 96, 16},
		{96, 0, 112, 16},
		{112, 0, 128, 16},
	};

	RECT rect2[4] = {
		{64, 16, 80, 32},
		{80, 16, 96, 32},
		{96, 16, 112, 32},
		{112, 16, 128, 32},
	};

	RECT rect3[4] = {
		{64, 32, 80, 48},
		{80, 32, 96, 48},
		{96, 32, 112, 48},
		{112, 32, 128, 48},
	};

	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		switch (level)
		{
			case 1:
				move = 0x1000;
				break;
			case 2:
				move = 0x1000;
				break;
			case 3:
				move = 0x1000;
				break;
		}

		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -move;
				bul->ym = Random(-0xAA, 0xAA);
				break;
			case DIR_UP:
				bul->ym = -move;
				bul->xm = Random(-0xAA, 0xAA);
				break;
			case DIR_RIGHT:
				bul->xm = move;
				bul->ym = Random(-0xAA, 0xAA);
				break;
			case DIR_DOWN:
				bul->ym = move;
				bul->xm = Random(-0xAA, 0xAA);
				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;

		switch (level)
		{
			case 1:
				bul->rect = rect1[bul->direct];
				break;

			case 2:
				bul->rect = rect2[bul->direct];

				if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
					SetNpChar(127, bul->x, bul->y, 0, 0, DIR_UP, NULL, 0x100);
				else
					SetNpChar(127, bul->x, bul->y, 0, 0, DIR_LEFT, NULL, 0x100);

				break;

			case 3:
				bul->rect = rect3[bul->direct];
				SetNpChar(128, bul->x, bul->y, 0, 0, bul->direct, NULL, 0x100);
				break;
		}
	}
}

void ActBullet_Missile(BULLET *bul, int level)
{
	BOOL bHit;

	static unsigned int inc;

	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	bHit = FALSE;

	if (bul->life != 10)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 1)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 4)
		bHit = TRUE;
	if (bul->direct == DIR_UP && bul->flag & 2)
		bHit = TRUE;
	if (bul->direct == DIR_DOWN && bul->flag & 8)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 0x80)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 0x20)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 0x40)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 0x10)
		bHit = TRUE;

	if (bHit)
	{
		SetBullet(level + 15, bul->x, bul->y, DIR_LEFT);
		bul->cond = 0;
	}

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (bul->direct)
			{
				case DIR_LEFT:
				case DIR_RIGHT:
					bul->tgt_y = bul->y;
					break;
				case DIR_UP:
				case DIR_DOWN:
					bul->tgt_x = bul->x;
					break;
			}

			if (level == 3)
			{
				switch (bul->direct)
				{
					case DIR_LEFT:
					case DIR_RIGHT:
						if (bul->y > gMC.y)
							bul->ym = 0x100;
						else
							bul->ym = -0x100;

						bul->xm = Random(-0x200, 0x200);
						break;

					case DIR_UP:
					case DIR_DOWN:
						if (bul->x > gMC.x)
							bul->xm = 0x100;
						else
							bul->xm = -0x100;

						bul->ym = Random(-0x200, 0x200);
						break;
				}

				switch (++inc % 3)
				{
					case 0:
						bul->ani_no = 0x80;
						break;
					case 1:
						bul->ani_no = 0x40;
						break;
					case 2:
						bul->ani_no = 0x33;
						break;
				}
			}
			else
			{
				bul->ani_no = 0x80;
			}
			// Fallthrough
		case 1:
			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm += -bul->ani_no;
					break;
				case DIR_UP:
					bul->ym += -bul->ani_no;
					break;
				case DIR_RIGHT:
					bul->xm += bul->ani_no;
					break;
				case DIR_DOWN:
					bul->ym += bul->ani_no;
					break;
			}

			if (level == 3)
			{
				switch (bul->direct)
				{
					case DIR_LEFT:
					case DIR_RIGHT:
						if (bul->y < bul->tgt_y)
							bul->ym += 0x20;
						else
							bul->ym -= 0x20;

						break;

					case DIR_UP:
					case DIR_DOWN:
						if (bul->x < bul->tgt_x)
							bul->xm += 0x20;
						else
							bul->xm -= 0x20;
						break;
				}
			}

			if (bul->xm < -0xA00)
				bul->xm = -0xA00;
			if (bul->xm > 0xA00)
				bul->xm = 0xA00;

			if (bul->ym < -0xA00)
				bul->ym = -0xA00;
			if (bul->ym > 0xA00)
				bul->ym = 0xA00;

			bul->x += bul->xm;
			bul->y += bul->ym;

			break;
	}

	if (++bul->count2 > 2)
	{
		bul->count2 = 0;

		switch (bul->direct)
		{
			case DIR_LEFT:
				SetCaret(bul->x + (8 * 0x200), bul->y, CARET_EXHAUST, DIR_RIGHT);
				break;
			case DIR_UP:
				SetCaret(bul->x, bul->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);
				break;
			case DIR_RIGHT:
				SetCaret(bul->x - (8 * 0x200), bul->y, CARET_EXHAUST, DIR_LEFT);
				break;
			case DIR_DOWN:
				SetCaret(bul->x, bul->y - (8 * 0x200), CARET_EXHAUST, DIR_UP);
				break;
		}
	}

	RECT rect1[4] = {
		{0, 0, 16, 16},
		{16, 0, 32, 16},
		{32, 0, 48, 16},
		{48, 0, 64, 16},
	};

	RECT rect2[4] = {
		{0, 16, 16, 32},
		{16, 16, 32, 32},
		{32, 16, 48, 32},
		{48, 16, 64, 32},
	};

	RECT rect3[4] = {
		{0, 32, 16, 48},
		{16, 32, 32, 48},
		{32, 32, 48, 48},
		{48, 32, 64, 48},
	};

	switch (level)
	{
		case 1:
			bul->rect = rect1[bul->direct];
			break;
		case 2:
			bul->rect = rect2[bul->direct];
			break;
		case 3:
			bul->rect = rect3[bul->direct];
			break;
	}
}

void ActBullet_Bom(BULLET *bul, int level)
{
	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (level)
			{
				case 1:
					bul->act_wait = 10;
					break;
				case 2:
					bul->act_wait = 15;
					break;
				case 3:
					bul->act_wait = 5;
					break;
			}

			PlaySoundObject(44, SOUND_MODE_PLAY);
			// Fallthrough
		case 1:
			switch (level)
			{
				case 1:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-16, 16) * 0x200), bul->y + (Random(-16, 16) * 0x200), bul->enemyXL, 2);
					break;

				case 2:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-32, 32) * 0x200), bul->y + (Random(-32, 32) * 0x200), bul->enemyXL, 2);
					break;

				case 3:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-40, 40) * 0x200), bul->y + (Random(-40, 40) * 0x200), bul->enemyXL, 2);
					break;
			}

			if (--bul->act_wait < 0)
				bul->cond = 0;

			break;
	}
}

void ActBullet_Bubblin1(BULLET *bul)
{
	if (bul->flag & 0x2FF)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		return;
	}

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm = -0x600;
					break;
				case DIR_RIGHT:
					bul->xm = 0x600;
					break;
				case DIR_UP:
					bul->ym = -0x600;
					break;
				case DIR_DOWN:
					bul->ym = 0x600;
					break;
			}

			break;
	}

	switch (bul->direct)
	{
		case DIR_LEFT:
			bul->xm += 0x2A;
			break;
		case DIR_RIGHT:
			bul->xm -= 0x2A;
			break;
		case DIR_UP:
			bul->ym += 0x2A;
			break;
		case DIR_DOWN:
			bul->ym -= 0x2A;
			break;
	}

	bul->x += bul->xm;
	bul->y += bul->ym;

	if (++bul->act_wait > 40)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION_TINY, DIR_LEFT);
	}

	RECT rect[4] = {
		{192, 0, 200, 8},
		{200, 0, 208, 8},
		{208, 0, 216, 8},
		{216, 0, 224, 8},
	};

	if (++bul->ani_wait > 3)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 3;

	bul->rect = rect[bul->ani_no];
}

void ActBullet_Bubblin2(BULLET *bul)
{
	BOOL bDelete = FALSE;

	if (bul->direct == DIR_LEFT && bul->flag & 1)
		bDelete = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 4)
		bDelete = TRUE;
	if (bul->direct == DIR_UP && bul->flag & 2)
		bDelete = TRUE;
	if (bul->direct == DIR_DOWN && bul->flag & 8)
		bDelete = TRUE;

	if (bDelete)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		return;
	}

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm = -0x600;
					bul->ym = Random(-0x100, 0x100);
					break;
				case DIR_RIGHT:
					bul->xm = 0x600;
					bul->ym = Random(-0x100, 0x100);
					break;
				case DIR_UP:
					bul->ym = -0x600;
					bul->xm = Random(-0x100, 0x100);
					break;
				case DIR_DOWN:
					bul->ym = 0x600;
					bul->xm = Random(-0x100, 0x100);
					break;
			}

			break;
	}

	switch (bul->direct)
	{
		case DIR_LEFT:
			bul->xm += 0x10;
			break;
		case DIR_RIGHT:
			bul->xm -= 0x10;
			break;
		case DIR_UP:
			bul->ym += 0x10;
			break;
		case DIR_DOWN:
			bul->ym -= 0x10;
			break;
	}

	bul->x += bul->xm;
	bul->y += bul->ym;

	if (++bul->act_wait > 60)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION_TINY, DIR_LEFT);
	}

	RECT rect[4] = {
		{192, 8, 200, 16},
		{200, 8, 208, 16},
		{208, 8, 216, 16},
		{216, 8, 224, 16},
	};

	if (++bul->ani_wait > 3)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 3;

	bul->rect = rect[bul->ani_no];
}

void ActBullet_Bubblin3(BULLET *bul)
{
	if (++bul->act_wait > 100 || !(gKey & gKeyShot))
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_PROJECTILE_DISSIPATION, DIR_LEFT);
		PlaySoundObject(100, SOUND_MODE_PLAY);

		if (gMC.up)
			SetBullet(22, bul->x, bul->y, DIR_UP);
		else if (gMC.down)
			SetBullet(22, bul->x, bul->y, DIR_DOWN);
		else
			SetBullet(22, bul->x, bul->y, gMC.direct);

		return;
	}

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm = Random(-0x400, -0x200);
					bul->ym = (Random(-4, 4) * 0x200) / 2;
					break;
				case DIR_RIGHT:
					bul->xm = Random(0x200, 0x400);
					bul->ym = (Random(-4, 4) * 0x200) / 2;
					break;
				case DIR_UP:
					bul->ym = Random(-0x400, -0x200);
					bul->xm = (Random(-4, 4) * 0x200) / 2;
					break;
				case DIR_DOWN:
					bul->ym = Random(0x80, 0x100);
					bul->xm = (Random(-4, 4) * 0x200) / 2;
					break;
			}

			break;
	}

	if (bul->x < gMC.x)
		bul->xm += 0x20;
	if (bul->x > gMC.x)
		bul->xm -= 0x20;

	if (bul->y < gMC.y)
		bul->ym += 0x20;
	if (bul->y > gMC.y)
		bul->ym -= 0x20;

	if (bul->xm < 0 && bul->flag & 1)
		bul->xm = 0x400;
	if (bul->xm > 0 && bul->flag & 4)
		bul->xm = -0x400;

	if (bul->ym < 0 && bul->flag & 2)
		bul->ym = 0x400;
	if (bul->ym > 0 && bul->flag & 8)
		bul->ym = -0x400;

	bul->x += bul->xm;
	bul->y += bul->ym;

	RECT rect[4] = {
		{240, 16, 248, 24},
		{248, 16, 256, 24},
		{240, 24, 248, 32},
		{248, 24, 256, 32},
	};

	if (++bul->ani_wait > 3)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 3;

	bul->rect = rect[bul->ani_no];
}

void ActBullet_Spine(BULLET *bul)
{
	if (++bul->count1 > bul->life_count || bul->flag & 8)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = (-Random(10, 16) * 0x200) / 2;
				break;
			case DIR_UP:
				bul->ym = (-Random(10, 16) * 0x200) / 2;
				break;
			case DIR_RIGHT:
				bul->xm = (Random(10, 16) * 0x200) / 2;
				break;
			case DIR_DOWN:
				bul->ym = (Random(10, 16) * 0x200) / 2;
				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	if (++bul->ani_wait > 1)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 1)
		bul->ani_no = 0;

	RECT rcLeft[2] = {
		{224, 0, 232, 8},
		{232, 0, 240, 8},
	};

	RECT rcRight[2] = {
		{224, 0, 232, 8},
		{232, 0, 240, 8},
	};

	RECT rcDown[2] = {
		{224, 8, 232, 16},
		{232, 8, 240, 16},
	};

	switch (bul->direct)
	{
		case DIR_LEFT:
			bul->rect = rcLeft[bul->ani_no];
			break;
		case DIR_UP:
			bul->rect = rcDown[bul->ani_no];
			break;
		case DIR_RIGHT:
			bul->rect = rcRight[bul->ani_no];
			break;
		case DIR_DOWN:
			bul->rect = rcDown[bul->ani_no];
			break;
	}
}

void ActBullet_Sword1(BULLET *bul)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->count1 == 3)
		bul->bbits &= ~4;

	if (bul->count1 % 5 == 1)
		PlaySoundObject(34, SOUND_MODE_PLAY);

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x800;
				break;
			case DIR_UP:
				bul->ym = -0x800;
				break;
			case DIR_RIGHT:
				bul->xm = 0x800;
				break;
			case DIR_DOWN:
				bul->ym = 0x800;
				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	RECT rcLeft[4] = {
		{0, 48, 16, 64},
		{16, 48, 32, 64},
		{32, 48, 48, 64},
		{48, 48, 64, 64},
	};

	RECT rcRight[4] = {
		{64, 48, 80, 64},
		{80, 48, 96, 64},
		{96, 48, 112, 64},
		{112, 48, 128, 64},
	};

	if (++bul->ani_wait > 1)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 0;

	if (bul->direct == DIR_LEFT)
		bul->rect = rcLeft[bul->ani_no];
	else
		bul->rect = rcRight[bul->ani_no];
}

void ActBullet_Sword2(BULLET *bul)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->count1 == 3)
		bul->bbits &= ~4;

	if (bul->count1 % 7 == 1)
		PlaySoundObject(106, SOUND_MODE_PLAY);

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x800;
				break;
			case DIR_UP:
				bul->ym = -0x800;
				break;
			case DIR_RIGHT:
				bul->xm = 0x800;
				break;
			case DIR_DOWN:
				bul->ym = 0x800;
				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	RECT rcLeft[4] = {
		{160, 48, 184, 72},
		{184, 48, 208, 72},
		{208, 48, 232, 72},
		{232, 48, 256, 72},
	};

	RECT rcRight[4] = {
		{160, 72, 184, 96},
		{184, 72, 208, 96},
		{208, 72, 232, 96},
		{232, 72, 256, 96},
	};

	if (++bul->ani_wait > 1)
	{
		bul->ani_wait = 0;
		++bul->ani_no;
	}

	if (bul->ani_no > 3)
		bul->ani_no = 0;

	if (bul->direct == DIR_LEFT)
		bul->rect = rcLeft[bul->ani_no];
	else
		bul->rect = rcRight[bul->ani_no];
}

void ActBullet_Sword3(BULLET *bul)
{
	RECT rcLeft[2] = {
		{272, 0, 296, 24},
		{296, 0, 320, 24},
	};

	RECT rcUp[2] = {
		{272, 48, 296, 72},
		{296, 0, 320, 24},
	};

	RECT rcRight[2] = {
		{272, 24, 296, 48},
		{296, 24, 320, 48},
	};

	RECT rcDown[2] = {
		{296, 48, 320, 72},
		{296, 24, 320, 48},
	};

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;
			bul->xm = 0;
			bul->ym = 0;
			// Fallthrough
		case 1:
			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm = -0x800;
					break;
				case DIR_UP:
					bul->ym = -0x800;
					break;
				case DIR_RIGHT:
					bul->xm = 0x800;
					break;
				case DIR_DOWN:
					bul->ym = 0x800;
					break;
			}

			if (bul->life != 100)
			{
				bul->act_no = 2;
				bul->ani_no = 1;
				bul->damage = -1;
				bul->act_wait = 0;
			}

			if (++bul->act_wait % 4 == 1)
			{
				PlaySoundObject(106, SOUND_MODE_PLAY);

				if (++bul->count1 % 2)
					SetBullet(23, bul->x, bul->y, DIR_LEFT);
				else
					SetBullet(23, bul->x, bul->y, DIR_RIGHT);
			}

			if (++bul->count1 == 5)
				bul->bbits &= ~4;

			if (bul->count1 > bul->life_count)
			{
				bul->cond = 0;
				SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
				return;
			}

			break;

		case 2:
			bul->xm = 0;
			bul->ym = 0;
			++bul->act_wait;

			if (Random(-1, 1) == 0)
			{
				PlaySoundObject(106, SOUND_MODE_PLAY);

				if (Random(0, 1) % 2)
					SetBullet(23, bul->x + (Random(-0x40, 0x40) * 0x200), bul->y + (Random(-0x40, 0x40) * 0x200), DIR_LEFT);
				else
					SetBullet(23, bul->x + (Random(-0x40, 0x40) * 0x200), bul->y + (Random(-0x40, 0x40) * 0x200), DIR_RIGHT);
			}

			if (bul->act_wait > 50)
				bul->cond = 0;
	}

	bul->x += bul->xm;
	bul->y += bul->ym;

	switch (bul->direct)
	{
		case DIR_LEFT:
			bul->rect = rcLeft[bul->ani_no];
			break;
		case DIR_UP:
			bul->rect = rcUp[bul->ani_no];
			break;
		case DIR_RIGHT:
			bul->rect = rcRight[bul->ani_no];
			break;
		case DIR_DOWN:
			bul->rect = rcDown[bul->ani_no];
			break;
	}

	if (bul->act_wait % 2)
		bul->rect.right = 0;
}

void ActBullet_Edge(BULLET *bul)
{
	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;
			bul->y -= 12 * 0x200;

			if (bul->direct == DIR_LEFT)
				bul->x += 16 * 0x200;
			else
				bul->x -= 16 * 0x200;
			// Fallthrough
		case 1:
			if (++bul->ani_wait > 2)
			{
				bul->ani_wait = 0;
				++bul->ani_no;
			}

			if (bul->direct == DIR_LEFT)
				bul->x -= 2 * 0x200;
			else
				bul->x += 2 * 0x200;

			bul->y += 2 * 0x200;

			if (bul->ani_no == 1)
				bul->damage = 2;
			else
				bul->damage = 1;

			if (bul->ani_no > 4)
			{
				bul->cond = 0;
			#ifdef FIX_MAJOR_BUGS
				return;	// The code below will use 'ani_no' to access 'rcLeft' and 'rcRight', even though it's now too high
			#endif
			}

			break;
	}

	RECT rcLeft[5] = {
		{0, 64, 24, 88},
		{24, 64, 48, 88},
		{48, 64, 72, 88},
		{72, 64, 96, 88},
		{96, 64, 120, 88},
	};

	RECT rcRight[5] = {
		{0, 88, 24, 112},
		{24, 88, 48, 112},
		{48, 88, 72, 112},
		{72, 88, 96, 112},
		{96, 88, 120, 112},
	};

	if (bul->direct == DIR_LEFT)
		bul->rect = rcLeft[bul->ani_no];
	else
		bul->rect = rcRight[bul->ani_no];
}

void ActBullet_Drop(BULLET *bul)
{
	RECT rc = {0, 0, 0, 0};

	if (++bul->act_wait > 2)
		bul->cond = 0;

	bul->rect = rc;
}

void ActBullet_SuperMissile(BULLET *bul, int level)
{
	BOOL bHit;

	static unsigned int inc;

	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	bHit = FALSE;

	if (bul->life != 10)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 1)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 4)
		bHit = TRUE;
	if (bul->direct == DIR_UP && bul->flag & 2)
		bHit = TRUE;
	if (bul->direct == DIR_DOWN && bul->flag & 8)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 0x80)
		bHit = TRUE;
	if (bul->direct == DIR_LEFT && bul->flag & 0x20)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 0x40)
		bHit = TRUE;
	if (bul->direct == DIR_RIGHT && bul->flag & 0x10)
		bHit = TRUE;

	if (bHit)
	{
		SetBullet(level + 30, bul->x, bul->y, DIR_LEFT);
		bul->cond = 0;
	}

	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (bul->direct)
			{
				case DIR_LEFT:
				case DIR_RIGHT:
					bul->tgt_y = bul->y;
					bul->enemyXL = 0x1000;
					bul->blockXL = 0x1000;
					break;

				case DIR_UP:
				case DIR_DOWN:
					bul->tgt_x = bul->x;
					bul->enemyYL = 0x1000;
					bul->blockYL = 0x1000;
					break;
			}

			if (level == 3)
			{
				switch (bul->direct)
				{
					case DIR_LEFT:
					case DIR_RIGHT:
						if (bul->y > gMC.y)
							bul->ym = 0x100;
						else
							bul->ym = -0x100;

						bul->xm = Random(-0x200, 0x200);
						break;

					case DIR_UP:
					case DIR_DOWN:
						if (bul->x > gMC.x)
							bul->xm = 0x100;
						else
							bul->xm = -0x100;

						bul->ym = Random(-0x200, 0x200);
						break;
				}

				switch (++inc % 3)
				{
					case 0:
						bul->ani_no = 0x200;
						break;
					case 1:
						bul->ani_no = 0x100;
						break;
					case 2:
						bul->ani_no = 0xAA;
						break;
				}
			}
			else
			{
				bul->ani_no = 0x200;
			}
			// Fallthrough
		case 1:
			switch (bul->direct)
			{
				case DIR_LEFT:
					bul->xm += -bul->ani_no;
					break;
				case DIR_UP:
					bul->ym += -bul->ani_no;
					break;
				case DIR_RIGHT:
					bul->xm += bul->ani_no;
					break;
				case DIR_DOWN:
					bul->ym += bul->ani_no;
					break;
			}

			if (level == 3)
			{
				switch (bul->direct)
				{
					case DIR_LEFT:
					case DIR_RIGHT:
						if (bul->y < bul->tgt_y)
							bul->ym += 0x40;
						else
							bul->ym -= 0x40;

						break;
					case DIR_UP:
					case DIR_DOWN:
						if (bul->x < bul->tgt_x)
							bul->xm += 0x40;
						else
							bul->xm -= 0x40;

						break;
				}
			}

			if (bul->xm < -0x1400)
				bul->xm = -0x1400;
			if (bul->xm > 0x1400)
				bul->xm = 0x1400;

			if (bul->ym < -0x1400)
				bul->ym = -0x1400;
			if (bul->ym > 0x1400)
				bul->ym = 0x1400;

			bul->x += bul->xm;
			bul->y += bul->ym;

			break;
	}

	if (++bul->count2 > 2)
	{
		bul->count2 = 0;

		switch (bul->direct)
		{
			case DIR_LEFT:
				SetCaret(bul->x + (8 * 0x200), bul->y, CARET_EXHAUST, DIR_RIGHT);
				break;
			case DIR_UP:
				SetCaret(bul->x, bul->y + (8 * 0x200), CARET_EXHAUST, DIR_DOWN);
				break;
			case DIR_RIGHT:
				SetCaret(bul->x - (8 * 0x200), bul->y, CARET_EXHAUST, DIR_LEFT);
				break;
			case DIR_DOWN:
				SetCaret(bul->x, bul->y - (8 * 0x200), CARET_EXHAUST, DIR_UP);
				break;
		}
	}

	RECT rect1[4] = {
		{120, 96, 136, 112},
		{136, 96, 152, 112},
		{152, 96, 168, 112},
		{168, 96, 184, 112},
	};

	RECT rect2[4] = {
		{184, 96, 200, 112},
		{200, 96, 216, 112},
		{216, 96, 232, 112},
		{232, 96, 248, 112},
	};

	switch (level)
	{
		case 1:
			bul->rect = rect1[bul->direct];
			break;
		case 2:
			bul->rect = rect2[bul->direct];
			break;
		case 3:
			bul->rect = rect1[bul->direct];
			break;
	}
}

void ActBullet_SuperBom(BULLET *bul, int level)
{
	switch (bul->act_no)
	{
		case 0:
			bul->act_no = 1;

			switch (level)
			{
				case 1:
					bul->act_wait = 10;
					break;
				case 2:
					bul->act_wait = 14;
					break;
				case 3:
					bul->act_wait = 6;
					break;
			}

			PlaySoundObject(44, SOUND_MODE_PLAY);
			// Fallthrough
		case 1:
			switch (level)
			{
				case 1:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-16, 16) * 0x200), bul->y + (Random(-16, 16) * 0x200), bul->enemyXL, 2);
					break;
				case 2:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-32, 32) * 0x200), bul->y + (Random(-32, 32) * 0x200), bul->enemyXL, 2);
					break;
				case 3:
					if (bul->act_wait % 3 == 0)
						SetDestroyNpCharUp(bul->x + (Random(-40, 40) * 0x200), bul->y + (Random(-40, 40) * 0x200), bul->enemyXL, 2);
					break;
			}

			if (--bul->act_wait < 0)
				bul->cond = 0;

			break;
	}
}

void ActBullet_Nemesis(BULLET *bul, int level)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->act_no == 0)
	{
		bul->act_no = 1;
		bul->count1 = 0;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x1000;
				break;
			case DIR_UP:
				bul->ym = -0x1000;
				break;
			case DIR_RIGHT:
				bul->xm = 0x1000;
				break;
			case DIR_DOWN:
				bul->ym = 0x1000;
				break;
		}

		switch (level)
		{
			case 3:
				bul->xm /= 3;
				bul->ym /= 3;
				break;
		}
	}
	else
	{
		if (level == 1 && bul->count1 % 4 == 1)
		{
			switch (bul->direct)
			{
				case DIR_LEFT:
					SetNpChar(4, bul->x, bul->y, -0x200, Random(-0x200, 0x200), DIR_RIGHT, NULL, 0x100);
					break;
				case DIR_UP:
					SetNpChar(4, bul->x, bul->y, Random(-0x200, 0x200), -0x200, DIR_RIGHT, NULL, 0x100);
					break;
				case DIR_RIGHT:
					SetNpChar(4, bul->x, bul->y, 0x200, Random(-0x200, 0x200), DIR_RIGHT, NULL, 0x100);
					break;
				case DIR_DOWN:
					SetNpChar(4, bul->x, bul->y, Random(-0x200, 0x200), 0x200, DIR_RIGHT, NULL, 0x100);
					break;
			}
		}

		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	if (++bul->ani_no > 1)
		bul->ani_no = 0;

	RECT rcL[2] = {
		{0, 112, 32, 128},
		{0, 128, 32, 144},
	};

	RECT rcU[2] = {
		{32, 112, 48, 144},
		{48, 112, 64, 144},
	};

	RECT rcR[2] = {
		{64, 112, 96, 128},
		{64, 128, 96, 144},
	};

	RECT rcD[2] = {
		{96, 112, 112, 144},
		{112, 112, 128, 144},
	};

	switch (bul->direct)
	{
		case DIR_LEFT:
			bul->rect = rcL[bul->ani_no];
			break;
		case DIR_UP:
			bul->rect = rcU[bul->ani_no];
			break;
		case DIR_RIGHT:
			bul->rect = rcR[bul->ani_no];
			break;
		case DIR_DOWN:
			bul->rect = rcD[bul->ani_no];
			break;
	}

	bul->rect.top += ((level - 1) / 2) * 32;
	bul->rect.bottom += ((level - 1) / 2) * 32;
	bul->rect.left += ((level - 1) % 2) * 128;
	bul->rect.right += ((level - 1) % 2) * 128;
}

void ActBullet_Spur(BULLET *bul, int level)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		SetCaret(bul->x, bul->y, CARET_SHOOT, DIR_LEFT);
		return;
	}

	if (bul->damage && bul->life != 100)
		bul->damage = 0;

	if (bul->act_no == 0)
	{
		bul->act_no = 1;

		switch (bul->direct)
		{
			case DIR_LEFT:
				bul->xm = -0x1000;
				break;
			case DIR_UP:
				bul->ym = -0x1000;
				break;
			case DIR_RIGHT:
				bul->xm = 0x1000;
				break;
			case DIR_DOWN:
				bul->ym = 0x1000;
				break;
		}

		switch (level)
		{
			case 1:
				switch (bul->direct)
				{
					case DIR_LEFT:
						bul->enemyYL = 0x400;
						break;
					case DIR_UP:
						bul->enemyXL = 0x400;
						break;
					case DIR_RIGHT:
						bul->enemyYL = 0x400;
						break;
					case DIR_DOWN:
						bul->enemyXL = 0x400;
						break;
				}

				break;

			case 2:
				switch (bul->direct)
				{
					case DIR_LEFT:
						bul->enemyYL = 0x800;
						break;
					case DIR_UP:
						bul->enemyXL = 0x800;
						break;
					case DIR_RIGHT:
						bul->enemyYL = 0x800;
						break;
					case DIR_DOWN:
						bul->enemyXL = 0x800;
						break;
				}

				break;
		}
	}
	else
	{
		bul->x += bul->xm;
		bul->y += bul->ym;
	}

	RECT rect1[2] = {
		{128, 32, 144, 48},
		{144, 32, 160, 48},
	};

	RECT rect2[2] = {
		{160, 32, 176, 48},
		{176, 32, 192, 48},
	};

	RECT rect3[2] = {
		{128, 48, 144, 64},
		{144, 48, 160, 64},
	};

	bul->damage = bul->life;

	switch (level)
	{
		case 1:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect1[1];
			else
				bul->rect = rect1[0];

			break;

		case 2:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect2[1];
			else
				bul->rect = rect2[0];

			break;

		case 3:
			if (bul->direct == DIR_UP || bul->direct == DIR_DOWN)
				bul->rect = rect3[1];
			else
				bul->rect = rect3[0];

			break;
	}

	SetBullet(39 + level, bul->x, bul->y, bul->direct);
}

void ActBullet_SpurTail(BULLET *bul, int level)
{
	if (++bul->count1 > 20)
		bul->ani_no = bul->count1 - 20;

	if (bul->ani_no > 2)
	{
		bul->cond = 0;
	#ifdef FIX_MAJOR_BUGS
		return;	// Avoid accessing the RECT arrays with an out-of-bounds index
	#endif
	}

	if (bul->damage && bul->life != 100)
		bul->damage = 0;

	RECT rc_h_lv1[3] = {
		{192, 32, 200, 40},
		{200, 32, 208, 40},
		{208, 32, 216, 40},
	};

	RECT rc_v_lv1[3] = {
		{192, 40, 200, 48},
		{200, 40, 208, 48},
		{208, 40, 216, 48},
	};

	RECT rc_h_lv2[3] = {
		{216, 32, 224, 40},
		{224, 32, 232, 40},
		{232, 32, 240, 40},
	};

	RECT rc_v_lv2[3] = {
		{216, 40, 224, 48},
		{224, 40, 232, 48},
		{232, 40, 240, 48},
	};

	RECT rc_h_lv3[3] = {
		{240, 32, 248, 40},
		{248, 32, 256, 40},
		{256, 32, 264, 40},
	};

	RECT rc_v_lv3[3] = {
		{240, 32, 248, 40},
		{248, 32, 256, 40},
		{256, 32, 264, 40},
	};

	switch (level)
	{
		case 1:
			if (bul->direct == DIR_LEFT || bul->direct == DIR_RIGHT)
				bul->rect = rc_h_lv1[bul->ani_no];
			else
				bul->rect = rc_v_lv1[bul->ani_no];

			break;

		case 2:
			if (bul->direct == DIR_LEFT || bul->direct == DIR_RIGHT)
				bul->rect = rc_h_lv2[bul->ani_no];
			else
				bul->rect = rc_v_lv2[bul->ani_no];

			break;

		case 3:
			if (bul->direct == DIR_LEFT || bul->direct == DIR_RIGHT)
				bul->rect = rc_h_lv3[bul->ani_no];
			else
				bul->rect = rc_v_lv3[bul->ani_no];

			break;
	}
}

void ActBullet_EnemyClear(BULLET *bul)
{
	if (++bul->count1 > bul->life_count)
	{
		bul->cond = 0;
		return;
	}

	bul->damage = 10000;
	bul->enemyXL = 0xC8000;
	bul->enemyYL = 0xC8000;
}

void ActBullet_Star(BULLET *bul)
{
	if (++bul->count1 > bul->life_count)
		bul->cond = 0;
}

void ActBullet(void)
{
	int i;

	for (i = 0; i < BULLET_MAX; ++i)
	{
		if (gBul[i].cond & 0x80)
		{
			if (gBul[i].life < 1)
			{
				gBul[i].cond = 0;
				continue;
			}

			switch (gBul[i].code_bullet)
			{
				// Snake
				case 1:
					ActBullet_Frontia1(&gBul[i]);
					break;
				case 2:
					ActBullet_Frontia2(&gBul[i], 2);
					break;
				case 3:
					ActBullet_Frontia2(&gBul[i], 3);
					break;

				// Polar Star
				case 4:
					ActBullet_PoleStar(&gBul[i], 1);
					break;
				case 5:
					ActBullet_PoleStar(&gBul[i], 2);
					break;
				case 6:
					ActBullet_PoleStar(&gBul[i], 3);
					break;

				// Fireball
				case 7:
					ActBullet_FireBall(&gBul[i], 1);
					break;
				case 8:
					ActBullet_FireBall(&gBul[i], 2);
					break;
				case 9:
					ActBullet_FireBall(&gBul[i], 3);
					break;

				// Machine Gun
				case 10:
					ActBullet_MachineGun(&gBul[i], 1);
					break;
				case 11:
					ActBullet_MachineGun(&gBul[i], 2);
					break;
				case 12:
					ActBullet_MachineGun(&gBul[i], 3);
					break;

				// Missile Launcher
				case 13:
					ActBullet_Missile(&gBul[i], 1);
					break;
				case 14:
					ActBullet_Missile(&gBul[i], 2);
					break;
				case 15:
					ActBullet_Missile(&gBul[i], 3);
					break;

				// Missile Launcher explosion
				case 16:
					ActBullet_Bom(&gBul[i], 1);
					break;
				case 17:
					ActBullet_Bom(&gBul[i], 2);
					break;
				case 18:
					ActBullet_Bom(&gBul[i], 3);
					break;

				// Bubbler
				case 19:
					ActBullet_Bubblin1(&gBul[i]);
					break;
				case 20:
					ActBullet_Bubblin2(&gBul[i]);
					break;
				case 21:
					ActBullet_Bubblin3(&gBul[i]);
					break;

				// Bubbler level 3 spines
				case 22:
					ActBullet_Spine(&gBul[i]);
					break;

				// Blade slashes
				case 23:
					ActBullet_Edge(&gBul[i]);
					break;

				// Falling spike that deals 127 damage
				case 24:
					ActBullet_Drop(&gBul[i]);
					break;

				// Blade
				case 25:
					ActBullet_Sword1(&gBul[i]);
					break;
				case 26:
					ActBullet_Sword2(&gBul[i]);
					break;
				case 27:
					ActBullet_Sword3(&gBul[i]);
					break;

				// Super Missile Launcher
				case 28:
					ActBullet_SuperMissile(&gBul[i], 1);
					break;
				case 29:
					ActBullet_SuperMissile(&gBul[i], 2);
					break;
				case 30:
					ActBullet_SuperMissile(&gBul[i], 3);
					break;

				// Super Missile Launcher explosion
				case 31:
					ActBullet_SuperBom(&gBul[i], 1);
					break;
				case 32:
					ActBullet_SuperBom(&gBul[i], 2);
					break;
				case 33:
					ActBullet_SuperBom(&gBul[i], 3);
					break;

				// Nemesis
				case 34:	// Identical to case 43
					ActBullet_Nemesis(&gBul[i], 1);
					break;
				case 35:
					ActBullet_Nemesis(&gBul[i], 2);
					break;
				case 36:
					ActBullet_Nemesis(&gBul[i], 3);
					break;

				// Spur
				case 37:
					ActBullet_Spur(&gBul[i], 1);
					break;
				case 38:
					ActBullet_Spur(&gBul[i], 2);
					break;
				case 39:
					ActBullet_Spur(&gBul[i], 3);
					break;

				// Spur trail
				case 40:
					ActBullet_SpurTail(&gBul[i], 1);
					break;
				case 41:
					ActBullet_SpurTail(&gBul[i], 2);
					break;
				case 42:
					ActBullet_SpurTail(&gBul[i], 3);
					break;

				// Curly's Nemesis
				case 43:	// Identical to case 34
					ActBullet_Nemesis(&gBul[i], 1);
					break;

				// Screen-nuke that kills all enemies
				case 44:
					ActBullet_EnemyClear(&gBul[i]);
					break;

				// Whimsical Star
				case 45:
					ActBullet_Star(&gBul[i]);
					break;
			}
		}
	}
}

BOOL IsActiveSomeBullet(void)
{
	int i;

	for (i = 0; i < BULLET_MAX; ++i)
	{
		if (gBul[i].cond & 0x80)
		{
			switch (gBul[i].code_bullet)
			{
				case 13:
				case 14:
				case 15:
				case 16:
				case 17:
				case 18:
				case 23:
				case 25:
				case 26:
				case 27:
				case 28:
				case 29:
				case 30:
				case 31:
				case 32:
				case 33:
					return TRUE;
			}
		}
	}

	return FALSE;
}

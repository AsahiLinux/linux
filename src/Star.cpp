// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Star.h"

#include <string.h>

#include "WindowsWrapper.h"

#include "Bullet.h"
#include "Draw.h"
#include "Game.h"
#include "MyChar.h"

static struct
{
	int cond;
	int code;
	int direct;
	int x;
	int y;
	int xm;
	int ym;
	int act_no;
	int act_wait;
	int ani_no;
	int ani_wait;
	int view_left;
	int view_top;
	RECT rect;
} star[3];

void InitStar(void)
{
	// Clear stars
	memset(star, 0, sizeof(star));

	// Position
	star[0].x = gMC.x;
	star[0].y = gMC.y;

	star[1].x = gMC.x;
	star[1].y = gMC.y;

	star[2].x = gMC.x;
	star[2].y = gMC.y;

	// Speed
	star[0].xm = 0x400;
	star[0].ym = -0x200;

	star[1].xm = -0x200;
	star[1].ym = 0x400;

	star[2].xm = 0x200;
	star[2].ym = 0x200;
}

void ActStar(void)
{
	int i;
	static int a;

	++a;
	a %= 3;

	for (i = 0; i < 3; ++i)
	{
		if (i != 0)
		{
			if (star[i - 1].x < star[i].x)
				star[i].xm -= 0x80;
			else
				star[i].xm += 0x80;

			if (star[i - 1].y < star[i].y)
				star[i].ym -= 0xAA;
			else
				star[i].ym += 0xAA;
		}
		else
		{
			if (gMC.x < star[i].x)
				star[i].xm -= 0x80;
			else
				star[i].xm += 0x80;

			if (gMC.y < star[i].y)
				star[i].ym -= 0xAA;
			else
				star[i].ym += 0xAA;
		}

		if (star[i].xm > 0xA00)
			star[i].xm = 0xA00;
		if (star[i].xm < -0xA00)
			star[i].xm = -0xA00;

		if (star[i].ym > 0xA00)
			star[i].ym = 0xA00;
		if (star[i].ym < -0xA00)
			star[i].ym = -0xA00;

		// Duplicate of the past 8 lines of code
		if (star[i].xm > 0xA00)
			star[i].xm = 0xA00;
		if (star[i].xm < -0xA00)
			star[i].xm = -0xA00;

		if (star[i].ym > 0xA00)
			star[i].ym = 0xA00;
		if (star[i].ym < -0xA00)
			star[i].ym = -0xA00;

		star[i].x += star[i].xm;
		star[i].y += star[i].ym;

		if (i < gMC.star && (gMC.equip & EQUIP_WHIMSICAL_STAR) && (g_GameFlags & 2) && a == i)
			SetBullet(45, star[a].x, star[a].y, 0);
	}
}

void PutStar(int fx, int fy)
{
	RECT rc[3] = {
		{192, 0, 200, 8},
		{192, 8, 200, 16},
		{192, 16, 200, 24},
	};

	int i;

	if (gMC.cond & 2)
		return;

	if (!(gMC.equip & EQUIP_WHIMSICAL_STAR))
		return;

	for (i = 0; i < 3; ++i)
		if (i < gMC.star)
			PutBitmap3(&grcGame, (star[i].x / 0x200) - (fx / 0x200) - 4, (star[i].y / 0x200) - (fy / 0x200) - 4, &rc[i], SURFACE_ID_MY_CHAR);
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "BossLife.h"

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Boss.h"
#include "Draw.h"
#include "NpChar.h"

BOSSLIFE gBL;

void InitBossLife(void)
{
	gBL.flag = FALSE;
}

BOOL StartBossLife(int code_event)
{
	int i = 0;
	while (i < NPC_MAX && gNPC[i].code_event != code_event)
		++i;

	if (i == NPC_MAX)
		return FALSE;

	gBL.flag = TRUE;
	gBL.max = gNPC[i].life;
	gBL.br = gNPC[i].life;
	gBL.pLife = &gNPC[i].life;
	return TRUE;
}

BOOL StartBossLife2(void)
{
	gBL.flag = TRUE;
	gBL.max = gBoss[0].life;
	gBL.br = gBoss[0].life;
	gBL.pLife = &gBoss[0].life;
	return TRUE;
}

void PutBossLife(void)
{
	RECT rcText = {0, 48, 32, 56};
	RECT rcBox1 = {0, 0, 244, 8};
	RECT rcBox2 = {0, 16, 244, 24};
	RECT rcLife = {0, 24, 0, 32};
	RECT rcBr = {0, 32, 232, 40};

	if (gBL.flag == FALSE)
		return;

	if (*gBL.pLife < 1)
	{
		gBL.flag = FALSE;
		return;
	}

	rcLife.right = (*gBL.pLife * 198) / gBL.max;

	if (gBL.br > *gBL.pLife)
	{
		if (++gBL.count > 30)
			--gBL.br;
	}
	else
	{
		gBL.count = 0;
	}

	rcBr.right = (gBL.br * 198) / gBL.max;

	PutBitmap3(&grcGame, (WINDOW_WIDTH / 2) - 128, WINDOW_HEIGHT - 20, &rcBox1, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, (WINDOW_WIDTH / 2) - 128, WINDOW_HEIGHT - 12, &rcBox2, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, (WINDOW_WIDTH / 2) - 88, WINDOW_HEIGHT - 16, &rcBr, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, (WINDOW_WIDTH / 2) - 88, WINDOW_HEIGHT - 16, &rcLife, SURFACE_ID_TEXT_BOX);
	PutBitmap3(&grcGame, (WINDOW_WIDTH / 2) - 120, WINDOW_HEIGHT - 16, &rcText, SURFACE_ID_TEXT_BOX);
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "SelStage.h"

#include <string.h>

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Draw.h"
#include "Escape.h"
#include "KeyControl.h"
#include "Main.h"
#include "Sound.h"
#include "TextScr.h"

PERMIT_STAGE gPermitStage[STAGE_MAX];

int gSelectedStage;
int gStageSelectTitleY;

void ClearPermitStage(void)
{
	memset(gPermitStage, 0, sizeof(gPermitStage));
}

BOOL AddPermitStage(int index, int event)
{
	int i = 0;

	while (i < STAGE_MAX)
	{
		if (gPermitStage[i].index == index)
			break;

		if (gPermitStage[i].index == 0)
			break;

		++i;
	}

	if (i == STAGE_MAX)
		return FALSE;

	gPermitStage[i].index = index;
	gPermitStage[i].event = event;

	return TRUE;
}

BOOL SubPermitStage(int index)
{
	int i;

	for (i = 0; i < STAGE_MAX; ++i)
		if (gPermitStage[i].index == index)
			break;

#ifdef FIX_BUGS
	if (i == STAGE_MAX)
#else
	if (i == 32) // Same value as 'ITEM_MAX'
#endif
		return FALSE;

	for (++i; i < STAGE_MAX; ++i)
		gPermitStage[i - 1] = gPermitStage[i];

	gPermitStage[i - 1].index = 0;
	gPermitStage[i - 1].event = 0;

	return TRUE;
}

void MoveStageSelectCursor(void)
{
	int stage_num;
	int stage_x;

	stage_num = 0;
	while (gPermitStage[stage_num].index != 0)
		++stage_num;

	stage_x = (WINDOW_WIDTH - (stage_num * 40)) / 2;	// Unused

	if (stage_num == 0)
		return;

	if (gKeyTrg & gKeyLeft)
		--gSelectedStage;

	if (gKeyTrg & gKeyRight)
		++gSelectedStage;

	if (gSelectedStage < 0)
		gSelectedStage = stage_num - 1;

	if (gSelectedStage > stage_num - 1)
		gSelectedStage = 0;

	if (gKeyTrg & (gKeyLeft | gKeyRight))
		StartTextScript(gPermitStage[gSelectedStage].index + 1000);

	if (gKeyTrg & (gKeyLeft | gKeyRight))
		PlaySoundObject(1, SOUND_MODE_PLAY);
}

void PutStageSelectObject(void)
{
	static unsigned int flash;

	int i;
	RECT rcStage;

	RECT rcView = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};

	RECT rcCur[2] = {
		{80, 88, 112, 104},
		{80, 104, 112, 120},
	};

	RECT rcTitle1 = {80, 64, 144, 72};

	int stage_num;
	int stage_x;

	if (gStageSelectTitleY > (WINDOW_HEIGHT / 2) - 74)
		--gStageSelectTitleY;

	PutBitmap3(&rcView, (WINDOW_WIDTH / 2) - 32, gStageSelectTitleY, &rcTitle1, SURFACE_ID_TEXT_BOX);

	stage_num = 0;
	while (gPermitStage[stage_num].index)
		++stage_num;

	++flash;

	if (stage_num != 0)
	{
		stage_x = (WINDOW_WIDTH - (stage_num * 40)) / 2;

		PutBitmap3(&rcView, stage_x + (gSelectedStage * 40), (WINDOW_HEIGHT / 2) - 56, &rcCur[flash / 2 % 2], SURFACE_ID_TEXT_BOX);

		for (i = 0; i < STAGE_MAX; ++i)
		{
			if (gPermitStage[i].index == 0)
				break;

			// Interestingly, there's code for reading multiple rows of icons
			// from the 'StageImage.pbm' file when there are more than 8 stages,
			// despite only 6 icons ever being used.
			rcStage.left = (gPermitStage[i].index % 8) * 32;
			rcStage.right = rcStage.left + 32;
			rcStage.top = (gPermitStage[i].index / 8) * 16;
			rcStage.bottom = rcStage.top + 16;

			PutBitmap3(&rcView, stage_x + (i * 40), (WINDOW_HEIGHT / 2) - 56, &rcStage, SURFACE_ID_STAGE_ITEM);
		}
	}
}

int StageSelectLoop(int *p_event)
{
	char old_script_path[MAX_PATH];

	RECT rcView = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};

	gSelectedStage = 0;
	BackupSurface(SURFACE_ID_SCREEN_GRAB, &grcFull);
	GetTextScriptPath(old_script_path);
	LoadTextScript2("StageSelect.tsc");
	gStageSelectTitleY = (WINDOW_HEIGHT / 2) - 66;
	StartTextScript(gPermitStage[gSelectedStage].index + 1000);

	for (;;)
	{
		GetTrg();

		if (gKey & KEY_ESCAPE)
		{
			switch (Call_Escape(ghWnd))
			{
				case enum_ESCRETURN_exit:
					return enum_ESCRETURN_exit;

				case enum_ESCRETURN_restart:
					return enum_ESCRETURN_restart;
			}
		}

		MoveStageSelectCursor();

		switch (TextScriptProc())
		{
			case enum_ESCRETURN_exit:
				return enum_ESCRETURN_exit;

			case enum_ESCRETURN_restart:
				return enum_ESCRETURN_restart;
		}

#ifdef FIX_BUGS
		PutBitmap4(&rcView, 0, 0, &rcView, SURFACE_ID_SCREEN_GRAB);
#else
		// The original accidentally drew the screencap with transparency enabled
		PutBitmap3(&rcView, 0, 0, &rcView, SURFACE_ID_SCREEN_GRAB);
#endif
		PutStageSelectObject();
		PutTextScript();

		if (gKeyTrg & gKeyOk)
		{
			StopTextScript();
			break;
		}

		if (gKeyTrg & gKeyCancel)
		{
			StopTextScript();
			LoadTextScript_Stage(old_script_path);
			*p_event = 0;
			return enum_ESCRETURN_continue;
		}

		PutFramePerSecound();

		if (!Flip_SystemTask(ghWnd))
			return enum_ESCRETURN_exit;
	}

	LoadTextScript_Stage(old_script_path);
	*p_event = gPermitStage[gSelectedStage].event;
	return enum_ESCRETURN_continue;
}

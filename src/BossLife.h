// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef struct BOSSLIFE	// Not the original struct name
{
	BOOL flag;
	int *pLife;
	int max;
	int br;
	int count;
} BOSSLIFE;

extern BOSSLIFE gBL;

void InitBossLife(void);
BOOL StartBossLife(int code_event);
BOOL StartBossLife2(void);
void PutBossLife(void);

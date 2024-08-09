// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef struct MAP_NAME
{
	BOOL flag;
	int wait;
	char name[0x20];
} MAP_NAME;

extern MAP_NAME gMapName;

void ReadyMapName(const char *str);
void PutMapName(BOOL bMini);
void StartMapName(void);
void RestoreMapName(void);

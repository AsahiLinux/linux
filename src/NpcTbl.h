// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

#include "NpChar.h"

struct NPC_TBL_RECT
{
	unsigned char front;
	unsigned char top;
	unsigned char back;
	unsigned char bottom;
};

struct NPC_TABLE
{
	unsigned short bits;
	unsigned short life;
	unsigned char surf;
	unsigned char hit_voice;
	unsigned char destroy_voice;
	unsigned char size;
	long exp;
	long damage;
	NPC_TBL_RECT hit;
	NPC_TBL_RECT view;
};

extern NPC_TABLE *gNpcTable;

BOOL LoadNpcTable(const char *path);
void ReleaseNpcTable(void);

// NPC Function table
typedef void (*NPCFUNCTION)(NPCHAR*);
extern const NPCFUNCTION gpNpcFuncTbl[];

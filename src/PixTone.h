// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef struct PIXTONEPARAMETER2
{
	int model;
	double num;
	int top;
	int offset;
} PIXTONEPARAMETER2;

typedef struct PIXTONEPARAMETER
{
	int use;
	int size;
	PIXTONEPARAMETER2 oMain;
	PIXTONEPARAMETER2 oPitch;
	PIXTONEPARAMETER2 oVolume;
	int initial;
	int pointAx;
	int pointAy;
	int pointBx;
	int pointBy;
	int pointCx;
	int pointCy;
} PIXTONEPARAMETER;

extern signed char gWaveModelTable[6][0x100];

void MakeWaveTables(void);
BOOL MakePixelWaveData(const PIXTONEPARAMETER *ptp, unsigned char *pData);

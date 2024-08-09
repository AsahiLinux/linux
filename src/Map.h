// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef struct MAP_DATA
{
	unsigned char *data;
	unsigned char atrb[0x100];
	short width;
	short length;
} MAP_DATA;

extern MAP_DATA gMap;

BOOL InitMapData2(void);
BOOL LoadMapData2(const char *path_map);
BOOL LoadAttributeData(const char *path_atrb);
void EndMapData(void);
void ReleasePartsImage(void);
void GetMapData(unsigned char **data, short *mw, short *ml);
unsigned char GetAttribute(int x, int y);
void DeleteMapParts(int x, int y);
void ShiftMapParts(int x, int y);
BOOL ChangeMapParts(int x, int y, unsigned char no);
void PutStage_Back(int fx, int fy);
void PutStage_Front(int fx, int fy);
void PutMapDataVector(int fx, int fy);

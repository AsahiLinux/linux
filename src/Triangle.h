// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

extern int gSin[0x100];
extern short gTan[0x21];

void InitTriangleTable(void);
int GetSin(unsigned char deg);
int GetCos(unsigned char deg);
unsigned char GetArktan(int x, int y);

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

extern signed char gMapping[0x80];

int MiniMapLoop(void);
BOOL IsMapping(void);
void StartMapping(void);
void SetMapping(int a);

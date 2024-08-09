// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

#include "NpChar.h"

void JadgeHitNpCharBlock(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleA(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleB(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleC(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleD(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleE(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleF(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleG(NPCHAR *npc, int x, int y);
void JudgeHitNpCharTriangleH(NPCHAR *npc, int x, int y);
void JudgeHitNpCharWater(NPCHAR *npc, int x, int y);
void HitNpCharMap(void);
void LoseNpChar(NPCHAR *npc, BOOL bVanish);
void HitNpCharBullet(void);

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "NpChar.h"

#define BOSS_MAX 20

extern NPCHAR gBoss[BOSS_MAX];

typedef void (*BOSSFUNCTION)(void);
extern BOSSFUNCTION gpBossFuncTbl[10];

void InitBossChar(int code);
void PutBossChar(int fx, int fy);
void SetBossCharActNo(int a);
void HitBossBullet(void);
void ActBossChar(void);
void HitBossMap(void);

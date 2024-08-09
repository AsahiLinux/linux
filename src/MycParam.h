// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef struct ARMS_LEVEL
{
	int exp[3];
} ARMS_LEVEL;

typedef struct REC
{
	long counter[4];
	unsigned char random[4];
} REC;

extern ARMS_LEVEL gArmsLevelTable[14];

void AddExpMyChar(int x);
void ZeroExpMyChar(void);
BOOL IsMaxExpMyChar(void);
void DamageMyChar(int damage);
void ZeroArmsEnergy_All(void);
void AddBulletMyChar(int no, int val);
void AddLifeMyChar(int x);
void AddMaxLifeMyChar(int val);
void PutArmsEnergy(BOOL flash);
void PutActiveArmsList(void);
void PutMyLife(BOOL flash);
void PutMyAir(int x, int y);
void PutTimeCounter(int x, int y);
BOOL SaveTimeCounter(void);
int LoadTimeCounter(void);

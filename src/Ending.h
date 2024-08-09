// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

#include "CommonDefines.h"

#define MAX_STRIP ((WINDOW_HEIGHT / 16) + 1)

void ActionStripper(void);
void PutStripper(void);
void SetStripper(int x, int y, const char *text, int cast);
void RestoreStripper(void);
void ActionIllust(void);
void PutIllust(void);
void ReloadIllust(int a);
void InitCreditScript(void);
void ReleaseCreditScript(void);
BOOL StartCreditScript(void);
void ActionCredit(void);
void SetCreditIllust(int a);
void CutCreditIllust(void);
int Scene_DownIsland(HWND hWnd, int mode);

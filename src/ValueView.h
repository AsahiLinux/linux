// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

#define VALUEVIEW_MAX 0x10

typedef struct VALUEVIEW
{
  BOOL flag;
  int *px;
  int *py;
  int offset_y;
  int value;
  int count;
  RECT rect;
} VALUEVIEW;

extern VALUEVIEW gVV[VALUEVIEW_MAX];
extern int gVVIndex;

void ClearValueView(void);
void SetValueView(int *px, int *py, int value);
void ActValueView(void);
void PutValueView(int flx, int fly);

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

typedef struct FRAME
{
	int x;
	int y;
	int *tgt_x;
	int *tgt_y;
	int wait;
	int quake;
	int quake2;
} FRAME;

extern FRAME gFrame;

void MoveFrame3(void);
void GetFramePosition(int *fx, int *fy);
void SetFramePosition(int fx, int fy);
void SetFrameMyChar(void);
void SetFrameTargetMyChar(int wait);
void SetFrameTargetNpChar(int event, int wait);
void SetFrameTargetBoss(int no, int wait);
void SetQuake(int time);
void SetQuake2(int time);
void ResetQuake(void);

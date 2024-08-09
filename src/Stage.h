// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

typedef enum MusicID
{
	MUS_SILENCE = 0x0,
	MUS_MISCHIEVOUS_ROBOT = 0x1,
	MUS_SAFETY = 0x2,
	MUS_GAME_OVER = 0x3,
	MUS_GRAVITY = 0x4,
	MUS_ON_TO_GRASSTOWN = 0x5,
	MUS_MELTDOWN2 = 0x6,
	MUS_EYES_OF_FLAME = 0x7,
	MUS_GESTATION = 0x8,
	MUS_MIMIGA_TOWN = 0x9,
	MUS_GOT_ITEM = 0xA,
	MUS_BALROGS_THEME = 0xB,
	MUS_CEMETERY = 0xC,
	MUS_PLANT = 0xD,
	MUS_PULSE = 0xE,
	MUS_VICTORY = 0xF,
	MUS_GET_HEART_TANK = 0x10,
	MUS_TYRANT = 0x11,
	MUS_RUN = 0x12,
	MUS_JENKA1 = 0x13,
	MUS_LABYRINTH_FIGHT = 0x14,
	MUS_ACCESS = 0x15,
	MUS_OPPRESSION = 0x16,
	MUS_GEOTHERMAL = 0x17,
	MUS_CAVE_STORY = 0x18,
	MUS_MOONSONG = 0x19,
	MUS_HEROS_END = 0x1A,
	MUS_SCORCHING_BACK = 0x1B,
	MUS_QUIET = 0x1C,
	MUS_LAST_CAVE = 0x1D,
	MUS_BALCONY = 0x1E,
	MUS_CHARGE = 0x1F,
	MUS_LAST_BATTLE = 0x20,
	MUS_THE_WAY_BACK_HOME = 0x21,
	MUS_ZOMBIE = 0x22,
	MUS_BREAK_DOWN = 0x23,
	MUS_RUNNING_HELL = 0x24,
	MUS_JENKA2 = 0x25,
	MUS_LIVING_WATERWAY = 0x26,
	MUS_SEAL_CHAMBER = 0x27,
	MUS_TOROKOS_THEME = 0x28,
	MUS_WHITE = 0x29
} MusicID;

typedef struct STAGE_TABLE
{
	char parts[0x20];
	char map[0x20];
	int bkType;
	char back[0x20];
	char npc[0x20];
	char boss[0x20];
	signed char boss_no;
	char name[0x20];
} STAGE_TABLE;

extern int gStageNo;
extern MusicID gMusicNo;
extern unsigned int gOldPos;
extern MusicID gOldNo;

extern const STAGE_TABLE gTMT[95];

extern const char* const gMusicTable[42];

BOOL TransferStage(int no, int w, int x, int y);
void ChangeMusic(MusicID no);
void ReCallMusic(void);

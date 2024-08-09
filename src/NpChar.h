// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Draw.h"

#define NPC_MAX 0x200

enum NPCCond
{
	NPCCOND_DAMAGE_BOSS = 0x10, // (gBoss npc exclusive) When set, damage the main boss
	NPCCOND_ALIVE = 0x80        // Whether the NPC is alive or not
};

// Be careful when changing these: they're baked into the 'npc.tbl' file
enum NPCFlags
{
	NPC_SOLID_SOFT = 1<<0,                // Pushes Quote out
	NPC_IGNORE_TILE_44 = 1<<1,            // Ignores tile 44, which normally blocks NPCs
	NPC_INVULNERABLE = 1<<2,              // Can't be hurt
	NPC_IGNORE_SOLIDITY = 1<<3,           // Doesn't collide with anything
	NPC_BOUNCY = 1<<4,                    // Quote bounces on top of NPC
	NPC_SHOOTABLE = 1<<5,                 // Can be shot
	NPC_SOLID_HARD = 1<<6,                // Essentially acts as level tiles
	NPC_REAR_AND_TOP_DONT_HURT = 1<<7,    // Rear and top don't hurt when touched
	NPC_EVENT_WHEN_TOUCHED = 1<<8,        // Run event when touched
	NPC_EVENT_WHEN_KILLED = 1<<9,         // Run event when killed
	NPC_APPEAR_WHEN_FLAG_SET = 1<<11,     // Only appear when flag is set
	NPC_SPAWN_IN_OTHER_DIRECTION = 1<<12, // Spawn facing to the right (or however the NPC interprets the direction)
	NPC_INTERACTABLE = 1<<13,             // Run event when interacted with
	NPC_HIDE_WHEN_FLAG_SET = 1<<14,       // Hide when flag is set
	NPC_SHOW_DAMAGE = 1<<15               // Show the number of damage taken when harmed
};

enum NPCNames
{
	NPC_NULL = 0,
	NPC_EXP = 1,
	NPC_ENEMY_BEHEMOTH = 2,
	NPC_DAMAGE_TEXT_HOLDER = 3,
	NPC_SMOKE = 4,
	// To be continued
	NPC_ENEMY_FROG = 104,
	NPC_SPEECH_BALLOON_HEY_LOW = 105,
	NPC_SPEECH_BALLOON_HEY_HIGH = 106,
	NPC_MALCO_UNDAMAGED = 107,
	NPC_PROJECTILE_BALFROG_SPITBALL = 108,
	NPC_MALCO_DAMAGED = 109,
	NPC_ENEMY_PUCHI = 110,
	// To be continued
	NPC_KINGS_SWORD = 145
};

typedef struct NPCHAR
{
	unsigned char cond;
	int flag;
	int x;
	int y;
	int xm;
	int ym;
	int xm2;
	int ym2;
	int tgt_x;
	int tgt_y;
	int code_char;
	int code_flag;
	int code_event;
	SurfaceID surf;
	int hit_voice;
	int destroy_voice;
	int life;
	int exp;
	int size;
	int direct;
	unsigned short bits;
	RECT rect;
	int ani_wait;
	int ani_no;
	int count1;
	int count2;
	int act_no;
	int act_wait;
	OTHER_RECT hit;
	OTHER_RECT view;
	unsigned char shock;
	int damage_view;
	int damage;
	struct NPCHAR *pNpc;
} NPCHAR;

struct EVENT
{
	short x;
	short y;
	short code_flag;
	short code_event;
	short code_char;
	unsigned short bits;
};

extern NPCHAR gNPC[NPC_MAX];
extern int gCurlyShoot_wait;
extern int gCurlyShoot_x;
extern int gCurlyShoot_y;
extern int gSuperXpos;
extern int gSuperYpos;

extern const char* const gPassPixEve;

void InitNpChar(void);
BOOL LoadEvent(const char *path_event);
void SetNpChar(int code_char, int x, int y, int xm, int ym, int dir, NPCHAR *npc, int start_index);
void SetDestroyNpChar(int x, int y, int w, int num);
void SetDestroyNpCharUp(int x, int y, int w, int num);
void SetExpObjects(int x, int y, int exp);
BOOL SetBulletObject(int x, int y, int val);
BOOL SetLifeObject(int x, int y, int val);
void VanishNpChar(NPCHAR *npc);
void PutNpChar(int fx, int fy);
void ActNpChar(void);
void ChangeNpCharByEvent(int code_event, int code_char, int dir);
void ChangeCheckableNpCharByEvent(int code_event, int code_char, int dir);
void SetNpCharActionNo(int code_event, int act_no, int dir);
void MoveNpChar(int code_event, int x, int y, int dir);
void BackStepMyChar(int code_event);
void DeleteNpCharEvent(int code);
void DeleteNpCharCode(int code, BOOL bSmoke);
void GetNpCharPosition(int *x, int *y, int i);
BOOL IsNpCharCode(int code);
BOOL GetNpCharAlive(int code_event);
int CountAliveNpChar(void);

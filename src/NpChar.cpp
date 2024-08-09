// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "NpChar.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "WindowsWrapper.h"

#include "ArmsItem.h"
#include "Caret.h"
#include "CommonDefines.h"
#include "Draw.h"
#include "Flags.h"
#include "Game.h"
#include "Main.h"
#include "MyChar.h"
#include "NpcTbl.h"
#include "Sound.h"
#include "ValueView.h"

NPCHAR gNPC[NPC_MAX];
int gCurlyShoot_wait;
int gCurlyShoot_x;
int gCurlyShoot_y;
int gSuperXpos;
int gSuperYpos;

const char* const gPassPixEve = "PXE";

static void SetUniqueParameter(NPCHAR *npc)
{
	int code = npc->code_char;
	npc->surf = (SurfaceID)gNpcTable[code].surf;
	npc->hit_voice = gNpcTable[code].hit_voice;
	npc->destroy_voice = gNpcTable[code].destroy_voice;
	npc->damage = gNpcTable[code].damage;
	npc->size = gNpcTable[code].size;
	npc->life = gNpcTable[code].life;
	npc->hit.front = gNpcTable[code].hit.front * 0x200;
	npc->hit.back = gNpcTable[code].hit.back * 0x200;
	npc->hit.top = gNpcTable[code].hit.top * 0x200;
	npc->hit.bottom = gNpcTable[code].hit.bottom * 0x200;
	npc->view.front = gNpcTable[code].view.front * 0x200;
	npc->view.back = gNpcTable[code].view.back * 0x200;
	npc->view.top = gNpcTable[code].view.top * 0x200;
	npc->view.bottom = gNpcTable[code].view.bottom * 0x200;
}

void InitNpChar(void)
{
	memset(gNPC, 0, sizeof(gNPC));
}

BOOL LoadEvent(const char *path_event)
{
	int i, n;
	FILE *fp;
	int count;
	char code[4];
	EVENT eve;

	char path[MAX_PATH];
	sprintf(path, "%s\\%s", gDataPath, path_event);

	fp = fopen(path, "rb");
	if (fp == NULL)
		return FALSE;

	// Read "PXE" check
	fread(code, 1, 4, fp);
	if (memcmp(code, gPassPixEve, 3) != 0)
	{
#ifdef FIX_MAJOR_BUGS
		// The original game forgot to close the file here
		fclose(fp);
#endif
		return FALSE;
	}

	// Get amount of NPCs
	fread(&count, 4, 1, fp);

	// Load NPCs
	memset(gNPC, 0, sizeof(gNPC));

	n = 170;
	for (i = 0; i < count; ++i)
	{
		// Get data from file
		fread(&eve, sizeof(EVENT), 1, fp);

		// Set NPC parameters
		gNPC[n].direct = (eve.bits & NPC_SPAWN_IN_OTHER_DIRECTION) ? 2 : 0;
		gNPC[n].code_char = eve.code_char;
		gNPC[n].code_event = eve.code_event;
		gNPC[n].code_flag = eve.code_flag;
		gNPC[n].x = eve.x * 0x10 * 0x200;
		gNPC[n].y = eve.y * 0x10 * 0x200;
		gNPC[n].bits = eve.bits;
		gNPC[n].bits |= gNpcTable[gNPC[n].code_char].bits;
		gNPC[n].exp = gNpcTable[gNPC[n].code_char].exp;
		SetUniqueParameter(&gNPC[n]);

		// Check flags
		if (gNPC[n].bits & NPC_APPEAR_WHEN_FLAG_SET)
		{
			if (GetNPCFlag(gNPC[n].code_flag) == TRUE)
				gNPC[n].cond |= 0x80;
		}
		else if (gNPC[n].bits & NPC_HIDE_WHEN_FLAG_SET)
		{
			if (GetNPCFlag(gNPC[n].code_flag) == FALSE)
				gNPC[n].cond |= 0x80;
		}
		else
		{
			gNPC[n].cond = 0x80;
		}

		// Increase index
		++n;
	}

	fclose(fp);
	return TRUE;
}

void SetNpChar(int code_char, int x, int y, int xm, int ym, int dir, NPCHAR *npc, int start_index)
{
	int n = start_index;
	while (n < NPC_MAX && gNPC[n].cond)
		++n;

	if (n == NPC_MAX)
		return;

	// Set NPC parameters
	memset(&gNPC[n], 0, sizeof(NPCHAR));
	gNPC[n].cond |= 0x80;
	gNPC[n].direct = dir;
	gNPC[n].code_char = code_char;
	gNPC[n].x = x;
	gNPC[n].y = y;
	gNPC[n].xm = xm;
	gNPC[n].ym = ym;
	gNPC[n].pNpc = npc;
	gNPC[n].bits = gNpcTable[gNPC[n].code_char].bits;
	gNPC[n].exp = gNpcTable[gNPC[n].code_char].exp;
	SetUniqueParameter(&gNPC[n]);
}

void SetDestroyNpChar(int x, int y, int w, int num)
{
	int i;
	int offset_x;
	int offset_y;

	// Create smoke
	w /= 0x200;
	for (i = 0; i < num; ++i)
	{
		offset_x = Random(-w, w) * 0x200;
		offset_y = Random(-w, w) * 0x200;
		SetNpChar(NPC_SMOKE, x + offset_x, y + offset_y, 0, 0, 0, NULL, 0x100);
	}

	// Flash effect
	SetCaret(x, y, CARET_EXPLOSION, DIR_LEFT);
}

void SetDestroyNpCharUp(int x, int y, int w, int num)
{
	int i;
	int offset_x;
	int offset_y;

	// Create smoke
	w /= 0x200;
	for (i = 0; i < num; ++i)
	{
		offset_x = Random(-w, w) * 0x200;
		offset_y = Random(-w, w) * 0x200;
		SetNpChar(4, x + offset_x, y + offset_y, 0, 0, 1, NULL, 0x100);
	}

	// Flash effect
	SetCaret(x, y, CARET_EXPLOSION, DIR_LEFT);
}

void SetExpObjects(int x, int y, int exp)
{
	int n;
	int sub_exp;

	n = 0x100;
	while (exp)
	{
		while (n < NPC_MAX && gNPC[n].cond)
			++n;

		if (n == NPC_MAX)
			break;

		memset(&gNPC[n], 0, sizeof(NPCHAR));

		if (exp >= 20)
		{
			exp -= 20;
			sub_exp = 20;
		}
		else if (exp >= 5)
		{
			exp -= 5;
			sub_exp = 5;
		}
		else if (exp >= 1)
		{
			exp -= 1;
			sub_exp = 1;
		}

		gNPC[n].cond |= 0x80;
		gNPC[n].direct = 0;
		gNPC[n].code_char = 1;
		gNPC[n].x = x;
		gNPC[n].y = y;
		gNPC[n].bits = gNpcTable[gNPC[n].code_char].bits;
		gNPC[n].exp = sub_exp;

		SetUniqueParameter(&gNPC[n]);
	}
}

BOOL SetBulletObject(int x, int y, int val)
{
	int n;
	int bullet_no;	// The Linux debug data claims there's a 3-line gap between this and the next variable declaration. Just enough space for an 'if' statement.

	// if (/* unknown */)
	{	// This is necessary for accurate ASM (stack frame layout)
		int tamakazu_ari[10];
		int t = 0;

		memset(tamakazu_ari, 0, sizeof(tamakazu_ari));

		for (n = 0; n < 8; ++n)
		{
			switch (gArmsData[n].code)
			{
				case 5:
					tamakazu_ari[t++] = 0;
					break;

				case 10:
					tamakazu_ari[t++] = 1;
					break;

				default:
					tamakazu_ari[t] = 0;
					break;
			}
		}

		if (t == 0)
			return FALSE;

		n = Random(1, 10 * t);
		bullet_no = tamakazu_ari[n % t];

		n = 0x100;
		while (n < NPC_MAX && gNPC[n].cond)
			++n;

		if (n == NPC_MAX)
			return FALSE;

		memset(&gNPC[n], 0, sizeof(NPCHAR));
		gNPC[n].cond |= 0x80;
		gNPC[n].direct = 0;
		gNPC[n].code_event = bullet_no;
		gNPC[n].code_char = 86;
		gNPC[n].x = x;
		gNPC[n].y = y;
		gNPC[n].bits = gNpcTable[gNPC[n].code_char].bits;
		gNPC[n].exp = val;
		SetUniqueParameter(&gNPC[n]);
	}

	return TRUE;
}

BOOL SetLifeObject(int x, int y, int val)
{
	int n = 0x100;
	while (n < NPC_MAX && gNPC[n].cond)
		++n;

	if (n == NPC_MAX)
		return FALSE;

	memset(&gNPC[n], 0, sizeof(NPCHAR));
	gNPC[n].cond |= 0x80;
	gNPC[n].direct = 0;
	gNPC[n].code_char = 87;
	gNPC[n].x = x;
	gNPC[n].y = y;
	gNPC[n].bits = gNpcTable[gNPC[n].code_char].bits;
	gNPC[n].exp = val;
	SetUniqueParameter(&gNPC[n]);
	return TRUE;
}

void VanishNpChar(NPCHAR *npc)
{
	int x, y;

	x = npc->x;
	y = npc->y;
	memset(npc, 0, sizeof(NPCHAR));
	npc->count1 = 0;
	npc->x = x;
	npc->y = y;
	npc->cond |= 0x80;
	npc->direct = 0;
	npc->code_char = 3;
	npc->bits = gNpcTable[npc->code_char].bits;
	npc->exp = gNpcTable[npc->code_char].exp;
	SetUniqueParameter(npc);
}

void PutNpChar(int fx, int fy)
{
	int n;
	signed char a = 0;

	int side;

	for (n = 0; n < NPC_MAX; ++n)
	{
		if (gNPC[n].cond & 0x80)
		{
			if (gNPC[n].shock)
			{
				a = 2 * ((gNPC[n].shock / 2) % 2) - 1;
			}
			else
			{
				a = 0;
				if (gNPC[n].bits & NPC_SHOW_DAMAGE && gNPC[n].damage_view)
				{
					SetValueView(&gNPC[n].x, &gNPC[n].y, gNPC[n].damage_view);
					gNPC[n].damage_view = 0;
				}
			}

			if (gNPC[n].direct == 0)
				side = gNPC[n].view.front;
			else
				side = gNPC[n].view.back;

			PutBitmap3(
				&grcGame,
				(gNPC[n].x - side) / 0x200 - fx / 0x200 + a,
				(gNPC[n].y - gNPC[n].view.top) / 0x200 - fy / 0x200,
				&gNPC[n].rect,
				(SurfaceID)gNPC[n].surf);
		}
	}
}

void ActNpChar(void)
{
	int i;
	int code_char;

	for (i = 0; i < NPC_MAX; ++i)
	{
		if (gNPC[i].cond & 0x80)
		{
			code_char = gNPC[i].code_char;

			gpNpcFuncTbl[code_char](&gNPC[i]);

			if (gNPC[i].shock)
				--gNPC[i].shock;
		}
	}
}

void ChangeNpCharByEvent(int code_event, int code_char, int dir)
{
	int n;

	for (n = 0; n < NPC_MAX; ++n)
	{
		if ((gNPC[n].cond & 0x80) && gNPC[n].code_event == code_event)
		{
			gNPC[n].bits &= ~(NPC_SOLID_SOFT | NPC_IGNORE_TILE_44 | NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_BOUNCY | NPC_SHOOTABLE | NPC_SOLID_HARD | NPC_REAR_AND_TOP_DONT_HURT | NPC_SHOW_DAMAGE);	// Clear these flags
			gNPC[n].code_char = code_char;
			gNPC[n].bits |= gNpcTable[gNPC[n].code_char].bits;
			gNPC[n].exp = gNpcTable[gNPC[n].code_char].exp;
			SetUniqueParameter(&gNPC[n]);
			gNPC[n].cond |= 0x80;
			gNPC[n].act_no = 0;
			gNPC[n].act_wait = 0;
			gNPC[n].count1 = 0;
			gNPC[n].count2 = 0;
			gNPC[n].ani_no = 0;
			gNPC[n].ani_wait = 0;
			gNPC[n].xm = 0;
			gNPC[n].ym = 0;

			if (dir == 5)
			{
				// Another empty case that has to exist for the same assembly to be generated
			}
			else if (dir == 4)
			{
				if (gNPC[n].x < gMC.x)
					gNPC[n].direct = 2;
				else
					gNPC[n].direct = 0;
			}
			else
			{
				gNPC[n].direct = dir;
			}

			gpNpcFuncTbl[code_char](&gNPC[n]);
		}
	}
}

void ChangeCheckableNpCharByEvent(int code_event, int code_char, int dir)
{
	int n;

	for (n = 0; n < NPC_MAX; ++n)
	{
		if (!(gNPC[n].cond & 0x80) && gNPC[n].code_event == code_event)
		{
			gNPC[n].bits &= ~(NPC_SOLID_SOFT | NPC_IGNORE_TILE_44 | NPC_INVULNERABLE | NPC_IGNORE_SOLIDITY | NPC_BOUNCY | NPC_SHOOTABLE | NPC_SOLID_HARD | NPC_REAR_AND_TOP_DONT_HURT | NPC_SHOW_DAMAGE);	// Clear these flags
			gNPC[n].bits |= NPC_INTERACTABLE;
			gNPC[n].code_char = code_char;
			gNPC[n].bits |= gNpcTable[gNPC[n].code_char].bits;
			gNPC[n].exp = gNpcTable[gNPC[n].code_char].exp;
			SetUniqueParameter(&gNPC[n]);
			gNPC[n].cond |= 0x80;
			gNPC[n].act_no = 0;
			gNPC[n].act_wait = 0;
			gNPC[n].count1 = 0;
			gNPC[n].count2 = 0;
			gNPC[n].ani_no = 0;
			gNPC[n].ani_wait = 0;
			gNPC[n].xm = 0;
			gNPC[n].ym = 0;

			if (dir == 5)
			{
				// Another empty case that has to exist for the same assembly to be generated
			}
			else if (dir == 4)
			{
				if (gNPC[n].x < gMC.x)
					gNPC[n].direct = 2;
				else
					gNPC[n].direct = 0;
			}
			else
			{
				gNPC[n].direct = (signed char)dir;
			}

			gpNpcFuncTbl[code_char](&gNPC[n]);
		}
	}
}

void SetNpCharActionNo(int code_event, int act_no, int dir)
{
	int n = 0;
	while (n < NPC_MAX)
	{
		if ((gNPC[n].cond & 0x80) && gNPC[n].code_event == code_event)
			break;

		++n;
	}

	if (n == NPC_MAX)
		return;

	gNPC[n].act_no = act_no;

	if (dir == 5)
	{
		// Another empty case that has to exist for the same assembly to be generated
	}
	else if (dir == 4)
	{
		if (gNPC[n].x < gMC.x)
			gNPC[n].direct = 2;
		else
			gNPC[n].direct = 0;
	}
	else
	{
		gNPC[n].direct = dir;
	}
}

void MoveNpChar(int code_event, int x, int y, int dir)
{
	int n = 0;
	while (n < NPC_MAX)
	{
		if ((gNPC[n].cond & 0x80) && gNPC[n].code_event == code_event)
			break;

		++n;
	}

	if (n == NPC_MAX)
		return;

	gNPC[n].x = x;
	gNPC[n].y = y;

	if (dir == 5)
	{
		// Another empty case that has to exist for the same assembly to be generated
	}
	else if (dir == 4)
	{
		if (gNPC[n].x < gMC.x)
			gNPC[n].direct = 2;
		else
			gNPC[n].direct = 0;
	}
	else
	{
		gNPC[n].direct = (signed char)dir;
	}
}

void BackStepMyChar(int code_event)
{
	int n = 0;

	gMC.cond &= ~1;
	gMC.ym = -0x200;

	if (code_event == 0)
	{
		gMC.direct = 0;
		gMC.xm = 0x200;
	}
	else if (code_event == 2)
	{
		gMC.direct = 2;
		gMC.xm = -0x200;
	}
	else
	{
		while (n < NPC_MAX)
		{
			if ((gNPC[n].cond & 0x80) && gNPC[n].code_event == code_event)
				break;

			++n;
		}

		if (n == NPC_MAX)
			return;

		if (gNPC[n].x < gMC.x)
		{
			gMC.direct = 0;
			gMC.xm = 0x200;
		}
		else
		{
			gMC.direct = 2;
			gMC.xm = -0x200;
		}
	}
}

void DeleteNpCharEvent(int code)
{
	int i;

	for (i = 0; i < NPC_MAX; ++i)
	{
		if ((gNPC[i].cond & 0x80) && gNPC[i].code_event == code)
		{
			gNPC[i].cond = 0;
			SetNPCFlag(gNPC[i].code_flag);
		}
	}
}

void DeleteNpCharCode(int code, BOOL bSmoke)
{
	int n;

	for (n = 0; n < NPC_MAX; ++n)
	{
		if ((gNPC[n].cond & 0x80) && gNPC[n].code_char == code)
		{
			gNPC[n].cond = 0;
			SetNPCFlag(gNPC[n].code_flag);

			if (bSmoke)
			{
				PlaySoundObject(gNPC[n].destroy_voice, SOUND_MODE_PLAY);

				switch (gNPC[n].size)
				{
					case 1:
						SetDestroyNpChar(gNPC[n].x, gNPC[n].y, gNPC[n].view.back, 4);
						break;

					case 2:
						SetDestroyNpChar(gNPC[n].x, gNPC[n].y, gNPC[n].view.back, 8);
						break;

					case 3:
						SetDestroyNpChar(gNPC[n].x, gNPC[n].y, gNPC[n].view.back, 16);
						break;
				}
			}
		}
	}
}

void GetNpCharPosition(int *x, int *y, int i)
{
	*x = gNPC[i].x;
	*y = gNPC[i].y;
}

BOOL IsNpCharCode(int code)
{
	int i;

	for (i = 0; i < NPC_MAX; ++i)
		if ((gNPC[i].cond & 0x80) && gNPC[i].code_char == code)
			return TRUE;

	return FALSE;
}

BOOL GetNpCharAlive(int code_event)
{
	int i;

	for (i = 0; i < NPC_MAX; ++i)
		if ((gNPC[i].cond & 0x80) && gNPC[i].code_event == code_event)
			break;

	if (i < NPC_MAX)
		return TRUE;
	else
		return FALSE;
}

int CountAliveNpChar(void)
{
	int n;
	int count = 0;

	for (n = 0; n < NPC_MAX; ++n)
		if (gNPC[n].cond & 0x80)
			++count;

	return count;
}

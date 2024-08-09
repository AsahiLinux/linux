// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "KeyControl.h"

long gKey;
long gKeyTrg;

long gKeyJump = KEY_Z;
long gKeyShot = KEY_X;
long gKeyArms = KEY_ARMS;
long gKeyArmsRev = KEY_ARMSREV;
long gKeyItem = KEY_ITEM;
long gKeyMap = KEY_MAP;

long gKeyOk = KEY_Z;
long gKeyCancel = KEY_X;

long gKeyLeft = KEY_LEFT;
long gKeyUp = KEY_UP;
long gKeyRight = KEY_RIGHT;
long gKeyDown = KEY_DOWN;

void GetTrg(void)
{
	static int key_old;
	gKeyTrg = gKey ^ key_old;
	gKeyTrg = gKey & gKeyTrg;
	key_old = gKey;
}

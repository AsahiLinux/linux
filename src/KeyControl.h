// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

enum KeyBind
{
	//The movement keys go in the order of left, right, up and down
	KEY_LEFT =      0x00000001,
	KEY_RIGHT =     0x00000002,
	KEY_UP =        0x00000004,
	KEY_DOWN =      0x00000008,
	//Map key
	KEY_MAP =       0x00000010,
	//Okay and cancel / Jump and Shoot keys
	KEY_X =         0x00000020,
	KEY_Z =         0x00000040,
	//Left and right weapon switch keys
	KEY_ARMS =      0x00000080,
	KEY_ARMSREV =   0x00000100,
	//Unused?
	KEY_SHIFT =     0x00000200,
	//Function keys
	KEY_F1 =        0x00000400,
	KEY_F2 =        0x00000800,
	//Inventory
	KEY_ITEM =      0x00001000,
	//Escape key
	KEY_ESCAPE =	0x00008000,
	//The alt movement keys go in the order of left, up, right and down
	KEY_ALT_LEFT =  0x00010000,
	KEY_ALT_DOWN =  0x00020000,
	KEY_ALT_RIGHT = 0x00040000,
	KEY_ALT_UP =    0x00180000,
	//Alt up actually has 2 seperate keys, but they're kind of merged together
	KEY_L =         0x00080000,
	KEY_PLUS =      0x00100000
};

extern long gKey;
extern long gKeyTrg;

extern long gKeyJump;
extern long gKeyShot;
extern long gKeyArms;
extern long gKeyArmsRev;
extern long gKeyItem;
extern long gKeyMap;

extern long gKeyOk;
extern long gKeyCancel;

extern long gKeyLeft;
extern long gKeyUp;
extern long gKeyRight;
extern long gKeyDown;

void GetTrg(void);

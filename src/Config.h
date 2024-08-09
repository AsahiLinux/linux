// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

struct CONFIGDATA
{
	char proof[0x20];
	char font_name[0x40];
	long move_button_mode;
	long attack_button_mode;
	long ok_button_mode;
	long display_mode;
	BOOL bJoystick;
	long joystick_button[8];
};

extern const char* const gConfigName;
extern const char* const gProof;

BOOL LoadConfigData(CONFIGDATA *conf);
void DefaultConfigData(CONFIGDATA *conf);

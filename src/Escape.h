// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

enum enum_ESCRETURN
{
	enum_ESCRETURN_exit,
	enum_ESCRETURN_continue,
	enum_ESCRETURN_restart
};

int Call_Escape(HWND hWnd);

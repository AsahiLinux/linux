// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include <windows.h>

// Visual Studio 6 is missing these, so define them here
#if defined(_MSC_VER) && _MSC_VER <= 1200
	#ifndef VK_OEM_PLUS
	#define VK_OEM_PLUS 0xBB
	#endif

	#ifndef VK_OEM_COMMA
	#define VK_OEM_COMMA 0xBC
	#endif

	#ifndef VK_OEM_PERIOD
	#define VK_OEM_PERIOD 0xBE
	#endif

	#ifndef VK_OEM_2
	#define VK_OEM_2 0xBF
	#endif

	#ifndef DWORD_PTR
	#define DWORD_PTR DWORD
	#endif

	// DLGPROC went from returning BOOL to INT_PTR in later versions, and VC6 doesn't like that
	#define DLGPROC_RET BOOL
#else
	#define DLGPROC_RET INT_PTR
#endif

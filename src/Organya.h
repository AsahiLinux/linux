// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#ifdef FIX_BUGS
// The original source code forgot to set this (you can tell because, in the original EXE,
// the DSBUFFERDESC structs in Organya.cpp have a different size to the ones in Sound.cpp)
#define DIRECTSOUND_VERSION 0x500
#endif
#include <dsound.h>

#include "WindowsWrapper.h"

#define MAXTRACK 16
#define MAXMELODY 8
#define MAXDRAM 8

extern BOOL g_mute[MAXTRACK];	// Used by the debug Mute menu

BOOL MakeOrganyaWave(signed char track, signed char wave_no, signed char pipi);
void OrganyaPlayData(void);
void SetPlayPointer(long x);
BOOL LoadOrganya(const char *name);
void SetOrganyaPosition(unsigned int x);
unsigned int GetOrganyaPosition(void);
void PlayOrganyaMusic(void);
BOOL ChangeOrganyaVolume(signed int volume);
void StopOrganyaMusic(void);
void SetOrganyaFadeout(void);
BOOL StartOrganya(LPDIRECTSOUND lpDS, const char *wave_filename);
void EndOrganya(void);

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

extern const char* const gVersionString;

DLGPROC_RET CALLBACK VersionDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);
DLGPROC_RET CALLBACK DebugMuteDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);
DLGPROC_RET CALLBACK DebugSaveDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);
DLGPROC_RET CALLBACK QuitDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);

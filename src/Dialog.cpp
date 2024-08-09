// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Dialog.h"

#include <stdio.h>

#include "WindowsWrapper.h"

#include "Generic.h"
#include "Organya.h"
#include "Profile.h"

// All of the original names for the functions/variables in this file are unknown

const char* const gVersionString = 
	"version.%d.%d.%d.%d\r\n"
	"2004/12/20 - %04d/%02d/%02d\r\n"
	"Studio Pixel"
	;

// TODO - Inaccurate stack frame
DLGPROC_RET CALLBACK VersionDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	char string_buffer[104];

	int year;
	int month;
	int day;

	int version1;
	int version2;
	int version3;
	int version4;

	(void)lParam;

	switch (Msg)
	{
		case WM_INITDIALOG:
			GetCompileDate(&year, &month, &day);
			GetCompileVersion(&version1, &version2, &version3, &version4);
			sprintf(string_buffer, gVersionString, version1, version2, version3, version4, year, month, day);
			SetDlgItemTextA(hWnd, 1011, string_buffer);

			CenteringWindowByParent(hWnd);

			return TRUE;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case 1:
					EndDialog(hWnd, 1);
					break;
			}

			break;
	}

	return FALSE;
}

DLGPROC_RET CALLBACK DebugMuteDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	(void)lParam;

	switch (Msg)
	{
		case WM_INITDIALOG:
			CenteringWindowByParent(hWnd);
			CheckDlgButton(hWnd, 1010, g_mute[0] != 0);
			CheckDlgButton(hWnd, 1018, g_mute[1] != 0);
			CheckDlgButton(hWnd, 1019, g_mute[2] != 0);
			CheckDlgButton(hWnd, 1020, g_mute[3] != 0);
			CheckDlgButton(hWnd, 1021, g_mute[4] != 0);
			CheckDlgButton(hWnd, 1022, g_mute[5] != 0);
			CheckDlgButton(hWnd, 1023, g_mute[6] != 0);
			CheckDlgButton(hWnd, 1024, g_mute[7] != 0);
			return TRUE;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case 2:
					EndDialog(hWnd, 0);
					break;

				case 1:
					g_mute[0] = IsDlgButtonChecked(hWnd, 1010);
					g_mute[1] = IsDlgButtonChecked(hWnd, 1018);
					g_mute[2] = IsDlgButtonChecked(hWnd, 1019);
					g_mute[3] = IsDlgButtonChecked(hWnd, 1020);
					g_mute[4] = IsDlgButtonChecked(hWnd, 1021);
					g_mute[5] = IsDlgButtonChecked(hWnd, 1022);
					g_mute[6] = IsDlgButtonChecked(hWnd, 1023);
					g_mute[7] = IsDlgButtonChecked(hWnd, 1024);
					EndDialog(hWnd, 1);
					break;
			}

			break;
	}

	return FALSE;
}

DLGPROC_RET CALLBACK DebugSaveDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	char string[100];

	(void)lParam;

	switch (Msg)
	{
		case WM_INITDIALOG:
			SetDlgItemTextA(hWnd, 1008, "000.dat");
			CenteringWindowByParent(hWnd);
			return TRUE;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case 2:
					EndDialog(hWnd, 0);
					break;

				case 1:
					GetDlgItemTextA(hWnd, 1008, string, sizeof(string));
					SaveProfile(string);
					EndDialog(hWnd, 1);
					break;
			}

			break;
	}

	return FALSE;
}

DLGPROC_RET CALLBACK QuitDialog(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	switch (Msg)
	{
		case WM_INITDIALOG:
			SetDlgItemTextA(hWnd, 1009, (LPCSTR)lParam);
			CenteringWindowByParent(hWnd);
			return TRUE;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case 2:
					EndDialog(hWnd, 2);
					break;

				case 1:
					EndDialog(hWnd, 1);
					break;
			}

			break;
	}

	return FALSE;
}

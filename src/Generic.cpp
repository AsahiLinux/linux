// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Generic.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "WindowsWrapper.h"

#include "Main.h"

void GetCompileDate(int *year, int *month, int *day)
{
	int i;
	char strMonth[0x10];

	const char *table[13] = {
		"XXX",
		"Jan",
		"Feb",
		"Mar",
		"Apr",
		"May",
		"Jun",
		"Jul",
		"Aug",
		"Sep",
		"Oct",
		"Nov",
		"Dec",
	};

	sscanf(__DATE__, "%s %d %d", strMonth, day, year);	// The expansion of __DATE__ is not reproductible. TODO : Think about changing this to be reproductible

	for (i = 0; i < 12; ++i)	// This being 12 instead of 13 might be a bug, but it works anyway by accident
		if (!memcmp(&strMonth, table[i], 3))
			break;

	*month = i;
}

// TODO - Inaccurate stack frame
BOOL GetCompileVersion(int *v1, int *v2, int *v3, int *v4)
{
	unsigned int puLen;
	VS_FIXEDFILEINFO *lpBuffer;
	DWORD dwHandle;
	DWORD dwLen;
	char path[MAX_PATH];
	LPVOID lpData;
	BOOL bResult;

	lpData = NULL;
	bResult = FALSE;

	GetModuleFileNameA(NULL, path, sizeof(path));
	dwLen = GetFileVersionInfoSizeA(path, &dwHandle);

	if (dwLen == 0)
		goto fail;

	lpData = malloc(dwLen);

	if (lpData == NULL)
		goto fail;

	if (!GetFileVersionInfoA(path, 0, dwLen, lpData))
		goto fail;

	if (!VerQueryValueA(lpData, "\\", (LPVOID*)&lpBuffer, &puLen))
		goto fail;

	*v1 = (unsigned short)(lpBuffer->dwFileVersionMS >> 16);
	*v2 = (unsigned short)(lpBuffer->dwFileVersionMS & 0xFFFF);
	*v3 = (unsigned short)(lpBuffer->dwFileVersionLS >> 16);
	*v4 = (unsigned short)(lpBuffer->dwFileVersionLS & 0xFFFF);
	bResult = TRUE;

fail:

	if (lpData != NULL)
		free(lpData);

	return bResult;
}

// This seems to be broken in recent Windows (Sndvol32.exe was renamed 'SndVol.exe')
// TODO - Inaccurate stack frame
BOOL OpenSoundVolume(HWND hWnd)
{
#ifdef FIX_BUGS
	char path[MAX_PATH];
	char path2[MAX_PATH];
	char path3[MAX_PATH];
	INT_PTR error;
	size_t i;

	GetSystemDirectoryA(path, sizeof(path));
	GetSystemDirectoryA(path2, sizeof(path2));

	i = strlen(path2);
	while (path2[i] != '\\')
		--i;

	path2[i] = '\0';

	sprintf(path3, "%s\\Sndvol32.exe", path);
	error = (INT_PTR)ShellExecuteA(hWnd, "open", path3, NULL, NULL, SW_SHOW);
	if (error > 32)
		return TRUE;

	sprintf(path3, "%s\\Sndvol32.exe", path2);
	error = (INT_PTR)ShellExecuteA(hWnd, "open", path3, NULL, NULL, SW_SHOW);
	if (error > 32)
		return TRUE;

	sprintf(path3, "%s\\Sndvol.exe", path);
	error = (INT_PTR)ShellExecuteA(hWnd, "open", path3, NULL, NULL, SW_SHOW);
	if (error > 32)
		return TRUE;

	sprintf(path3, "%s\\Sndvol.exe", path2);
	error = (INT_PTR)ShellExecuteA(hWnd, "open", path3, NULL, NULL, SW_SHOW);
	if (error > 32)
		return TRUE;

	return FALSE;
#else
	char path[MAX_PATH];
	char path2[MAX_PATH];
	char path3[MAX_PATH];
	INT_PTR error1;
	INT_PTR error2;
	size_t i;

	GetSystemDirectoryA(path, sizeof(path));
	sprintf(path2, "%s\\Sndvol32.exe", path);

	i = strlen(path);
	while (path[i] != '\\')
		--i;

	path[i] = '\0';
	sprintf(path3, "%s\\Sndvol32.exe", path);

	error1 = (INT_PTR)ShellExecuteA(hWnd, "open", path2, NULL, NULL, SW_SHOW);
	error2 = (INT_PTR)ShellExecuteA(hWnd, "open", path3, NULL, NULL, SW_SHOW);

	if (error1 <= 32 && error2 <= 32)
		return FALSE;
	else
		return TRUE;
#endif
}

void DeleteLog(void)
{
	char path[MAX_PATH];

	sprintf(path, "%s\\debug.txt", gModulePath);
	DeleteFileA(path);
}

BOOL WriteLog(const char *string, int value1, int value2, int value3)
{
	char path[MAX_PATH];
	FILE *fp;

	sprintf(path, "%s\\debug.txt", gModulePath);
	fp = fopen(path, "a+t");

	if (fp == NULL)
		return FALSE;

	fprintf(fp, "%s,%d,%d,%d\n", string, value1, value2, value3);
	fclose(fp);
	return TRUE;
}

/*
This function is a mystery. It seems to check if the system time is within
a certain range, specified by the two parameters. Nothing in the original game
uses this code.

This is just speculation, but this *might* have been used in those prototypes
Pixel released to testers, to prevent them from running after a certain date.
*/
int GetDateLimit(SYSTEMTIME *system_time_low, SYSTEMTIME *system_time_high)
{
	FILETIME FileTime1;
	FILETIME FileTime2;
	SYSTEMTIME SystemTime;

	GetSystemTime(&SystemTime);
	SystemTimeToFileTime(&SystemTime, &FileTime1);
	SystemTimeToFileTime(system_time_low, &FileTime2);

	if (CompareFileTime(&FileTime2, &FileTime1) >= 0)
		return -1;	// Return if actual time is lower than system_time_low

	SystemTimeToFileTime(system_time_high, &FileTime2);

	if (CompareFileTime(&FileTime2, &FileTime1) <= 0)
		return 1;	// Return if actual time is higher than system_time_high

	return 0;
}

BOOL IsKeyFile(const char *name)
{
	char path[MAX_PATH];
	FILE *fp;

	sprintf(path, "%s\\%s", gModulePath, name);

	fp = fopen(path, "rb");

	if (fp == NULL)
		return FALSE;

	fclose(fp);
	return TRUE;
}

long GetFileSizeLong(const char *path)
{
	DWORD len;
	HANDLE hFile;

	len = 0;

	hFile = CreateFileA(path, 0, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return INVALID_FILE_SIZE;

	len = GetFileSize(hFile, NULL);
	CloseHandle(hFile);
	return len;
}

BOOL ErrorLog(const char *string, int value)
{
	char path[MAX_PATH];
	FILE *fp;

	sprintf(path, "%s\\%s", gModulePath, "error.log");

	if (GetFileSizeLong(path) > 0x19000)	// Purge the error log if it gets too big, I guess
		DeleteFileA(path);

	fp = fopen(path, "a+t");
	if (fp == NULL)
		return FALSE;

	fprintf(fp, "%s,%d\n", string, value);
	fclose(fp);
	return TRUE;
}

BOOL IsShiftJIS(unsigned char c)
{
	if (c >= 0x81 && c <= 0x9F)
		return TRUE;

	if (c >= 0xE0 && c <= 0xEF)
		return TRUE;

	return FALSE;
}

// TODO - Inaccurate stack frame
BOOL CenteringWindowByParent(HWND hWnd)
{
	RECT window_rect;
	HWND parent_hwnd;
	RECT parent_rect;
	int x;
	int y;
	RECT child_rect;

	SystemParametersInfoA(SPI_GETWORKAREA, 0, &child_rect, 0);

	GetWindowRect(hWnd, &window_rect);

	parent_hwnd = GetParent(hWnd);
	if (parent_hwnd != NULL)
		GetWindowRect(parent_hwnd, &parent_rect);
	else
		SystemParametersInfoA(SPI_GETWORKAREA, 0, &parent_rect, 0);

	x = parent_rect.left + ((parent_rect.right - parent_rect.left) - (window_rect.right - window_rect.left)) / 2;
	y = parent_rect.top + ((parent_rect.bottom - parent_rect.top) - (window_rect.bottom - window_rect.top)) / 2;

	if (x < child_rect.left)
		x = child_rect.left;

	if (y < child_rect.top)
		y = child_rect.top;

	if (x + (window_rect.right - window_rect.left) > child_rect.right)
		x = child_rect.right - (window_rect.right - window_rect.left);

	if (y + (window_rect.bottom - window_rect.top) > child_rect.bottom)
		y = child_rect.bottom - (window_rect.bottom - window_rect.top);

	return SetWindowPos(hWnd, HWND_TOP, x, y, 0, 0, SWP_NOSIZE);
}

// TODO - Inaccurate stack frame
BOOL LoadWindowRect(HWND hWnd, const char *filename, BOOL unknown)
{
	char path[MAX_PATH];
	int min_window_width;
	int min_window_height;
	int max_window_width;
	int max_window_height;
	FILE *fp;
	RECT Rect;
	int showCmd;
	RECT pvParam;

	showCmd = SW_SHOWNORMAL;

	sprintf(path, "%s\\%s", gModulePath, filename);

	fp = fopen(path, "rb");
	if (fp != NULL)
	{
		fread(&Rect, sizeof(RECT), 1, fp);
		fread(&showCmd, sizeof(int), 1, fp);
		fclose(fp);

		SystemParametersInfoA(SPI_GETWORKAREA, 0, &pvParam, 0);

		max_window_width = GetSystemMetrics(SM_CXMAXIMIZED);
		max_window_height = GetSystemMetrics(SM_CYMAXIMIZED);
		min_window_width = GetSystemMetrics(SM_CXMIN);
		min_window_height = GetSystemMetrics(SM_CYMIN);

		if (Rect.right - Rect.left < min_window_width)
			Rect.right = Rect.left + min_window_width;
		if (Rect.bottom - Rect.top < min_window_height)
			Rect.bottom = Rect.top + min_window_height;
		if (Rect.right - Rect.left > max_window_width)
			Rect.right = Rect.left + max_window_width;
		if (Rect.bottom - Rect.top > max_window_height)
			Rect.bottom = Rect.top + max_window_width;

		if (Rect.left < pvParam.left)
		{
			Rect.right += pvParam.left - Rect.left;
			Rect.left = pvParam.left;
		}
		if (Rect.top < pvParam.top)
		{
			Rect.bottom += pvParam.top - Rect.top;
			Rect.top = pvParam.top;
		}
		if (Rect.right > pvParam.right)
		{
			Rect.left -= Rect.right - pvParam.right;
			Rect.right -= Rect.right - pvParam.right;
		}
		if (Rect.bottom > pvParam.bottom)
		{
			Rect.top -= Rect.bottom - pvParam.bottom;
			Rect.bottom -= Rect.bottom - pvParam.bottom;
		}

		if (unknown)
			MoveWindow(hWnd, Rect.left, Rect.top, Rect.right - Rect.left, Rect.bottom - Rect.top, 0);
		else
			SetWindowPos(hWnd, HWND_TOP, Rect.left, Rect.top, 0, 0, SWP_NOSIZE);
	}

	if (showCmd == SW_MAXIMIZE)
	{
		if (!ShowWindow(hWnd, SW_MAXIMIZE))
			return FALSE;
	}
	else
	{
		ShowWindow(hWnd, SW_SHOWNORMAL);
	}

	return TRUE;
}

BOOL SaveWindowRect(HWND hWnd, const char *filename)
{
	char path[MAX_PATH];
	WINDOWPLACEMENT wndpl;
	FILE *fp;
	RECT rect;

	if (!GetWindowPlacement(hWnd, &wndpl))
		return FALSE;

	if (wndpl.showCmd == SW_SHOWNORMAL)
	{
		if (!GetWindowRect(hWnd, &rect))
			return FALSE;

		wndpl.rcNormalPosition = rect;
	}

	sprintf(path, "%s\\%s", gModulePath, filename);

	fp = fopen(path, "wb");
	if (fp == NULL)
		return FALSE;

	fwrite(&wndpl.rcNormalPosition, sizeof(RECT), 1, fp);
	fwrite(&wndpl.showCmd, sizeof(int), 1, fp);
	fclose(fp);

	return TRUE;
}

static const char* const extra_text = "(C)Pixel";

BOOL IsEnableBitmap(const char *path)
{
	FILE *fp;
	long len;
	char str[16];

	len = (long)strlen(extra_text);

	fp = fopen(path, "rb");

	if (fp == NULL)
		return FALSE;

	fseek(fp, len * -1, SEEK_END);
	fread(str, 1, len, fp);
	fclose(fp);

	if (memcmp(str, extra_text, len))
		return FALSE;

	return TRUE;
}

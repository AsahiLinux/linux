// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Main.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <shlwapi.h>

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Config.h"
#include "Dialog.h"
#include "Draw.h"
#include "Game.h"
#include "Generic.h"
#include "Input.h"
#include "KeyControl.h"
#include "MyChar.h"
#include "Organya.h"
#include "Profile.h"
#include "Sound.h"
#include "Triangle.h"

LRESULT CALLBACK WindowProcedure(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam);

char gModulePath[MAX_PATH];
char gDataPath[MAX_PATH];

HWND ghWnd;
BOOL bFullscreen;
BOOL gbUseJoystick = FALSE;

int gJoystickButtonTable[8];

static BOOL bActive = TRUE;
static BOOL bFPS = FALSE;

static HANDLE hObject;
static HANDLE hMutex;
static HINSTANCE ghInstance;

static int windowWidth;
static int windowHeight;

static const char* const mutex_name = "Doukutsu";

#ifdef JAPANESE
static const char* const lpWindowName = "\x93\xB4\x8C\x41\x95\xA8\x8C\xEA";	// '洞窟物語' (Cave Story) in Shift-JIS
#else
static const char* const lpWindowName = "Cave Story ~ Doukutsu Monogatari";
#endif

// The original name for this function is unknown
void SetWindowName(HWND hWnd)
{
	char window_name[0x100];

	sprintf(window_name, "%s", lpWindowName);
	SetWindowTextA(hWnd, window_name);
}

// Framerate stuff
static unsigned long CountFramePerSecound(void)
{
	unsigned long current_tick;	// The original name for this variable is unknown
	static BOOL first = TRUE;
	static unsigned long max_count;
	static unsigned long count;
	static unsigned long wait;

	if (first)
	{
		wait = GetTickCount();
		first = FALSE;
	}

	current_tick = GetTickCount();
	++count;

	if (wait + 1000 <= current_tick)
	{
		wait += 1000;
		max_count = count;
		count = 0;
	}

	return max_count;
}

void PutFramePerSecound(void)
{
	if (bFPS)
	{
		const unsigned long fps = CountFramePerSecound();
		PutNumber4(WINDOW_WIDTH - 40, 8, fps, FALSE);
	}
}

// TODO - Inaccurate stack frame
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	(void)hPrevInstance;
	(void)lpCmdLine;
	(void)nShowCmd;

	int i;

	hObject = OpenMutexA(MUTEX_ALL_ACCESS, 0, mutex_name);
	if (hObject != NULL)
	{
		CloseHandle(hObject);
		return 0;
	}

	hMutex = CreateMutexA(NULL, FALSE, mutex_name);

	ghInstance = hInstance;

	// Get executable's path
	GetModuleFileNameA(NULL, gModulePath, MAX_PATH);
	PathRemoveFileSpecA(gModulePath);

	// Get path of the data folder
	strcpy(gDataPath, gModulePath);
	strcat(gDataPath, "\\data");

	CONFIGDATA conf;
	if (!LoadConfigData(&conf))
		DefaultConfigData(&conf);

	// Apply keybinds
	// Swap X and Z buttons
	switch (conf.attack_button_mode)
	{
		case 0:
			gKeyJump = KEY_Z;
			gKeyShot = KEY_X;
			break;

		case 1:
			gKeyJump = KEY_X;
			gKeyShot = KEY_Z;
			break;
	}

	// Swap Okay and Cancel buttons
	switch (conf.ok_button_mode)
	{
		case 0:
			gKeyOk = gKeyJump;
			gKeyCancel = gKeyShot;
			break;

		case 1:
			gKeyOk = gKeyShot;
			gKeyCancel = gKeyJump;
			break;
	}

	// Swap left and right weapon switch keys
	if (IsKeyFile("s_reverse"))
	{
		gKeyArms = KEY_ARMSREV;
		gKeyArmsRev = KEY_ARMS;
	}

	// Alternate movement keys
	switch (conf.move_button_mode)
	{
		case 0:
			gKeyLeft = KEY_LEFT;
			gKeyUp = KEY_UP;
			gKeyRight = KEY_RIGHT;
			gKeyDown = KEY_DOWN;
			break;

		case 1:
			gKeyLeft = KEY_ALT_LEFT;
			gKeyUp = KEY_ALT_UP;
			gKeyRight = KEY_ALT_RIGHT;
			gKeyDown = KEY_ALT_DOWN;
			break;
	}

	// Set gamepad inputs
	for (i = 0; i < 8; ++i)
	{
		switch (conf.joystick_button[i])
		{
			case 1:
				gJoystickButtonTable[i] = gKeyJump;
				break;

			case 2:
				gJoystickButtonTable[i] = gKeyShot;
				break;

			case 3:
				gJoystickButtonTable[i] = gKeyArms;
				break;

			case 6:
				gJoystickButtonTable[i] = gKeyArmsRev;
				break;

			case 4:
				gJoystickButtonTable[i] = gKeyItem;
				break;

			case 5:
				gJoystickButtonTable[i] = gKeyMap;
				break;
		}
	}

	RECT unused_rect = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};

	WNDCLASSEXA wndclassex;
	memset(&wndclassex, 0, sizeof(WNDCLASSEXA));
	wndclassex.cbSize = sizeof(WNDCLASSEXA);
	wndclassex.lpfnWndProc = WindowProcedure;
	wndclassex.hInstance = hInstance;
	wndclassex.hbrBackground = (HBRUSH)GetStockObject(DKGRAY_BRUSH);	// This is what gives the window's undrawn regions its grey colour
	wndclassex.lpszClassName = lpWindowName;
	wndclassex.hCursor = LoadCursorA(hInstance, "CURSOR_NORMAL");
	wndclassex.hIcon = LoadIconA(hInstance, "0");
	wndclassex.hIconSm = LoadIconA(hInstance, "ICON_MINI");

	HWND hWnd;
	HMENU hMenu;
	int nWidth;
	int nHeight;
	int x;
	int y;

	switch (conf.display_mode)
	{
		case 1:
		case 2:
			wndclassex.lpszMenuName = "MENU_MAIN";

			if (RegisterClassExA(&wndclassex) == 0)
			{
				ReleaseMutex(hMutex);
				return 0;
			}

			// Set window dimensions
			if (conf.display_mode == 1)
			{
				windowWidth = WINDOW_WIDTH;
				windowHeight = WINDOW_HEIGHT;
			}
			else
			{
				windowWidth = WINDOW_WIDTH * 2;
				windowHeight = WINDOW_HEIGHT * 2;
			}

			nWidth = (GetSystemMetrics(SM_CXFIXEDFRAME) * 2) + windowWidth + 2;
			nHeight = (GetSystemMetrics(SM_CYFIXEDFRAME) * 2) + GetSystemMetrics(SM_CYCAPTION) + GetSystemMetrics(SM_CYMENU) + windowHeight + 2;
			x = (GetSystemMetrics(SM_CXSCREEN) - nWidth) / 2;
			y = (GetSystemMetrics(SM_CYSCREEN) - nHeight) / 2;

			SetClientOffset(GetSystemMetrics(SM_CXFIXEDFRAME) + 1, GetSystemMetrics(SM_CYFIXEDFRAME) + GetSystemMetrics(SM_CYCAPTION) + GetSystemMetrics(SM_CYMENU) + 1);

			hWnd = CreateWindowExA(WS_EX_LEFT | WS_EX_LTRREADING | WS_EX_RIGHTSCROLLBAR, lpWindowName, lpWindowName, WS_MINIMIZEBOX | WS_SYSMENU | WS_BORDER | WS_DLGFRAME | WS_VISIBLE, x, y, nWidth, nHeight, NULL, NULL, hInstance, NULL);
			ghWnd = hWnd;

			if (hWnd == NULL)
			{
				ReleaseMutex(hMutex);
				return 0;
			}

			hMenu = GetMenu(hWnd);

		#ifdef FIX_MAJOR_BUGS
			if (conf.display_mode == 1)
			{
				if (!StartDirectDraw(hWnd, 0, 0))
				{
					ReleaseMutex(hMutex);
					return 0;
				}
			}
			else
			{
				if (!StartDirectDraw(hWnd, 1, 0))
				{
					ReleaseMutex(hMutex);
					return 0;
				}
			}
		#else
			// Doesn't handle StartDirectDraw failing
			if (conf.display_mode == 1)
				StartDirectDraw(hWnd, 0, 0);
			else
				StartDirectDraw(hWnd, 1, 0);
		#endif

			break;

		case 0:
		case 3:
		case 4:
			if (RegisterClassExA(&wndclassex) == 0)
			{
				ReleaseMutex(hMutex);
				return 0;
			}

			// Set window dimensions
			windowWidth = WINDOW_WIDTH * 2;
			windowHeight = WINDOW_HEIGHT * 2;

			SetClientOffset(0, 0);

			hWnd = CreateWindowExA(WS_EX_LEFT | WS_EX_LTRREADING | WS_EX_RIGHTSCROLLBAR, lpWindowName, lpWindowName, WS_SYSMENU | WS_VISIBLE | WS_POPUP, 0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN), NULL, NULL, hInstance, NULL);
			ghWnd = hWnd;

			if (hWnd == NULL)
			{
				ReleaseMutex(hMutex);
				return 0;
			}

			// Set colour depth
			int depth;

			switch (conf.display_mode)
			{
				case 0:
					depth = 16;
					break;
				case 3:
					depth = 24;
					break;
				case 4:
					depth = 32;
					break;
			}

		#ifdef FIX_MAJOR_BUGS
			if (!StartDirectDraw(hWnd, 2, depth))
			{
				ReleaseMutex(hMutex);
				return 0;
			}
		#else
			// Doesn't handle StartDirectDraw failing
			StartDirectDraw(hWnd, 2, depth);
		#endif

			bFullscreen = TRUE;

			ShowCursor(FALSE);
			break;
	}

	// Set rects
	RECT rcLoading = {0, 0, 64, 8};
	RECT rcFull = {0, 0, 0, 0};
	rcFull.right = WINDOW_WIDTH;
	rcFull.bottom = WINDOW_HEIGHT;

	// Load the "LOADING" text
	BOOL b = MakeSurface_File("Loading", SURFACE_ID_LOADING);

	// Draw loading screen
	CortBox(&rcFull, 0x000000);
	PutBitmap3(&rcFull, (WINDOW_WIDTH / 2) - 32, (WINDOW_HEIGHT / 2) - 4, &rcLoading, SURFACE_ID_LOADING);

	// Draw to screen
	if (!Flip_SystemTask(ghWnd))
	{
		ReleaseMutex(hMutex);
		return 1;
	}

	// Initialize sound
	InitDirectSound(hWnd);

	// Initialize joystick
	if (conf.bJoystick && InitDirectInput(hInstance, hWnd))
	{
		ResetJoystickStatus();
		gbUseJoystick = TRUE;
	}

	// Initialize stuff
	InitTextObject(conf.font_name);
	InitTriangleTable();

	// Run game code
	Game(hWnd);

	// End stuff
	EndTextObject();
	EndDirectSound();
	EndDirectDraw(hWnd);

	ReleaseMutex(hMutex);

	return 1;
}

void InactiveWindow(void)
{
	if (bActive)
	{
		bActive = FALSE;
		StopOrganyaMusic();
		SleepNoise();
	}

	PlaySoundObject(7, SOUND_MODE_STOP);
}

void ActiveWindow(void)
{
	if (!bActive)
	{
		bActive = TRUE;
		StopOrganyaMusic();
		PlayOrganyaMusic();
		ResetNoise();
	}

	PlaySoundObject(7, SOUND_MODE_PLAY_LOOP);
}

// Turns out you could drag-and-drop a save file onto the
// window to load it, but this behavior is dummied-out.
BOOL DragAndDropHandler(HWND hWnd, WPARAM wParam)
{
	char path[MAX_PATH];
	HDROP hDrop = (HDROP)wParam;

	if (DragQueryFileA(hDrop, 0xFFFFFFFF, NULL, 0) != 0)
	{
		DragQueryFileA(hDrop, 0, path, sizeof(path));
		LoadProfile(path);
	}

	DragFinish(hDrop);

	return TRUE;
}

// TODO - Inaccurate stack frame
LRESULT CALLBACK WindowProcedure(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
{
	BOOL window_focus;
	HMENU hMenu;

	switch (Msg)
	{
		case WM_CREATE:
			hMenu = GetMenu(hWnd);
		#ifdef DEBUG_SAVE
			if (!IsKeyFile("save"))	// Chances are a line like this used to exist
		#endif
				DeleteMenu(hMenu, 40005, MF_BYCOMMAND);
			DrawMenuBar(hWnd);

			hMenu = GetMenu(hWnd);
			if (!IsKeyFile("mute"))
				DeleteMenu(hMenu, 40007, MF_BYCOMMAND);
			DrawMenuBar(hWnd);

			if (IsKeyFile("fps"))
				bFPS = TRUE;

			if (!bFullscreen)
				LoadWindowRect(hWnd, "window.rect", FALSE);

			SetWindowName(hWnd);

		#ifdef DEBUG_SAVE
			DragAcceptFiles(hWnd, TRUE);
		#endif

			break;

		case WM_SYSCOMMAND:
			switch (wParam)
			{
				case SC_MONITORPOWER:
					break;

				case SC_KEYMENU:
					break;

				case SC_SCREENSAVE:
					break;

				default:
					DefWindowProcA(hWnd, Msg, wParam, lParam);
					break;
			}

			break;

		case WM_IME_NOTIFY:
			if (wParam == IMN_SETOPENSTATUS)
			{
				HIMC hImc = ImmGetContext(hWnd);
				ImmSetOpenStatus(hImc, 0);
				ImmReleaseContext(hWnd, hImc);
			}

			break;

		case WM_KEYDOWN:
			switch (wParam)
			{
				case VK_ESCAPE:
					gKey |= KEY_ESCAPE;
					break;

				case 'W':
					gKey |= KEY_MAP;
					break;

				case VK_LEFT:
					gKey |= KEY_LEFT;
					break;

				case VK_RIGHT:
					gKey |= KEY_RIGHT;
					break;

				case VK_UP:
					gKey |= KEY_UP;
					break;

				case VK_DOWN:
					gKey |= KEY_DOWN;
					break;

				case 'X':
					gKey |= KEY_X;
					break;

				case 'Z':
					gKey |= KEY_Z;
					break;

				case 'S':
					gKey |= KEY_ARMS;
					break;

				case 'A':
					gKey |= KEY_ARMSREV;
					break;

				case VK_SHIFT:
					gKey |= KEY_SHIFT;
					break;

				case VK_F1:
					gKey |= KEY_F1;
					break;

				case VK_F2:
					gKey |= KEY_F2;
					break;

				case 'Q':
					gKey |= KEY_ITEM;
					break;

				case VK_OEM_COMMA:
					gKey |= KEY_ALT_LEFT;
					break;

				case VK_OEM_PERIOD:
					gKey |= KEY_ALT_DOWN;
					break;

				case VK_OEM_2:
					gKey |= KEY_ALT_RIGHT;
					break;

				case 'L':
					gKey |= KEY_L;
					break;

				case VK_OEM_PLUS:
					gKey |= KEY_PLUS;
					break;

				case VK_F5:
					gbUseJoystick = FALSE;
					break;
			}

			break;

		case WM_KEYUP:
			switch (wParam)
			{
				case VK_ESCAPE:
					gKey &= ~KEY_ESCAPE;
					break;

				case 'W':
					gKey &= ~KEY_MAP;
					break;

				case VK_LEFT:
					gKey &= ~KEY_LEFT;
					break;

				case VK_RIGHT:
					gKey &= ~KEY_RIGHT;
					break;

				case VK_UP:
					gKey &= ~KEY_UP;
					break;

				case VK_DOWN:
					gKey &= ~KEY_DOWN;
					break;

				case 'X':
					gKey &= ~KEY_X;
					break;

				case 'Z':
					gKey &= ~KEY_Z;
					break;

				case 'S':
					gKey &= ~KEY_ARMS;
					break;

				case 'A':
					gKey &= ~KEY_ARMSREV;
					break;

				case VK_SHIFT:
					gKey &= ~KEY_SHIFT;
					break;

				case VK_F1:
					gKey &= ~KEY_F1;
					break;

				case VK_F2:
					gKey &= ~KEY_F2;
					break;

				case 'Q':
					gKey &= ~KEY_ITEM;
					break;

				case VK_OEM_COMMA:
					gKey &= ~KEY_ALT_LEFT;
					break;

				case VK_OEM_PERIOD:
					gKey &= ~KEY_ALT_DOWN;
					break;

				case VK_OEM_2:
					gKey &= ~KEY_ALT_RIGHT;
					break;

				case 'L':
					gKey &= ~KEY_L;
					break;

				case VK_OEM_PLUS:
					gKey &= ~KEY_PLUS;
					break;
			}

			break;

		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case 40001:
					if (DialogBoxParamA(ghInstance, "DLG_YESNO", hWnd, QuitDialog, (LPARAM)"Quit?") == 1)
						PostMessageA(hWnd, WM_CLOSE, 0, 0);
					break;

				case 40002:
					DialogBoxParamA(ghInstance, "DLG_ABOUT", hWnd, VersionDialog, 0);
					break;

				case 40004:
					if (!OpenSoundVolume(hWnd))
						MessageBoxA(hWnd, "\x83\x7B\x83\x8A\x83\x85\x81\x5B\x83\x80\x90\xDD\x92\xE8\x82\xF0\x8B\x4E\x93\xAE\x82\xC5\x82\xAB\x82\xDC\x82\xB9\x82\xF1\x82\xC5\x82\xB5\x82\xBD", lpWindowName, 0);	// 'ボリューム設定を起動できませんでした' (Could not launch volume configuration) in Shift-JIS
					break;

				case 40005:
					DialogBoxParamA(ghInstance, "DLG_SAVE", hWnd, DebugSaveDialog, 0);
					break;

				case 40007:
					DialogBoxParamA(ghInstance, "DLG_MUTE", hWnd, DebugMuteDialog, 0);
					break;
			}

			break;

		case WM_DROPFILES:
			DragAndDropHandler(hWnd, wParam);
			break;

		case WM_ACTIVATE:
			switch (LOWORD(wParam))
			{
				case WA_INACTIVE:
					window_focus = FALSE;
					break;

				case WA_ACTIVE:
				case WA_CLICKACTIVE:
					if (HIWORD(wParam) != 0)
						window_focus = FALSE;
					else
						window_focus = TRUE;

					break;
			}

			if (window_focus)
				ActiveWindow();
			else
				InactiveWindow();

			break;

		case WM_CLOSE:
			StopOrganyaMusic();
			PostQuitMessage(0);
			break;

		default:
			return DefWindowProcA(hWnd, Msg, wParam, lParam);
	}

	return 1;
}

void JoystickProc(void);

BOOL SystemTask(void)
{
	MSG Msg;

	while (PeekMessageA(&Msg, NULL, 0, 0, PM_NOREMOVE) || !bActive)
	{
		if (!GetMessageA(&Msg, NULL, 0, 0))
			return FALSE;

		TranslateMessage(&Msg);
		DispatchMessageA(&Msg);
	}

	// Run joystick code
	if (gbUseJoystick)
		JoystickProc();

	return TRUE;
}

void JoystickProc(void)
{
	int i;
	DIRECTINPUTSTATUS status;

	if (!GetJoystickStatus(&status))
		return;

	gKey &= (KEY_ESCAPE | KEY_F1 | KEY_F2);

	// Set movement buttons
	if (status.bLeft)
		gKey |= gKeyLeft;
	else
		gKey &= ~gKeyLeft;

	if (status.bRight)
		gKey |= gKeyRight;
	else
		gKey &= ~gKeyRight;

	if (status.bUp)
		gKey |= gKeyUp;
	else
		gKey &= ~gKeyUp;

	if (status.bDown)
		gKey |= gKeyDown;
	else
		gKey &= ~gKeyDown;

	// Clear held buttons
	for (i = 0; i < 8; ++i)
		gKey &= ~gJoystickButtonTable[i];

	// Set held buttons
	for (i = 0; i < 8; ++i)
		if (status.bButton[i])
			gKey |= gJoystickButtonTable[i];
}

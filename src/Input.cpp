// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Input.h"

#include <stddef.h>
#include <stdio.h>

#if defined(_MSC_VER) && _MSC_VER >= 1500	// Newer versions of Visual Studio don't support anything earlier than DirectInput8
#define DIRECTINPUT_VERSION 0x800
#else
#define DIRECTINPUT_VERSION 0x500
#endif
#include <dinput.h>

#include "WindowsWrapper.h"

typedef struct DirectInputPair
{
	LPDIRECTINPUTA lpDI;
	LPDIRECTINPUTDEVICE2A device;
} DirectInputPair;

// The original names for these variables are unknown
static LPDIRECTINPUTA lpDI = NULL;
static LPDIRECTINPUTDEVICE2A joystick = NULL;
static int joystick_neutral_x = 0;
static int joystick_neutral_y = 0;

void ReleaseDirectInput(void)
{
	if (joystick != NULL)
	{
		joystick->Release();
		joystick = NULL;
	}

	if (lpDI != NULL)
	{
		lpDI->Release();
		lpDI = NULL;
	}
}

// The original name for this function's variables are unknown
BOOL ActivateDirectInput(BOOL aquire)
{
	if (aquire == TRUE)
	{
		if (joystick != NULL)
			joystick->Acquire();
	}
	else
	{
		if (joystick != NULL)
			joystick->Unacquire();
	}

	return TRUE;
}

// It looks like Pixel declared his functions early, so he could forward-reference
BOOL FindAndOpenDirectInputDevice(HWND hWnd);
BOOL CALLBACK EnumDevices_Callback(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef);

BOOL InitDirectInput(HINSTANCE hinst, HWND hWnd)
{
#if defined(_MSC_VER) && _MSC_VER >= 1500
	if (DirectInput8Create(hinst, DIRECTINPUT_VERSION, IID_IDirectInput8A, (LPVOID*)&lpDI, NULL) != DI_OK)
#else
	if (DirectInputCreateA(hinst, DIRECTINPUT_VERSION, &lpDI, NULL) != DI_OK)
#endif
		return FALSE;

	if (!FindAndOpenDirectInputDevice(hWnd))
		return FALSE;

	return TRUE;
}

// The original name for this function and its variables are unknown.
// This function finds and hooks the first available DirectInput device.
BOOL FindAndOpenDirectInputDevice(HWND hWnd)
{
	DirectInputPair directinput_objects;

	directinput_objects.device = NULL;
	directinput_objects.lpDI = lpDI;

	directinput_objects.lpDI->AddRef();

#if defined(_MSC_VER) && _MSC_VER >= 1500
	lpDI->EnumDevices(DI8DEVTYPE_JOYSTICK, EnumDevices_Callback, &directinput_objects, DIEDFL_ATTACHEDONLY);
#else
	lpDI->EnumDevices(DIDEVTYPE_JOYSTICK, EnumDevices_Callback, &directinput_objects, DIEDFL_ATTACHEDONLY);
#endif

	if (directinput_objects.lpDI != NULL)
	{
		directinput_objects.lpDI->Release();
		directinput_objects.lpDI = NULL;
	}

	if (directinput_objects.device == NULL)
		return FALSE;

	joystick = directinput_objects.device;

	if (joystick->SetDataFormat(&c_dfDIJoystick) != DI_OK)	// c_dfDIJoystick might be incorrect
		return FALSE;

	if (joystick->SetCooperativeLevel(hWnd, DISCL_EXCLUSIVE | DISCL_BACKGROUND) != DI_OK)
		return FALSE;

	joystick->Acquire();

	return TRUE;
}

// The original names for this function and its variables are unknown
BOOL CALLBACK EnumDevices_Callback(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef)
{
	static int already_ran;
	static DirectInputPair *directinput_objects;

	if (!(already_ran & 1))
	{
		already_ran |= 1;
		directinput_objects = (DirectInputPair*)pvRef;
	}

	static LPDIRECTINPUTDEVICEA device;
	if (directinput_objects->lpDI->CreateDevice(lpddi->guidInstance, &device, NULL) != DI_OK)
	{
		directinput_objects->device = NULL;
		return DIENUM_CONTINUE;
	}

	static LPDIRECTINPUTDEVICE2A _joystick;
	HRESULT res = device->QueryInterface(IID_IDirectInputDevice2A, (LPVOID*)&_joystick);

	if (FAILED(res))
	{
		joystick = NULL;
		return DIENUM_CONTINUE;
	}

	if (device != NULL)
	{
		device->Release();
		device = NULL;
	}

	directinput_objects->device = _joystick;

	// This is interesting: there are at least two places in the game
	// where it seems like there's meant to be a debug print just like
	// this one: these are the 'out' function in 'Draw.cpp', and the
	// 'LoadGenericData' function in 'GenericLoad.cpp'.
	// Perhaps Pixel kept them wrapped in '#ifdef DEBUG' blocks, and
	// simply forgot to do the same here.

	char str[0x100];
#ifdef FIX_MAJOR_BUGS
	sprintf(str, "DeviceGUID = %08lX-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X\n", lpddi->guidInstance.Data1, lpddi->guidInstance.Data2, lpddi->guidInstance.Data3, lpddi->guidInstance.Data4[0], lpddi->guidInstance.Data4[1], lpddi->guidInstance.Data4[2], lpddi->guidInstance.Data4[3], lpddi->guidInstance.Data4[4], lpddi->guidInstance.Data4[5], lpddi->guidInstance.Data4[6], lpddi->guidInstance.Data4[7]);
#else
	sprintf(str, "DeviceGUID = %x\n", lpddi->guidInstance);	// Tries to print a struct as an int
#endif
	OutputDebugStringA(str);

	return DIENUM_STOP;
}

BOOL GetJoystickStatus(DIRECTINPUTSTATUS *status)
{
	DIJOYSTATE joystate;

	if (joystick == NULL)
		return FALSE;

	if (joystick->Poll() != DI_OK)
		return FALSE;

	HRESULT res = joystick->GetDeviceState(sizeof(DIJOYSTATE), &joystate);
	if (res != DI_OK)
	{
		if (res == DIERR_INPUTLOST)
			ActivateDirectInput(FALSE);
		else
			return FALSE;
	}

	for (int i = 0; i < 32; ++i)
	{
		if (joystate.rgbButtons[i] & 0x80)
			status->bButton[i] = TRUE;
		else
			status->bButton[i] = FALSE;
	}

	status->bDown = FALSE;
	status->bRight = FALSE;
	status->bUp = FALSE;
	status->bLeft = FALSE;

	if (joystate.lX < joystick_neutral_x - 10000)
		status->bLeft = TRUE;
	else if (joystate.lX > joystick_neutral_x + 10000)
		status->bRight = TRUE;

	if (joystate.lY < joystick_neutral_y - 10000)
		status->bUp = TRUE;
	else if (joystate.lY > joystick_neutral_y + 10000)
		status->bDown = TRUE;

	return TRUE;
}

BOOL ResetJoystickStatus(void)
{
	DIJOYSTATE joystate;

	if (joystick == NULL)
		return FALSE;

	if (joystick->Poll() != DI_OK)
		return FALSE;

	HRESULT res = joystick->GetDeviceState(sizeof(DIJOYSTATE), &joystate);
	if (res != DI_OK)
	{
		if (res == DIERR_INPUTLOST)
			ActivateDirectInput(FALSE);
		else
			return FALSE;
	}

	joystick_neutral_x = joystate.lX;
	joystick_neutral_y = joystate.lY;

	return TRUE;
}

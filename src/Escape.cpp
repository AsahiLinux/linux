// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Escape.h"

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Draw.h"
#include "KeyControl.h"
#include "Main.h"

int Call_Escape(HWND hWnd)
{
	RECT rc = {0, 128, 208, 144};

	while (1)
	{
		// Get pressed keys
		GetTrg();

		if (gKeyTrg & KEY_ESCAPE) // Escape is pressed, quit game
		{
			gKeyTrg = 0;
			return enum_ESCRETURN_exit;
		}
		if (gKeyTrg & KEY_F1) // F1 is pressed, continue
		{
			gKeyTrg = 0;
			return enum_ESCRETURN_continue;
		}
		if (gKeyTrg & KEY_F2) // F2 is pressed, reset
		{
			gKeyTrg = 0;
			return enum_ESCRETURN_restart;
		}

		// Draw screen
		CortBox(&grcFull, 0x000000);
		PutBitmap3(&grcFull, (WINDOW_WIDTH / 2) - 104, (WINDOW_HEIGHT / 2) - 8, &rc, SURFACE_ID_TEXT_BOX);
		PutFramePerSecound();

		if (!Flip_SystemTask(hWnd))
		{
			// Quit if window is closed
			gKeyTrg = 0;
			return enum_ESCRETURN_exit;
		}
	}

	return enum_ESCRETURN_exit;
}

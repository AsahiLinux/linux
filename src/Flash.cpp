// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Flash.h"

#include "CommonDefines.h"
#include "Draw.h"
#include "WindowsWrapper.h"

static struct
{
	FlashMode mode;
	int act_no;
	BOOL flag;
	int cnt;
	int width;
	int x;
	int y;
	RECT rect1;
	RECT rect2;
} flash;

unsigned long gFlashColor;

void InitFlash(void)
{
	gFlashColor = GetCortBoxColor(RGB(0xFF, 0xFF, 0xFE));
}

void SetFlash(int x, int y, FlashMode mode)
{
	flash.act_no = 0;
	flash.flag = TRUE;
	flash.x = x;
	flash.y = y;
	flash.mode = mode;
	flash.cnt = 0;
	flash.width = 0;
}

void ActFlash_Explosion(int flx, int fly)
{
	int left, top, right, bottom;

	switch (flash.act_no)
	{
		case 0: // Expand
			flash.cnt += 0x200;
			flash.width += flash.cnt;

			left = (flash.x - flx - flash.width) / 0x200;
			top = (flash.y - fly - flash.width) / 0x200;
			right = (flash.x - flx + flash.width) / 0x200;
			bottom = (flash.y - fly + flash.width) / 0x200;

			if (left < 0)
				left = 0;
			if (top < 0)
				top = 0;
			if (right > WINDOW_WIDTH)
				right = WINDOW_WIDTH;
			if (bottom > WINDOW_HEIGHT)
				bottom = WINDOW_HEIGHT;

			// The tall part of the explosion
			flash.rect1.left = left;
			flash.rect1.right = right;
			flash.rect1.top = 0;
			flash.rect1.bottom = WINDOW_HEIGHT;

			// The wide part of the explosion
			flash.rect2.left = 0;
			flash.rect2.right = WINDOW_WIDTH;
			flash.rect2.top = top;
			flash.rect2.bottom = bottom;

			if (flash.width > WINDOW_WIDTH * 0x200 * 4) // I guess in theory this means that the explosion would take longer in widescreen...
			{
				flash.act_no = 1;
				flash.cnt = 0;
				flash.width = WINDOW_HEIGHT * 0x200;
			}

			break;

		case 1: // Shrink
			flash.width -= flash.width / 8;

			if ((flash.width / 0x100) == 0)
				flash.flag = FALSE;

			top = (flash.y - fly - flash.width) / 0x200;
			if (top < 0)
				top = 0;

			bottom = (flash.y - fly + flash.width) / 0x200;
			if (bottom > WINDOW_HEIGHT)
				bottom = WINDOW_HEIGHT;

			// The tall part of the explosion
			flash.rect1.left = 0;
			flash.rect1.right = 0;
			flash.rect1.top = 0;
			flash.rect1.bottom = 0;

			// The wide part of the explosion
			flash.rect2.top = top;
			flash.rect2.bottom = bottom;
			flash.rect2.left = 0;
			flash.rect2.right = WINDOW_WIDTH;

			break;
	}
}

void ActFlash_Flash(void)
{
	++flash.cnt;

	flash.rect1.left = 0;
	flash.rect1.right = 0;
	flash.rect1.top = 0;
	flash.rect1.bottom = 0;

	if (flash.cnt / 2 % 2)
	{
		flash.rect2.top = 0;
		flash.rect2.bottom = WINDOW_HEIGHT;
		flash.rect2.left = 0;
		flash.rect2.right = WINDOW_WIDTH;
	}
	else
	{
		flash.rect2.left = 0;
		flash.rect2.right = 0;
		flash.rect2.top = 0;
		flash.rect2.bottom = 0;
	}

	if (flash.cnt > 20)
		flash.flag = FALSE;
}

void ActFlash(int flx, int fly)
{
	if (!flash.flag)
		return;

	switch (flash.mode)
	{
		case FLASH_MODE_EXPLOSION:
			ActFlash_Explosion(flx, fly);
			break;

		case FLASH_MODE_FLASH:
			ActFlash_Flash();
			break;
	}
}

void PutFlash(void)
{
	if (!flash.flag)
		return;

	CortBox(&flash.rect1, gFlashColor);
	CortBox(&flash.rect2, gFlashColor);
}

void ResetFlash(void)
{
	flash.flag = FALSE;
}

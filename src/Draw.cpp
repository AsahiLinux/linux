// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Draw.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <ddraw.h>

#include "WindowsWrapper.h"

#include "CommonDefines.h"
#include "Ending.h"
#include "Generic.h"
#include "Main.h"
#include "MapName.h"
#include "TextScr.h"

typedef enum SurfaceType
{
	SURFACE_SOURCE_NONE = 1,
	SURFACE_SOURCE_RESOURCE,
	SURFACE_SOURCE_FILE
} SurfaceType;

RECT grcGame = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};
RECT grcFull = {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT};

static int mag;
static BOOL fullscreen;	// TODO - Not the original variable name

static LPDIRECTDRAW lpDD;	// TODO - Not the original variable name
static LPDIRECTDRAWSURFACE frontbuffer;	// TODO - Not the original variable name
static LPDIRECTDRAWSURFACE backbuffer;	// TODO - Not the original variable name

static LPDIRECTDRAWCLIPPER clipper;	// TODO - Not the original variable name

static LPDIRECTDRAWSURFACE surf[SURFACE_ID_MAX];

static RECT backbuffer_rect;	// TODO - Not the original variable name

static int scaled_window_width;	// TODO - Not the original variable name
static int scaled_window_height;	// TODO - Not the original variable name

static HFONT font;	// TODO - Not the original variable name

// This doesn't exist in the Linux port, so none of these symbol names are accurate
static struct
{
	char name[20];
	unsigned int width;
	unsigned int height;
	SurfaceType type;
	BOOL bSystem;	// Basically a 'do not regenerate' flag
} surface_metadata[SURFACE_ID_MAX];

static int client_x;
static int client_y;

void SetClientOffset(int width, int height)
{
	client_x = width;
	client_y = height;
}

BOOL Flip_SystemTask(HWND hWnd)
{
	// TODO - Not the original variable names
	static DWORD timePrev;
	static DWORD timeNow;

	while (TRUE)
	{
		if (!SystemTask())
			return FALSE;

		// Framerate limiter
		timeNow = GetTickCount();

		if (timeNow >= timePrev + 20)
			break;

		Sleep(1);
	}

	if (timeNow >= timePrev + 100)
		timePrev = timeNow;	// If the timer is freakishly out of sync, panic and reset it, instead of spamming frames for who-knows how long
	else
		timePrev += 20;

	static RECT dst_rect;	// TODO - Not the original variable name
	GetWindowRect(hWnd, &dst_rect);
	dst_rect.left += client_x;
	dst_rect.top += client_y;
	dst_rect.right = dst_rect.left + scaled_window_width;
	dst_rect.bottom = dst_rect.top + scaled_window_height;

	frontbuffer->Blt(&dst_rect, backbuffer, &backbuffer_rect, DDBLT_WAIT, NULL);

	if (RestoreSurfaces())
	{
		RestoreStripper();
		RestoreMapName();
		RestoreTextScript();
	}

	return TRUE;
}

BOOL StartDirectDraw(HWND hWnd, int lMagnification, int lColourDepth)
{
	DDSURFACEDESC ddsd;

	if (DirectDrawCreate(NULL, &lpDD, NULL) != DD_OK)
		return FALSE;

	memset(surface_metadata, 0, sizeof(surface_metadata));

	switch (lMagnification)
	{
		case 0:
			mag = 1;
			fullscreen = FALSE;
			lpDD->SetCooperativeLevel(hWnd, DDSCL_NORMAL);
			break;

		case 1:
			mag = 2;
			fullscreen = FALSE;
			lpDD->SetCooperativeLevel(hWnd, DDSCL_NORMAL);
			break;

		case 2:
			mag = 2;
			fullscreen = TRUE;
			lpDD->SetCooperativeLevel(hWnd, DDSCL_FULLSCREEN | DDSCL_EXCLUSIVE);
			lpDD->SetDisplayMode(WINDOW_WIDTH * mag, WINDOW_HEIGHT * mag, lColourDepth);
			break;
	}

	backbuffer_rect.left = 0;
	backbuffer_rect.top = 0;
	backbuffer_rect.right = WINDOW_WIDTH * mag;
	backbuffer_rect.bottom = WINDOW_HEIGHT * mag;

	scaled_window_width = WINDOW_WIDTH * mag;
	scaled_window_height = WINDOW_HEIGHT * mag;

	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);
	ddsd.dwFlags = DDSD_CAPS;
	ddsd.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE;
	ddsd.dwBackBufferCount = 0;

	if (lpDD->CreateSurface(&ddsd, &frontbuffer, NULL) != DD_OK)
		return FALSE;

	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);
	ddsd.dwFlags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH;
	ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN;
	ddsd.dwWidth = WINDOW_WIDTH * mag;
	ddsd.dwHeight = WINDOW_HEIGHT * mag;

	if (lpDD->CreateSurface(&ddsd, &backbuffer, NULL) != DD_OK)
		return FALSE;

	lpDD->CreateClipper(0, &clipper, NULL);
	clipper->SetHWnd(0, hWnd);
	frontbuffer->SetClipper(clipper);

	return TRUE;
}

void EndDirectDraw(HWND hWnd)
{
	int i;

	// Release all surfaces
	for (i = 0; i < SURFACE_ID_MAX; ++i)
	{
		if (surf[i] != NULL)
		{
			surf[i]->Release();
			surf[i] = NULL;
		}
	}

	if (frontbuffer != NULL)
	{
		frontbuffer->Release();
		frontbuffer = NULL;
		backbuffer = NULL;
	}

	if (fullscreen)
		lpDD->SetCooperativeLevel(hWnd, DDSCL_NORMAL);

	if (lpDD != NULL)
	{
		lpDD->Release();
		lpDD = NULL;
	}

	memset(surface_metadata, 0, sizeof(surface_metadata));
}

void ReleaseSurface(SurfaceID s)
{
	// Release the surface we want to release
	if (surf[s] != NULL)
	{
		surf[s]->Release();
		surf[s] = NULL;
	}

	memset(&surface_metadata[s], 0, sizeof(surface_metadata[0]));
}

// TODO - Inaccurate stack frame
BOOL MakeSurface_Resource(const char *name, SurfaceID surf_no)
{
	if (surf_no >= SURFACE_ID_MAX)
		return FALSE;

	if (surf[surf_no] != NULL)
		return FALSE;

	HANDLE handle = LoadImageA(GetModuleHandleA(NULL), name, IMAGE_BITMAP, 0, 0, LR_CREATEDIBSECTION);
	if (handle == NULL)
		return FALSE;

	BITMAP bitmap;
	GetObjectA(handle, sizeof(BITMAP), &bitmap);

	DDSURFACEDESC ddsd;
	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);
	ddsd.dwFlags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH;
	ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN;
	ddsd.dwWidth = bitmap.bmWidth * mag;
	ddsd.dwHeight = bitmap.bmHeight * mag;

	if (lpDD->CreateSurface(&ddsd, &surf[surf_no], NULL) != DD_OK)
		return FALSE;

	int src_x = 0;
	int src_y = 0;
	int src_w = bitmap.bmWidth;
	int src_h = bitmap.bmHeight;

	int dst_x = 0;
	int dst_y = 0;
	int dst_w = bitmap.bmWidth * mag;
	int dst_h = bitmap.bmHeight * mag;

	HDC hdc = CreateCompatibleDC(NULL);
	HGDIOBJ hgdiobj = SelectObject(hdc, handle);

	HDC hdc2;
	surf[surf_no]->GetDC(&hdc2);
	StretchBlt(hdc2, dst_x, dst_y, dst_w, dst_h, hdc, src_x, src_y, src_w, src_h, SRCCOPY);
	surf[surf_no]->ReleaseDC(hdc2);

	SelectObject(hdc, hgdiobj);
	DeleteDC(hdc);

	DDCOLORKEY ddcolorkey;
	ddcolorkey.dwColorSpaceLowValue = 0;
	ddcolorkey.dwColorSpaceHighValue = 0;

	surf[surf_no]->SetColorKey(DDCKEY_SRCBLT, &ddcolorkey);
	surf[surf_no]->SetClipper(clipper);

#ifdef FIX_MAJOR_BUGS
	DeleteObject(handle);
#endif

	surface_metadata[surf_no].type = SURFACE_SOURCE_RESOURCE;
	surface_metadata[surf_no].width = bitmap.bmWidth;
	surface_metadata[surf_no].height = bitmap.bmHeight;
	surface_metadata[surf_no].bSystem = FALSE;
	strcpy(surface_metadata[surf_no].name, name);

	return TRUE;
}

// TODO - Inaccurate stack frame
BOOL MakeSurface_File(const char *name, SurfaceID surf_no)
{
	char path[MAX_PATH];
	sprintf(path, "%s\\%s.pbm", gDataPath, name);

	if (!IsEnableBitmap(path))
	{
		ErrorLog(path, 0);
		return FALSE;
	}

#ifdef FIX_BUGS
	if (surf_no >= SURFACE_ID_MAX)
#else
	if (surf_no > SURFACE_ID_MAX)
#endif
	{
		ErrorLog("surface no", surf_no);
		return FALSE;
	}

	if (surf[surf_no] != NULL)
	{
		ErrorLog("existing", surf_no);
		return FALSE;
	}

	HANDLE handle = LoadImageA(GetModuleHandleA(NULL), path, IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE | LR_CREATEDIBSECTION);
	if (handle == NULL)
	{
		ErrorLog(path, 1);
		return FALSE;
	}

	BITMAP bitmap;
	GetObjectA(handle, sizeof(BITMAP), &bitmap);

	DDSURFACEDESC ddsd;
	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);
	ddsd.dwFlags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH;
	ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN;
	ddsd.dwWidth = bitmap.bmWidth * mag;
	ddsd.dwHeight = bitmap.bmHeight * mag;

	lpDD->CreateSurface(&ddsd, &surf[surf_no], NULL);

	int src_x = 0;
	int src_y = 0;
	int src_w = bitmap.bmWidth;
	int src_h = bitmap.bmHeight;

	int dst_x = 0;
	int dst_y = 0;
	int dst_w = bitmap.bmWidth * mag;
	int dst_h = bitmap.bmHeight * mag;

	HDC hdc = CreateCompatibleDC(NULL);
	HGDIOBJ hgdiobj = SelectObject(hdc, handle);

	HDC hdc2;
	surf[surf_no]->GetDC(&hdc2);
	StretchBlt(hdc2, dst_x, dst_y, dst_w, dst_h, hdc, src_x, src_y, src_w, src_h, SRCCOPY);
	surf[surf_no]->ReleaseDC(hdc2);

	SelectObject(hdc, hgdiobj);
	DeleteDC(hdc);

	DDCOLORKEY ddcolorkey;
	ddcolorkey.dwColorSpaceLowValue = 0;
	ddcolorkey.dwColorSpaceHighValue = 0;

	surf[surf_no]->SetColorKey(DDCKEY_SRCBLT, &ddcolorkey);
	surf[surf_no]->SetClipper(clipper);

	DeleteObject(handle);

	surface_metadata[surf_no].type = SURFACE_SOURCE_FILE;
	surface_metadata[surf_no].width = bitmap.bmWidth;
	surface_metadata[surf_no].height = bitmap.bmHeight;
	surface_metadata[surf_no].bSystem = FALSE;
	strcpy(surface_metadata[surf_no].name, name);

	return TRUE;
}

// TODO - Inaccurate stack frame
BOOL ReloadBitmap_Resource(const char *name, SurfaceID surf_no)
{
	if (surf_no >= SURFACE_ID_MAX)
		return FALSE;

	HANDLE handle = LoadImageA(GetModuleHandleA(NULL), name, IMAGE_BITMAP, 0, 0, LR_CREATEDIBSECTION);
	if (handle == NULL)
		return FALSE;

	BITMAP bitmap;
	GetObjectA(handle, sizeof(BITMAP), &bitmap);

	int src_x = 0;
	int src_y = 0;
	int src_w = bitmap.bmWidth;
	int src_h = bitmap.bmHeight;

	int dst_x = 0;
	int dst_y = 0;
	int dst_w = bitmap.bmWidth * mag;
	int dst_h = bitmap.bmHeight * mag;

	HDC hdc = CreateCompatibleDC(NULL);
	HGDIOBJ hgdiobj = SelectObject(hdc, handle);

	HDC hdc2;
	surf[surf_no]->GetDC(&hdc2);
	StretchBlt(hdc2, dst_x, dst_y, dst_w, dst_h, hdc, src_x, src_y, src_w, src_h, SRCCOPY);
	surf[surf_no]->ReleaseDC(hdc2);

	SelectObject(hdc, hgdiobj);
	DeleteDC(hdc);

	DDCOLORKEY ddcolorkey;
	ddcolorkey.dwColorSpaceLowValue = 0;
	ddcolorkey.dwColorSpaceHighValue = 0;

	surf[surf_no]->SetColorKey(DDCKEY_SRCBLT, &ddcolorkey);
	surf[surf_no]->SetClipper(clipper);

#ifdef FIX_MAJOR_BUGS
	DeleteObject(handle);
#endif

	surface_metadata[surf_no].type = SURFACE_SOURCE_RESOURCE;
	strcpy(surface_metadata[surf_no].name, name);

	return TRUE;
}

// TODO - Inaccurate stack frame
BOOL ReloadBitmap_File(const char *name, SurfaceID surf_no)
{
	char path[MAX_PATH];
	sprintf(path, "%s\\%s.pbm", gDataPath, name);

	if (!IsEnableBitmap(path))
	{
		ErrorLog(path, 0);
		return FALSE;
	}

#ifdef FIX_BUGS
	if (surf_no >= SURFACE_ID_MAX)
#else
	if (surf_no > SURFACE_ID_MAX)
#endif
	{
		ErrorLog("surface no", surf_no);
		return FALSE;
	}

	HANDLE handle = LoadImageA(GetModuleHandleA(NULL), path, IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE | LR_CREATEDIBSECTION);
	if (handle == NULL)
	{
		ErrorLog(path, 1);
		return FALSE;
	}

	BITMAP bitmap;
	GetObjectA(handle, sizeof(BITMAP), &bitmap);

	int src_x = 0;
	int src_y = 0;
	int src_w = bitmap.bmWidth;
	int src_h = bitmap.bmHeight;

	int dst_x = 0;
	int dst_y = 0;
	int dst_w = bitmap.bmWidth * mag;
	int dst_h = bitmap.bmHeight * mag;

	HDC hdc = CreateCompatibleDC(NULL);
	HGDIOBJ hgdiobj = SelectObject(hdc, handle);

	HDC hdc2;
	surf[surf_no]->GetDC(&hdc2);
	StretchBlt(hdc2, dst_x, dst_y, dst_w, dst_h, hdc, src_x, src_y, src_w, src_h, SRCCOPY);
	surf[surf_no]->ReleaseDC(hdc2);

	SelectObject(hdc, hgdiobj);
	DeleteDC(hdc);

	// No colour-keying

	DeleteObject(handle);

	surface_metadata[surf_no].type = SURFACE_SOURCE_FILE;
	strcpy(surface_metadata[surf_no].name, name);

	return TRUE;
}

// TODO - Inaccurate stack frame
BOOL MakeSurface_Generic(int bxsize, int bysize, SurfaceID surf_no, BOOL bSystem)
{
#ifdef FIX_BUGS
	if (surf_no >= SURFACE_ID_MAX)
#else
	if (surf_no > SURFACE_ID_MAX)	// OOPS (should be '>=')
#endif
		return FALSE;

	if (surf[surf_no] != NULL)
		return FALSE;

	DDSURFACEDESC ddsd;
	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);
	ddsd.dwFlags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH;

	if (bSystem)
		ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN | DDSCAPS_SYSTEMMEMORY;
	else
		ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN;

	ddsd.dwWidth = bxsize * mag;
	ddsd.dwHeight = bysize * mag;

	lpDD->CreateSurface(&ddsd, &surf[surf_no], NULL);

	DDCOLORKEY ddcolorkey;
	ddcolorkey.dwColorSpaceLowValue = 0;
	ddcolorkey.dwColorSpaceHighValue = 0;

	surf[surf_no]->SetColorKey(DDCKEY_SRCBLT, &ddcolorkey);

	surface_metadata[surf_no].type = SURFACE_SOURCE_NONE;
	surface_metadata[surf_no].width = ddsd.dwWidth / mag;
	surface_metadata[surf_no].height = ddsd.dwHeight / mag;

	if (bSystem)
		surface_metadata[surf_no].bSystem = TRUE;
	else
		surface_metadata[surf_no].bSystem = FALSE;

	strcpy(surface_metadata[surf_no].name, "generic");

	return TRUE;
}

void BackupSurface(SurfaceID surf_no, const RECT *rect)
{
	static DDBLTFX ddbltfx;	// TODO - Not the original variable name

	memset(&ddbltfx, 0, sizeof(DDBLTFX));
	ddbltfx.dwSize = sizeof(DDBLTFX);

	static RECT rcSet;	// TODO - Not the original variable name
	rcSet.left = rect->left * mag;
	rcSet.top = rect->top * mag;
	rcSet.right = rect->right * mag;
	rcSet.bottom = rect->bottom * mag;

	surf[surf_no]->Blt(&rcSet, backbuffer, &rcSet, DDBLT_WAIT, &ddbltfx);
}

void PutBitmap3(const RECT *rcView, int x, int y, const RECT *rect, SurfaceID surf_no) // Transparency
{
	static RECT rcWork;
	static RECT rcSet;

	rcWork = *rect;

	if (x + rect->right - rect->left > rcView->right)
		rcWork.right -= (x + rect->right - rect->left) - rcView->right;

	if (x < rcView->left)
	{
		rcWork.left += rcView->left - x;
		x = rcView->left;
	}

	if (y + rect->bottom - rect->top > rcView->bottom)
		rcWork.bottom -= (y + rect->bottom - rect->top) - rcView->bottom;

	if (y < rcView->top)
	{
		rcWork.top += rcView->top - y;
		y = rcView->top;
	}

	rcSet.left = x;
	rcSet.top = y;
	rcSet.right = x + rcWork.right - rcWork.left;
	rcSet.bottom = y + rcWork.bottom - rcWork.top;

	rcWork.left *= mag;
	rcWork.top *= mag;
	rcWork.right *= mag;
	rcWork.bottom *= mag;

	rcSet.left *= mag;
	rcSet.top *= mag;
	rcSet.right *= mag;
	rcSet.bottom *= mag;

	backbuffer->Blt(&rcSet, surf[surf_no], &rcWork, DDBLT_KEYSRC | DDBLT_WAIT, NULL);
}

void PutBitmap4(const RECT *rcView, int x, int y, const RECT *rect, SurfaceID surf_no) // No Transparency
{
	static RECT rcWork;
	static RECT rcSet;

	rcWork = *rect;

	if (x + rect->right - rect->left > rcView->right)
		rcWork.right -= (x + rect->right - rect->left) - rcView->right;

	if (x < rcView->left)
	{
		rcWork.left += rcView->left - x;
		x = rcView->left;
	}

	if (y + rect->bottom - rect->top > rcView->bottom)
		rcWork.bottom -= (y + rect->bottom - rect->top) - rcView->bottom;

	if (y < rcView->top)
	{
		rcWork.top += rcView->top - y;
		y = rcView->top;
	}

	rcSet.left = x;
	rcSet.top = y;
	rcSet.right = x + rcWork.right - rcWork.left;
	rcSet.bottom = y + rcWork.bottom - rcWork.top;

	rcWork.left *= mag;
	rcWork.top *= mag;
	rcWork.right *= mag;
	rcWork.bottom *= mag;

	rcSet.left *= mag;
	rcSet.top *= mag;
	rcSet.right *= mag;
	rcSet.bottom *= mag;

	backbuffer->Blt(&rcSet, surf[surf_no], &rcWork, DDBLT_WAIT, NULL);
}

void Surface2Surface(int x, int y, const RECT *rect, SurfaceID to, SurfaceID from)
{
	static RECT rcWork;
	static RECT rcSet;

	rcWork.left = rect->left * mag;
	rcWork.top = rect->top * mag;
	rcWork.right = rect->right * mag;
	rcWork.bottom = rect->bottom * mag;

	rcSet.left = x;
	rcSet.top = y;
	rcSet.right = x + rect->right - rect->left;
	rcSet.bottom = y + rect->bottom - rect->top;

	rcSet.left *= mag;
	rcSet.top *= mag;
	rcSet.right *= mag;
	rcSet.bottom *= mag;

	surf[to]->Blt(&rcSet, surf[from], &rcWork, DDBLT_KEYSRC | DDBLT_WAIT, NULL);
}

// This converts a colour to the 'native' format by writing it
// straight to the framebuffer, and then reading it back
unsigned long GetCortBoxColor(COLORREF col)
{
	HDC hdc;

	if (backbuffer->GetDC(&hdc) != DD_OK)
		return 0xFFFFFFFF;

	COLORREF original_colour = GetPixel(hdc, 0, 0);
	SetPixel(hdc, 0, 0, col);
	backbuffer->ReleaseDC(hdc);

	DDSURFACEDESC ddsd;
	memset(&ddsd, 0, sizeof(DDSURFACEDESC));
	ddsd.dwSize = sizeof(DDSURFACEDESC);

	if (backbuffer->Lock(NULL, &ddsd, DDLOCK_WAIT, NULL) != DD_OK)
		return 0xFFFFFFFF;

	DWORD native_colour = *(DWORD*)ddsd.lpSurface;

	if (ddsd.ddpfPixelFormat.dwRGBBitCount < 32)
		native_colour &= (1 << ddsd.ddpfPixelFormat.dwRGBBitCount) - 1;

	backbuffer->Unlock(0);

	if (backbuffer->GetDC(&hdc) != DD_OK)
		return 0xFFFFFFFF;

	SetPixel(hdc, 0, 0, original_colour);
	backbuffer->ReleaseDC(hdc);

	return native_colour;
}

void CortBox(const RECT *rect, unsigned long col)
{
	static DDBLTFX ddbltfx;	// TODO - Not the original variable name
	memset(&ddbltfx, 0, sizeof(DDBLTFX));
	ddbltfx.dwSize = sizeof(DDBLTFX);
	ddbltfx.dwFillColor = col;

	static RECT rcSet;	// TODO - Not the original variable name
	rcSet.left = rect->left * mag;
	rcSet.top = rect->top * mag;
	rcSet.right = rect->right * mag;
	rcSet.bottom = rect->bottom * mag;

	backbuffer->Blt(&rcSet, 0, 0, DDBLT_COLORFILL | DDBLT_WAIT, &ddbltfx);
}

void CortBox2(const RECT *rect, unsigned long col, SurfaceID surf_no)
{
	static DDBLTFX ddbltfx;	// TODO - Not the original variable name
	memset(&ddbltfx, 0, sizeof(DDBLTFX));
	ddbltfx.dwSize = sizeof(DDBLTFX);
	ddbltfx.dwFillColor = col;

	static RECT rcSet;	// TODO - Not the original variable name
	rcSet.left = rect->left * mag;
	rcSet.top = rect->top * mag;
	rcSet.right = rect->right * mag;
	rcSet.bottom = rect->bottom * mag;

	surface_metadata[surf_no].type = SURFACE_SOURCE_NONE;

	surf[surf_no]->Blt(&rcSet, 0, 0, DDBLT_COLORFILL | DDBLT_WAIT, &ddbltfx);
}

// Dummied-out log function
// According to the Mac port, its name really is just "out".
BOOL out(char surface_identifier)
{
	// The actual name (and type) of these two variables are unknown
	char path[MAX_PATH];
	FILE *fp;

	(void)surface_identifier;
	(void)path;
	(void)fp;

	// There may have been some kind of 'OutputDebugStringA' call here,
	// like the one in 'EnumDevices_Callback' in 'Input.cpp'.
	// Pixel may have kept them wrapped in '#ifdef DEBUG' blocks.

	return TRUE;
}

// TODO - Probably not the original function name (this is an educated guess)
int RestoreSurfaces(void)
{
	int s;
	RECT rect;
	int surfaces_regenerated = 0;

	if (frontbuffer == NULL)
		return surfaces_regenerated;

	if (backbuffer == NULL)
		return surfaces_regenerated;

	if (frontbuffer->IsLost() == DDERR_SURFACELOST)
	{
		++surfaces_regenerated;
		frontbuffer->Restore();
		out('f');	// 'f' for 'frontbuffer'
	}

	if (backbuffer->IsLost() == DDERR_SURFACELOST)
	{
		++surfaces_regenerated;
		backbuffer->Restore();
		out('b');	// 'b' for 'backbuffer'
	}

	for (s = 0; s < SURFACE_ID_MAX; ++s)
	{
		if (surf[s] != NULL)
		{
			if (surf[s]->IsLost() == DDERR_SURFACELOST)
			{
				++surfaces_regenerated;
				surf[s]->Restore();
				out('0' + s);	// The number of the surface lost

				if (!surface_metadata[s].bSystem)
				{
					switch (surface_metadata[s].type)
					{
						case SURFACE_SOURCE_NONE:
							rect.left = 0;
							rect.top = 0;
							rect.right = surface_metadata[s].width;
							rect.bottom = surface_metadata[s].height;
							CortBox2(&rect, 0, (SurfaceID)s);
							break;

						case SURFACE_SOURCE_RESOURCE:
							ReloadBitmap_Resource(surface_metadata[s].name, (SurfaceID)s);
							break;

						case SURFACE_SOURCE_FILE:
							ReloadBitmap_File(surface_metadata[s].name, (SurfaceID)s);
							break;
					}
				}
			}
		}
	}

	return surfaces_regenerated;
}

// TODO - Inaccurate stack frame
void InitTextObject(const char *name)
{
	// Get font size
	unsigned int width, height;

	// Let me tell you why these font sizes are unfortunate...
	// 6x12 is a good font size - fonts use high-quality bitmaps at that
	// size, and it works with Cave Story's internal assumption that
	// characters are spaced 6 pixels apart.
	// The sad part is the 10x20 size: you might be wondering why Pixel
	// didn't use 12x24 instead. Well, that's because fonts don't use
	// bitmaps at that size - instead you get ugly low-res vector
	// renders. So, Pixel had to use 10x20 instead. But there's a
	// problem: this means the characters are spaced 5 pixels apart
	// instead. This normally isn't a problem because the game usually
	// hardcodes it, but this isn't the case when either <SAT is used, a
	// texture is regenerated, or when the game prints the name of the
	// map. This causes the font to render with incorrect spacing.
	// There's really not much of a solution to this, especially when
	// you consider that the English translation was specifically built
	// around the broken spacing.

	switch (mag)
	{
		case 1:
			height = 12;
			width = 6;
			break;

		case 2:
			height = 20;
			width = 10;
			break;
	}

	// The game uses DEFAULT_CHARSET when it should have used SHIFTJIS_CHARSET.
	// This breaks the Japanese text on English Windows installations.

	// Also, the game uses DEFAULT_QUALITY instead of NONANTIALIASED_QUALITY,
	// causing font antialiasing to blend with the colour-key, giving text an
	// ugly black outline.
#ifdef FIX_BUGS
	#define QUALITY NONANTIALIASED_QUALITY
	#ifdef JAPANESE
		#define CHARSET SHIFTJIS_CHARSET
	#else
		#define CHARSET DEFAULT_CHARSET
	#endif
#else
	#define QUALITY DEFAULT_QUALITY
	#define CHARSET DEFAULT_CHARSET
#endif

	font = CreateFontA(height, width, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, QUALITY, FIXED_PITCH | FF_DONTCARE, name);

	if (font == NULL)
		font = CreateFontA(height, width, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, QUALITY, FIXED_PITCH | FF_DONTCARE, NULL);
}

void PutText(int x, int y, const char *text, unsigned long color)
{
	HDC hdc;
	backbuffer->GetDC(&hdc);
	HGDIOBJ hgdiobj = SelectObject(hdc, font);
	SetBkMode(hdc, 1);
	SetTextColor(hdc, color);
	TextOutA(hdc, x * mag, y * mag, text, (int)strlen(text));
	SelectObject(hdc, hgdiobj);
	backbuffer->ReleaseDC(hdc);
}

void PutText2(int x, int y, const char *text, unsigned long color, SurfaceID surf_no)
{
	HDC hdc;
	surf[surf_no]->GetDC(&hdc);
	HGDIOBJ hgdiobj = SelectObject(hdc, font);
	SetBkMode(hdc, 1);
	SetTextColor(hdc, color);
	TextOutA(hdc, x * mag, y * mag, text, (int)strlen(text));
	SelectObject(hdc, hgdiobj);
	surf[surf_no]->ReleaseDC(hdc);
}

void EndTextObject(void)
{
	DeleteObject(font);
}

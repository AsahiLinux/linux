// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

// Some of the original source code for this file can be found here:
// https://github.com/shbow/organya/blob/master/source/Sound.cpp

/*
TODO - Code style
Pixel's code was *extremely* Windows-centric, to the point of using
things like ZeroMemory and LPCSTR instead of standard things like
memset and const char*. For now, the decompilation is accurate despite
not using these since they're just macros that evaluate to the portable
equivalents.
*/

#include "Sound.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIRECTSOUND_VERSION 0x500
#include <dsound.h>

#include "WindowsWrapper.h"

#include "Main.h"
#include "Organya.h"
#include "PixTone.h"

LPDIRECTSOUND       lpDS;            // DirectSoundオブジェクト (DirectSound object)
LPDIRECTSOUNDBUFFER lpPRIMARYBUFFER; // 一時バッファ (Temporary buffer)
LPDIRECTSOUNDBUFFER lpSECONDARYBUFFER[SE_MAX]; 

// DirectSoundの開始 (Starting DirectSound)
BOOL InitDirectSound(HWND hwnd)
{
	int i;
	DSBUFFERDESC dsbd;

	// DirectDrawの初期化 (DirectDraw initialization)
	if (DirectSoundCreate(NULL, &lpDS, NULL) != DS_OK)
	{
		lpDS = NULL;
	#ifndef FIX_BUGS
		// This makes absolutely no sense here
		StartOrganya(lpDS, "Org\\Wave.dat");
	#endif
		return FALSE;
	}

	lpDS->SetCooperativeLevel(hwnd, DSSCL_EXCLUSIVE);

	// 一次バッファの初期化 (Initializing the primary buffer)
	ZeroMemory(&dsbd, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	dsbd.dwFlags = DSBCAPS_PRIMARYBUFFER | DSBCAPS_CTRLPAN | DSBCAPS_CTRLVOLUME;
	lpDS->CreateSoundBuffer(&dsbd, &lpPRIMARYBUFFER, NULL);

	for (i = 0; i < SE_MAX; i++)
		lpSECONDARYBUFFER[i] = NULL;

	StartOrganya(lpDS, "Org\\Wave.dat");

	return TRUE;
}

// DirectSoundの終了 (Exit DirectSound)
void EndDirectSound(void)
{
	int i;

	if (lpDS == NULL)
		return;

	EndOrganya();

	for (i = 0; i < SE_MAX; i++)
		if (lpSECONDARYBUFFER[i] != NULL)
			lpSECONDARYBUFFER[i]->Release();

	if (lpPRIMARYBUFFER != NULL)
		lpPRIMARYBUFFER->Release();

	if (lpDS != NULL)
		lpDS->Release();

	lpDS = NULL;
}

// Below are two completely unused functions for loading .wav files as sound effects.
// Some say that sounds heard in CS Beta footage don't sound like PixTone...

// There's a bit of a problem with this code: it hardcodes the offsets of various bits
// of data in the WAV header - this makes the code only compatible with very specific
// .wav files. You can check the prototype OrgView EXEs for examples of those.

// サウンドの設定 (Sound settings)
BOOL InitSoundObject(LPCSTR resname, int no)
{
	HRSRC hrscr;
	DSBUFFERDESC dsbd;
	DWORD *lpdword;	// リソースのアドレス (Resource address)

	if (lpDS == NULL)
		return TRUE;

	// リソースの検索 (Search for resources)
	if ((hrscr = FindResourceA(NULL, resname, "WAVE")) == NULL)
		return FALSE;

	// リソースのアドレスを取得 (Get resource address)
	lpdword = (DWORD*)LockResource(LoadResource(NULL, hrscr));

	// 二次バッファの生成 (Create secondary buffer)
	ZeroMemory(&dsbd, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	dsbd.dwFlags = DSBCAPS_STATIC | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPAN | DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLFREQUENCY;
	dsbd.dwBufferBytes = *(DWORD*)((BYTE*)lpdword+0x36);	// WAVEデータのサイズ (WAVE data size)
	dsbd.lpwfxFormat = (LPWAVEFORMATEX)(lpdword+5); 

	if (lpDS->CreateSoundBuffer(&dsbd, &lpSECONDARYBUFFER[no], NULL) != DS_OK)
		return FALSE;

	LPVOID lpbuf1, lpbuf2;
	DWORD dwbuf1, dwbuf2;

	// 二次バッファのロック (Secondary buffer lock)
	lpSECONDARYBUFFER[no]->Lock(0, *(DWORD*)((BYTE*)lpdword+0x36), &lpbuf1, &dwbuf1, &lpbuf2, &dwbuf2, 0); 

	// 音源データの設定 (Sound source data settings)
	CopyMemory(lpbuf1, (BYTE*)lpdword+0x3A, dwbuf1);

	if (dwbuf2 != 0)
		CopyMemory(lpbuf2, (BYTE*)lpdword+0x3A+dwbuf1, dwbuf2);

	// 二次バッファのロック解除 (Unlock secondary buffer)
	lpSECONDARYBUFFER[no]->Unlock(lpbuf1, dwbuf1, lpbuf2, dwbuf2); 

	return TRUE;
}

BOOL LoadSoundObject(LPCSTR file_name, int no)
{
	char path[MAX_PATH];
	DWORD i;
	DWORD file_size = 0;
	char check_box[58];
	FILE *fp;
	HANDLE hFile;

	sprintf(path, "%s\\%s", gModulePath, file_name);

	if (lpDS == NULL)
		return TRUE;

	hFile = CreateFileA(path, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return FALSE;

	file_size = GetFileSize(hFile, NULL);
	CloseHandle(hFile);

	if ((fp = fopen(path, "rb")) == NULL)
		return FALSE;

	for (i = 0; i < 58; i++)
		fread(&check_box[i], sizeof(char), 1, fp);	// Holy hell, this is inefficient

#ifdef FIX_MAJOR_BUGS
	// The original code forgets to close 'fp'
	if (check_box[0] != 'R' || check_box[1] != 'I' || check_box[2] != 'F' || check_box[3] != 'F')
	{
		fclose(fp);
		return FALSE;
	}
#else
	if (check_box[0] != 'R')
		return FALSE;
	if (check_box[1] != 'I')
		return FALSE;
	if (check_box[2] != 'F')
		return FALSE;
	if (check_box[3] != 'F')
		return FALSE;
#endif

	DWORD *wp;
	wp = (DWORD*)malloc(file_size);	// ファイルのワークスペースを作る (Create a file workspace)

#ifdef FIX_MAJOR_BUGS
	if (wp == NULL)
	{
		fclose(fp);
		return FALSE;
	}
#endif

	fseek(fp, 0, SEEK_SET);

	for (i = 0; i < file_size; i++)
		fread((BYTE*)wp+i, sizeof(BYTE), 1, fp);	// Pixel, stahp

	fclose(fp);

	// セカンダリバッファの生成 (Create secondary buffer)
	DSBUFFERDESC dsbd;
	ZeroMemory(&dsbd, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	dsbd.dwFlags = DSBCAPS_STATIC | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPAN | DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLFREQUENCY;
	dsbd.dwBufferBytes = *(DWORD*)((BYTE*)wp+0x36);	// WAVEデータのサイズ (WAVE data size)
	dsbd.lpwfxFormat = (LPWAVEFORMATEX)(wp+5); 

	if (lpDS->CreateSoundBuffer(&dsbd, &lpSECONDARYBUFFER[no], NULL) != DS_OK)
	{
#ifdef FIX_MAJOR_BUGS
		free(wp);	// The updated Organya source code includes this fix
#endif
		return FALSE;	
	}

	LPVOID lpbuf1, lpbuf2;
	DWORD dwbuf1, dwbuf2;

	HRESULT hr;
	hr = lpSECONDARYBUFFER[no]->Lock(0, *(DWORD*)((BYTE*)wp+0x36), &lpbuf1, &dwbuf1, &lpbuf2, &dwbuf2, 0);

	if (hr != DS_OK)
	{
#ifdef FIX_MAJOR_BUGS
		free(wp);	// The updated Organya source code includes this fix
#endif
		return FALSE;
	}

	CopyMemory(lpbuf1, (BYTE*)wp+0x3A, dwbuf1);	// +3aはデータの頭 (+ 3a is the head of the data)

	if (dwbuf2 != 0)
		CopyMemory(lpbuf2, (BYTE*)wp+0x3A+dwbuf1, dwbuf2);

	lpSECONDARYBUFFER[no]->Unlock(lpbuf1, dwbuf1, lpbuf2, dwbuf2); 
	
	free(wp);

	return TRUE;
}

void PlaySoundObject(int no, SoundMode mode)
{
	if (lpDS == NULL)
		return;

	if (lpSECONDARYBUFFER[no] != NULL)
	{
		switch (mode)
		{
			case SOUND_MODE_STOP:	// 停止 (Stop)
				lpSECONDARYBUFFER[no]->Stop();
				break;

			case SOUND_MODE_PLAY:	// 再生 (Playback)
				lpSECONDARYBUFFER[no]->Stop();
				lpSECONDARYBUFFER[no]->SetCurrentPosition(0);
				lpSECONDARYBUFFER[no]->Play(0, 0, 0);
				break;

			case SOUND_MODE_PLAY_LOOP:// ループ再生 (Loop playback)
				lpSECONDARYBUFFER[no]->Play(0, 0, DSBPLAY_LOOPING);
				break;
		}
	}
}

void ChangeSoundFrequency(int no, DWORD rate)	// 100がMIN9999がMAXで2195?がﾉｰﾏﾙ (100 is MIN, 9999 is MAX, and 2195 is normal)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[no]->SetFrequency((rate * 10) + 100);
}

void ChangeSoundVolume(int no, long volume)	// 300がMAXで300がﾉｰﾏﾙ (300 is MAX and 300 is normal)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[no]->SetVolume((volume - 300) * 8);
}

void ChangeSoundPan(int no, long pan)	// 512がMAXで256がﾉｰﾏﾙ (512 is MAX and 256 is normal)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[no]->SetPan((pan - 256) * 10);
}

// TODO - The stack frame for this function is inaccurate
int MakePixToneObject(const PIXTONEPARAMETER *ptp, int ptp_num, int no)
{
	// For some reason, this function creates an entire WAV file header,
	// when it only needs a WAVEFORMATEX.
	// From what I can tell, there's no struct like this in the Windows
	// headers, so Pixel must have defined it manually, just like this:
	typedef struct WavHeader
	{
		FOURCC riff_id;
		DWORD riff_size;
		FOURCC wave_id;
		FOURCC fmt_id;
		DWORD fmt_size;
		PCMWAVEFORMAT format;
		FOURCC data_id;
		DWORD data_size;
	} WavHeader;

	int sample_count;
	int i, j;
	DSBUFFERDESC dsbd;
	WavHeader wav_header;
	const PIXTONEPARAMETER *ptp_pointer;
	unsigned char *pcm_buffer;
	unsigned char *mixed_pcm_buffer;

	if (lpDS == NULL)
		return 0;

	const char *riff = "RIFF";
	const char *fmt  = "fmt ";
	const char *wave = "WAVE";
	const char *data = "data";

	wav_header.format.wBitsPerSample = 8;
	wav_header.format.wf.nSamplesPerSec = 22050;
	wav_header.format.wf.nChannels = 1;
	wav_header.format.wf.wFormatTag = WAVE_FORMAT_PCM;
	wav_header.fmt_size = sizeof(wav_header.format);
	memcpy(&wav_header.riff_id, riff, sizeof(FOURCC));
	memcpy(&wav_header.fmt_id, fmt, sizeof(FOURCC));
	memcpy(&wav_header.wave_id, wave, sizeof(FOURCC));
	memcpy(&wav_header.data_id, data, sizeof(FOURCC));
	wav_header.format.wf.nBlockAlign = (wav_header.format.wBitsPerSample / 8) * wav_header.format.wf.nChannels;
	wav_header.format.wf.nAvgBytesPerSec = (wav_header.format.wBitsPerSample / 8) * wav_header.format.wf.nChannels * wav_header.format.wf.nSamplesPerSec;
	wav_header.data_size = wav_header.format.wf.nBlockAlign * ptp->size;	// Note that this uses ptp->size, not sample_count. If this header were ever used, it would be incorrect.
	wav_header.riff_size = sizeof(wav_header) - 8 + wav_header.data_size;

	ptp_pointer = ptp;
	sample_count = 0;

	for (i = 0; i < ptp_num; i++)
	{
		if (ptp_pointer->size > sample_count)
			sample_count = ptp_pointer->size;

		++ptp_pointer;
	}

	ZeroMemory(&dsbd, sizeof(dsbd));
	dsbd.dwSize = sizeof(dsbd);
	dsbd.dwFlags = DSBCAPS_STATIC | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPAN | DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLFREQUENCY;
	dsbd.dwBufferBytes = sample_count;
	dsbd.lpwfxFormat = (LPWAVEFORMATEX)&wav_header.format;

	if (lpDS->CreateSoundBuffer(&dsbd, &lpSECONDARYBUFFER[no], 0) != DS_OK)
		return -1;

	pcm_buffer = mixed_pcm_buffer = NULL;

	pcm_buffer = (unsigned char*)malloc(sample_count);
	mixed_pcm_buffer = (unsigned char*)malloc(sample_count);

	if (pcm_buffer == NULL || mixed_pcm_buffer == NULL)
	{
		if (pcm_buffer != NULL)
			free(pcm_buffer);

		if (mixed_pcm_buffer != NULL)
			free(mixed_pcm_buffer);

		return -1;
	}

	memset(pcm_buffer, 0x80, sample_count);
	memset(mixed_pcm_buffer, 0x80, sample_count);

	ptp_pointer = ptp;

	for (i = 0; i < ptp_num; i++)
	{
		if (!MakePixelWaveData(ptp_pointer, pcm_buffer))
		{
			if (pcm_buffer != NULL) // This is always true
				free(pcm_buffer);

			if (mixed_pcm_buffer != NULL) // This is always true
				free(mixed_pcm_buffer);

			return -1;
		}

		for (j = 0; j < ptp_pointer->size; j++)
		{
			if (pcm_buffer[j] + mixed_pcm_buffer[j] - 0x100 < -0x7F)
				mixed_pcm_buffer[j] = 0;
			else if (pcm_buffer[j] + mixed_pcm_buffer[j] - 0x100 > 0x7F)
				mixed_pcm_buffer[j] = 0xFF;
			else
				mixed_pcm_buffer[j] = mixed_pcm_buffer[j] + pcm_buffer[j] - 0x80;
		}

		++ptp_pointer;
	}

	// This is self-assignment, so redundant. Maybe this used to be something to prevent audio popping ?
	mixed_pcm_buffer[0] = mixed_pcm_buffer[0];
	mixed_pcm_buffer[sample_count - 1] = mixed_pcm_buffer[sample_count - 1];

	LPVOID lpbuf1, lpbuf2;
	DWORD dwbuf1, dwbuf2;

	lpSECONDARYBUFFER[no]->Lock(0, sample_count, &lpbuf1, &dwbuf1, &lpbuf2, &dwbuf2, 0); 

	CopyMemory(lpbuf1, mixed_pcm_buffer, dwbuf1);

	if (dwbuf2 != 0)
		CopyMemory(lpbuf2, mixed_pcm_buffer + dwbuf1, dwbuf2);

	lpSECONDARYBUFFER[no]->Unlock(lpbuf1, dwbuf1, lpbuf2, dwbuf2);

	if (pcm_buffer != NULL)
		free(pcm_buffer);

	if (mixed_pcm_buffer != NULL)
		free(mixed_pcm_buffer);

	return sample_count;
}

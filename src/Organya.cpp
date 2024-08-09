// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

// Some of the original source code for this file can be found here:
// https://github.com/shbow/organya/blob/master/source/OrgFile.cpp
// https://github.com/shbow/organya/blob/master/source/OrgPlay.cpp
// https://github.com/shbow/organya/blob/master/source/Sound.cpp
// https://github.com/shbow/organya/blob/master/source/WinTimer.cpp

#include "Organya.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef FIX_BUGS
// The original source code forgot to set this (you can tell because, in the original EXE,
// the DSBUFFERDESC structs in this file have a different size to the ones in Sound.cpp)
#define DIRECTSOUND_VERSION 0x500
#endif
#include <dsound.h>

#include "WindowsWrapper.h"

#include "Sound.h"

#define PANDUMMY 0xFF
#define VOLDUMMY 0xFF
#define KEYDUMMY 0xFF

#define ALLOCNOTE 4096

#define DEFVOLUME	200//255はVOLDUMMY。MAXは254
#define DEFPAN		6

//曲情報をセットする時のフラグ
#define SETALL		0xffffffff//全てをセット
#define SETWAIT		0x00000001
#define SETGRID		0x00000002
#define SETALLOC	0x00000004
#define SETREPEAT	0x00000008
#define SETFREQ		0x00000010
#define SETWAVE		0x00000020
#define SETPIPI		0x00000040

typedef struct ORGANYATRACK
{
	unsigned short freq;	// +α周波数(1000がDefault) (+ α frequency (1000 is Default))
	unsigned char wave_no;	// 波形No (Waveform No)
	unsigned char pipi;	// ☆
	unsigned short note_num;	// 音符の数 (Number of notes)
} ORGANYATRACK;

typedef struct ORGANYADATA
{
	unsigned short wait;
	unsigned char line;
	unsigned char dot;
	long repeat_x;	// リピート (repeat)
	long end_x;	// 曲の終わり(リピートに戻る)	(End of song (return to repeat))
	ORGANYATRACK tdata[MAXTRACK];
} ORGANYADATA;

// Below are Organya song data structures
typedef struct NOTELIST
{
	NOTELIST *from;	// Previous address
	NOTELIST *to;	// Next address

	long x;	// Position
	unsigned char length;	// Sound length
	unsigned char y;	// Sound height
	unsigned char volume;	// Volume
	unsigned char pan;
} NOTELIST;

// Track data * 8
typedef struct TRACKDATA
{
	unsigned short freq;	// Frequency (1000 is default)
	unsigned char wave_no;	// Waveform No.
	signed char pipi;

	NOTELIST *note_p;
	NOTELIST *note_list;
} TRACKDATA;

// Unique information held in songs
typedef struct MUSICINFO
{
	unsigned short wait;
	unsigned char line;	// Number of lines in one measure
	unsigned char dot;	// Number of dots per line
	unsigned short alloc_note;	// Number of allocated notes
	long repeat_x;	// Repeat
	long end_x;	// End of song (Return to repeat)
	TRACKDATA tdata[MAXTRACK];
} MUSICINFO;

// メインクラス。このアプリケーションの中心。（クラスってやつを初めて使う） (Main class. The heart of this application. (Class is used for the first time))
typedef struct OrgData
{
	OrgData();	// コンストラクタ (Constructor)
//	~OrgData();	// デストラクタ (Destructor)
	MUSICINFO info;
	char track;
	char mute[MAXTRACK];
	unsigned char def_pan;
	unsigned char def_volume;
	void InitOrgData(void);
	void GetMusicInfo(MUSICINFO *mi);	// 曲情報を取得 (Get song information)
	// 曲情報を設定。flagは設定アイテムを指定 (Set song information. flag specifies the setting item)
	BOOL SetMusicInfo(MUSICINFO *mi,unsigned long flag);
	BOOL NoteAlloc(unsigned short note_num);	// 指定の数だけNoteDataの領域を確保 (Allocate the specified number of NoteData areas.)
	void ReleaseNote(void);	// NoteDataを開放 (Release NoteData)
	// 以下は再生 (The following is playback)
	void PlayData(void);
	void SetPlayPointer(long x);	// 再生ポインターを指定の位置に設定 (Set playback pointer to specified position)
	// 以下はファイル関係 (The following are related to files)
	BOOL InitMusicData(const char *path);
} ORGDATA;

LPDIRECTSOUNDBUFFER lpORGANBUFFER[8][8][2] = {NULL};

/////////////////////////////////////////////
//■オルガーニャ■■■■■■■■■■■■/////// (Organya)
/////////////////////

// Wave playing and loading
typedef struct
{
	short wave_size;
	short oct_par;
	short oct_size;
} OCTWAVE;

OCTWAVE oct_wave[8] =
{
	{ 256,  1,  4 }, // 0 Oct
	{ 256,  2,  8 }, // 1 Oct
	{ 128,  4, 12 }, // 2 Oct
	{ 128,  8, 16 }, // 3 Oct
	{  64, 16, 20 }, // 4 Oct
	{  32, 32, 24 }, // 5 Oct
	{  16, 64, 28 }, // 6 Oct
	{   8,128, 32 }, // 7 Oct
};

WAVEFORMATEX format_tbl2 = {WAVE_FORMAT_PCM, 1, 22050, 22050, 1, 8, 0};	// 22050HzのFormat

// In the original source code, format_tbl2 was a raw array of bytes, as seen below
// BYTE format_tbl2[] = {0x01,0x00,0x01,0x00,0x22,0x56,0x00,0x00,0x22,0x56,0x00,0x00,0x01,0x00,0x08,0x00,0x00,0x00};	// 22050HzのFormat

BOOL MakeSoundObject8(signed char *wavep, signed char track, signed char pipi)
{
	DWORD i,j,k;
	unsigned long wav_tp;	// WAVテーブルをさすポインタ (Pointer to WAV table)
	DWORD wave_size;	// 256;
	DWORD data_size;
	BYTE *wp;
	BYTE *wp_sub;
	int work;
	// セカンダリバッファの生成 (Create secondary buffer)
	DSBUFFERDESC dsbd;

	if (lpDS == NULL)
		return FALSE;

	for (j = 0; j < 8; j++)
	{
		for (k = 0; k < 2; k++)
		{
			wave_size = oct_wave[j].wave_size;

			if (pipi)
				data_size = wave_size * oct_wave[j].oct_size;
			else
				data_size = wave_size;

			ZeroMemory(&dsbd, sizeof(dsbd));

			dsbd.dwSize = sizeof(dsbd);
			dsbd.dwBufferBytes = data_size;
			dsbd.lpwfxFormat = &format_tbl2;
			dsbd.dwFlags = DSBCAPS_STATIC | DSBCAPS_GLOBALFOCUS | DSBCAPS_CTRLPAN | DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLFREQUENCY;

			if(lpDS->CreateSoundBuffer(&dsbd, &lpORGANBUFFER[track][j][k], NULL) != DS_OK)	// j = se_no
				return FALSE;

			// Get wave data
			wp = (BYTE*)malloc(data_size);
			wp_sub = wp;
			wav_tp = 0;

			for (i = 0; i < data_size; i++)
			{
				work = *(wavep + wav_tp);
				work += 0x80;

				*wp_sub = (BYTE)work;

				wav_tp += 0x100 / wave_size;
				if (wav_tp > 0xFF)
					wav_tp -= 0x100;

				wp_sub++;
			}

			// データの転送 (Data transfer)
			LPVOID lpbuf1, lpbuf2;
			DWORD dwbuf1, dwbuf2=0;
			HRESULT hr;

			hr = lpORGANBUFFER[track][j][k]->Lock(0, data_size, &lpbuf1, &dwbuf1, &lpbuf2, &dwbuf2, 0); 

			if (hr != DS_OK)
			{
			#ifdef FIX_MAJOR_BUGS
				free(wp);	// The updated Organya source code includes this fix
			#endif
				return FALSE;
			}

			CopyMemory(lpbuf1, (BYTE*)wp, dwbuf1);

			if (dwbuf2 != 0)
				CopyMemory(lpbuf2, (BYTE*)wp+dwbuf1, dwbuf2);

			lpORGANBUFFER[track][j][k]->Unlock(lpbuf1, dwbuf1, lpbuf2, dwbuf2);
			lpORGANBUFFER[track][j][k]->SetCurrentPosition(0);
			free(wp);
		}
	}

	return TRUE;
}

short freq_tbl[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};

void ChangeOrganFrequency(unsigned char key, signed char track, long a)
{
	if (lpDS == NULL)
		return;

	for (int j = 0; j < 8; j++)
		for (int i = 0; i < 2; i++)
			lpORGANBUFFER[track][j][i]->SetFrequency(((oct_wave[j].wave_size * freq_tbl[key]) * oct_wave[j].oct_par) / 8 + (a - 1000));	// 1000を+αのデフォルト値とする (1000 is the default value for + α)
}

BOOL g_mute[MAXTRACK];	// Used by the debug Mute menu
short pan_tbl[13] = {0, 43, 86, 129, 172, 215, 256, 297, 340, 383, 426, 469, 512};
unsigned char old_key[MAXTRACK] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// 再生中の音 (Sound being played)
unsigned char key_on[MAXTRACK];	// キースイッチ (Key switch)
unsigned char key_twin[MAXTRACK];	// 今使っているキー(連続時のノイズ防止の為に二つ用意) (Currently used keys (prepared for continuous noise prevention))

void ChangeOrganPan(unsigned char key, unsigned char pan, signed char track)	// 512がMAXで256がﾉｰﾏﾙ (512 is MAX and 256 is normal)
{
	if (lpDS == NULL)
		return;

	if (old_key[track] != KEYDUMMY)
		lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->SetPan((pan_tbl[pan] - 0x100) * 10);
}

void ChangeOrganVolume(int no, long volume, signed char track)	// 300がMAXで300がﾉｰﾏﾙ (300 is MAX and 300 is normal)
{
	if (lpDS == NULL)
		return;

	if (old_key[track] != KEYDUMMY)
		lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->SetVolume((volume - 0xFF) * 8);
}

// サウンドの再生 (Play sound)
void PlayOrganObject(unsigned char key, int mode, signed char track, long freq)
{
	if (lpDS == NULL)
		return;

	if (lpORGANBUFFER[track][key / 12][key_twin[track]] != NULL)
	{
		switch (mode)
		{
			case 0:	// 停止 (Stop)
				if (old_key[track] != 0xFF)
				{
					lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->Stop();
					lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->SetCurrentPosition(0);
				}
				break;

			case 1: // 再生 (Playback)
				break;

			case 2:	// 歩かせ停止 (Stop playback)
				if (old_key[track] != 0xFF)
				{
					lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->Play(0, 0, 0);
					old_key[track] = 0xFF;
				}
				break;

			case -1:
				if (old_key[track] == 0xFF)	// 新規鳴らす (New sound)
				{
					ChangeOrganFrequency(key % 12, track, freq);	// 周波数を設定して (Set the frequency)
					lpORGANBUFFER[track][key / 12][key_twin[track]]->Play(0, 0, DSBPLAY_LOOPING);
					old_key[track] = key;
					key_on[track] = 1;
				}
				else if (key_on[track] == 1 && old_key[track] == key)	// 同じ音 (Same sound)
				{
					// 今なっているのを歩かせ停止 (Stop playback now)
					lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->Play(0, 0, 0);
					key_twin[track]++;
					if (key_twin[track] > 1)
						key_twin[track] = 0;
					lpORGANBUFFER[track][key / 12][key_twin[track]]->Play(0, 0, DSBPLAY_LOOPING);
				}
				else	// 違う音を鳴らすなら (If you make a different sound)
				{
					lpORGANBUFFER[track][old_key[track] / 12][key_twin[track]]->Play(0, 0, 0);	// 今なっているのを歩かせ停止 (Stop playback now)
					key_twin[track]++;
					if (key_twin[track] > 1)
						key_twin[track] = 0;
					ChangeOrganFrequency(key % 12, track, freq);	// 周波数を設定して (Set the frequency)
					lpORGANBUFFER[track][key / 12][key_twin[track]]->Play(0, 0, DSBPLAY_LOOPING);
					old_key[track] = key;
				}

				break;
		}
	}
}

// オルガーニャオブジェクトを開放 (Open Organya object)
void ReleaseOrganyaObject(signed char track)
{
	if (lpDS == NULL)
		return;

	for (int i = 0; i < 8; i++)
	{
		if (lpORGANBUFFER[track][i][0] != NULL)
		{
			lpORGANBUFFER[track][i][0]->Release();
			lpORGANBUFFER[track][i][0] = NULL;
		}
		if (lpORGANBUFFER[track][i][1] != NULL)
		{
			lpORGANBUFFER[track][i][1]->Release();
			lpORGANBUFFER[track][i][1] = NULL;
		}
	}
}

// 波形データをロード (Load waveform data)
signed char wave_data[100][0x100];

BOOL InitWaveData100(void)
{
	HRSRC hrscr;
	DWORD *lpdword;	// リソースのアドレス (Resource address)

	if (lpDS == NULL)
		return FALSE;

	// リソースの検索 (Search for resources)
	hrscr = FindResourceA(NULL, "WAVE100", "WAVE");

	if (hrscr == NULL)
		return FALSE;

	// リソースのアドレスを取得 (Get resource address)
	lpdword = (DWORD*)LockResource(LoadResource(NULL, hrscr));
	memcpy(wave_data, lpdword, 100 * 0x100);

	return TRUE;
}

// 波形を１００個の中から選択して作成 (Select from 100 waveforms to create)
BOOL MakeOrganyaWave(signed char track, signed char wave_no, signed char pipi)
{
	if (lpDS == NULL)
		return FALSE;

	if (wave_no > 99)
		return FALSE;

	ReleaseOrganyaObject(track);
	MakeSoundObject8(wave_data[wave_no], track, pipi);

	return TRUE;
}

/////////////////////////////////////////////
//■オルガーニャドラムス■■■■■■■■/////// (Organya drums)
/////////////////////

void ChangeDramFrequency(unsigned char key, signed char track)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[150 + track]->SetFrequency(key * 800 + 100);
}

void ChangeDramPan(unsigned char pan, signed char track)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[150 + track]->SetPan((pan_tbl[pan] - 0x100) * 10);
}

void ChangeDramVolume(long volume, signed char track)
{
	if (lpDS == NULL)
		return;

	lpSECONDARYBUFFER[150 + track]->SetVolume((volume - 0xFF) * 8);
}

// サウンドの再生 (Play sound)
void PlayDramObject(unsigned char key, int mode, signed char track)
{
	if (lpDS == NULL)
		return;

	if (lpSECONDARYBUFFER[150 + track] != NULL)
	{
		switch (mode)
		{
			case 0:	// 停止 (Stop)
				lpSECONDARYBUFFER[150 + track]->Stop();
				lpSECONDARYBUFFER[150 + track]->SetCurrentPosition(0);
				break;

			case 1:	// 再生 (Playback)
				lpSECONDARYBUFFER[150 + track]->Stop();
				lpSECONDARYBUFFER[150 + track]->SetCurrentPosition(0);
				ChangeDramFrequency(key, track);	// 周波数を設定して (Set the frequency)
				lpSECONDARYBUFFER[150 + track]->Play(0, 0, 0);
				break;

			case 2:	// 歩かせ停止 (Stop playback)
				break;

			case -1:
				break;
		}
	}
}

ORGDATA org_data;

OrgData::OrgData(void)
{
	for (int i = 0; i < MAXTRACK; i++)
	{
		info.tdata[i].note_list = NULL;
		info.tdata[i].note_p = NULL;
	}
}

void OrgData::InitOrgData(void)
{
	track = 0;
	info.alloc_note = ALLOCNOTE;	// とりあえず10000個確保 (For the time being, secure 10,000 pieces)
	info.dot = 4;
	info.line = 4;
	info.wait = 128;
	info.repeat_x = info.dot * info.line * 0;
	info.end_x = info.dot * info.line * 255;

	for (int i = 0; i < MAXTRACK; i++)
	{
		info.tdata[i].freq = 1000;
		info.tdata[i].wave_no = 0;
		info.tdata[i].pipi = 0;
	}

	NoteAlloc(info.alloc_note);
	SetMusicInfo(&info, SETALL);

	def_pan = DEFPAN;
	def_volume = DEFVOLUME;
}

// 曲情報を設定。flagはアイテムを指定 (Set song information. flag specifies an item)
BOOL OrgData::SetMusicInfo(MUSICINFO *mi, unsigned long flag)
{
	char str[32];	// Leftover debug junk
	int i;

	if (flag & SETGRID)	// グリッドを有効に (Enable grid)
	{
		info.dot = mi->dot;
		info.line = mi->line;
	}

	if (flag & SETWAIT)
	{
		info.wait = mi->wait;
		itoa(mi->wait, str, 10);	// Leftover debug junk
	}

	if (flag & SETREPEAT)
	{
		info.repeat_x = mi->repeat_x;
		info.end_x = mi->end_x;
	}

	if (flag & SETFREQ)
	{
		for (i = 0; i < MAXMELODY; i++)
		{
			info.tdata[i].freq = mi->tdata[i].freq;
			info.tdata[i].pipi = info.tdata[i].pipi;	 // Just sets info.tdata[i].pipi to itself (SETPIPI already sets pipi, so maybe this line shouldn't be here in the first place)
		}
	}

	if (flag & SETWAVE)
		for (i = 0; i < MAXTRACK; i++)
			info.tdata[i].wave_no = mi->tdata[i].wave_no;

	if (flag & SETPIPI)
		for (i = 0; i < MAXTRACK; i++)
			info.tdata[i].pipi = mi->tdata[i].pipi;

	return TRUE;
}

// 指定の数だけNoteDataの領域を確保(初期化) (Allocate the specified number of NoteData areas (initialization))
BOOL OrgData::NoteAlloc(unsigned short alloc)
{
	int i,j;

	for (j = 0; j < MAXTRACK; j++)
	{
		info.tdata[j].wave_no = 0;
		info.tdata[j].note_list = NULL;	// コンストラクタにやらせたい (I want the constructor to do it)
		info.tdata[j].note_p = (NOTELIST*)malloc(sizeof(NOTELIST) * alloc);

		if (info.tdata[j].note_p == NULL)
		{
			for (i = 0; i < MAXTRACK; i++)
			{
				if (info.tdata[i].note_p != NULL)
				{
					free(info.tdata[i].note_p);
				#ifdef FIX_BUGS
					info.tdata[i].note_p = NULL;
				#else
					info.tdata[j].note_p = NULL;	// Uses j instead of i
				#endif
				}
			}

			return FALSE;
		}

		for (i = 0; i < alloc; i++)
		{
			(info.tdata[j].note_p + i)->from = NULL;
			(info.tdata[j].note_p + i)->to = NULL;
			(info.tdata[j].note_p + i)->length = 0;
			(info.tdata[j].note_p + i)->pan = PANDUMMY;
			(info.tdata[j].note_p + i)->volume = VOLDUMMY;
			(info.tdata[j].note_p + i)->y = KEYDUMMY;
		}
	}

	for (j = 0; j < MAXMELODY; j++)
		MakeOrganyaWave(j, info.tdata[j].wave_no, info.tdata[j].pipi);

	track = 0;	// 今はここに書いておく (Write here now)

	return TRUE;
}

// NoteDataを開放 (Release NoteData)
void OrgData::ReleaseNote(void)
{
	for (int i = 0; i < MAXTRACK; i++)
	{
		if (info.tdata[i].note_p != NULL)
		{
			free(info.tdata[i].note_p);
			info.tdata[i].note_p = NULL;
		}
	}
}

char pass[7] = "Org-01";
char pass2[7] = "Org-02";	// Pipi

BOOL OrgData::InitMusicData(const char *path)
{
	ORGANYADATA org_data;
	NOTELIST *np;
	int i,j;
	char pass_check[6];
	char ver = 0;

	HRSRC hrscr = FindResourceA(NULL, path, "ORG");
	if (hrscr == NULL)
		return FALSE;

	unsigned char *p = (unsigned char*)LockResource(LoadResource(0, hrscr));

	memcpy(&pass_check[0], p, 6);
	p += 6;

	if(memcmp(pass_check, pass, 6) == 0)
		ver = 1;
	if(memcmp(pass_check, pass2, 6) == 0)
		ver = 2;

	if(ver == 0)
		return FALSE;

	// 曲情報の読み込み (Loading song information)
	memcpy(&org_data, p, sizeof(ORGANYADATA));
	p += sizeof(ORGANYADATA);

	// 曲の情報を設定 (Set song information)
	info.wait = org_data.wait;
	info.line = org_data.line;
	info.dot = org_data.dot;
	info.repeat_x = org_data.repeat_x;
	info.end_x = org_data.end_x;

	for (i = 0; i < MAXTRACK; i++)
	{
		info.tdata[i].freq = org_data.tdata[i].freq;

		if (ver == 1)
			info.tdata[i].pipi = 0;
		else
			info.tdata[i].pipi = org_data.tdata[i].pipi;

		info.tdata[i].wave_no = org_data.tdata[i].wave_no;
	}

	// 音符のロード (Loading notes)
	for (j = 0; j < MAXTRACK; j++)
	{
		// 最初の音符はfromがNULLとなる (The first note has from as NULL)
		if (org_data.tdata[j].note_num == 0)
		{
			info.tdata[j].note_list = NULL;
			continue;
		}

		// リストを作る (Make a list)
		np = info.tdata[j].note_p;
		info.tdata[j].note_list = info.tdata[j].note_p;
		np->from = NULL;
		np->to = (np + 1);
		np++;

		for (i = 1; i < org_data.tdata[j].note_num; i++)
		{
			np->from = (np - 1);
			np->to = (np + 1);
			np++;
		}

		// 最後の音符のtoはNULL (The last note to is NULL)
		np--;
		np->to = NULL;

		// 内容を代入 (Assign content)
		np = info.tdata[j].note_p;	// Ｘ座標 (X coordinate)
		for (i = 0; i < org_data.tdata[j].note_num; i++)
		{
			memcpy(&np->x, p, sizeof(long));
			p += sizeof(long);
			np++;
		}

		np = info.tdata[j].note_p;	// Ｙ座標 (Y coordinate)
		for (i = 0; i < org_data.tdata[j].note_num; i++)
		{
			memcpy(&np->y, p, sizeof(unsigned char));
			p += sizeof(unsigned char);
			np++;
		}

		np = info.tdata[j].note_p;	// 長さ (Length)
		for (i = 0; i < org_data.tdata[j].note_num; i++)
		{
			memcpy(&np->length, p, sizeof(unsigned char));
			p += sizeof(unsigned char);
			np++;
		}

		np = info.tdata[j].note_p;	// ボリューム (Volume)
		for (i = 0; i < org_data.tdata[j].note_num; i++)
		{
			memcpy(&np->volume, p, sizeof(unsigned char));
			p += sizeof(unsigned char);
			np++;
		}

		np = info.tdata[j].note_p;	// パン (Pan)
		for (i = 0; i < org_data.tdata[j].note_num; i++)
		{
			memcpy(&np->pan, p, sizeof(unsigned char));
			p += sizeof(unsigned char);
			np++;
		}
	}

	// データを有効に (Enable data)
	for (j = 0; j < MAXMELODY; j++)
		MakeOrganyaWave(j,info.tdata[j].wave_no, info.tdata[j].pipi);

	// Pixel ripped out some code so he could use PixTone sounds as drums, but he left this dead code
	for (j = MAXMELODY; j < MAXTRACK; j++)
	{
		i = info.tdata[j].wave_no;
		//InitDramObject(dram_name[i], j - MAXMELODY);
	}

	SetPlayPointer(0);	// 頭出し (Cue)

	return TRUE;
}

// 曲情報を取得 (Get song information)
void OrgData::GetMusicInfo(MUSICINFO *mi)
{
	mi->dot = info.dot;
	mi->line = info.line;
	mi->alloc_note = info.alloc_note;
	mi->wait = info.wait;
	mi->repeat_x = info.repeat_x;
	mi->end_x = info.end_x;

	for (int i = 0; i < MAXTRACK; i++)
	{
		mi->tdata[i].freq = info.tdata[i].freq;
		mi->tdata[i].wave_no = info.tdata[i].wave_no;
		mi->tdata[i].pipi = info.tdata[i].pipi;
	}
}

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
//プロトタイプ宣言 (prototype declaration)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/

BOOL InitMMTimer();
BOOL StartTimer(DWORD dwTimer);
//VOID CALLBACK TimerProc(UINT uTID,UINT uMsg,DWORD dwUser,DWORD dwParam1,DWORD dwParam2);	// The original code used the wrong types
VOID CALLBACK TimerProc(UINT uTID,UINT uMsg,DWORD_PTR dwUser,DWORD_PTR dwParam1,DWORD_PTR dwParam2);
BOOL QuitMMTimer();

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
//グローバル変数 (Global variable)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
UINT ExactTime   = 13;	// 最小精度 (Minimum accuracy)
UINT TimerID;
BOOL bTimer;

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
// タイマー精度を設定する。 (Set timer accuracy.)
// この関数はアプリケーション初期化時に一度呼び出す。 (This function is called once when the application is initialized.)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
BOOL InitMMTimer(void)
{
	TIMECAPS tc;
	MMRESULT ret;

	// タイマーの精度情報を取得する (Get timer accuracy information)
	ret = timeGetDevCaps(&tc,sizeof(TIMECAPS));
	if (ret != TIMERR_NOERROR)
		return FALSE;

	if (ExactTime < tc.wPeriodMin)
		ExactTime = tc.wPeriodMin;

	// この精度で初期化する (Initialize with this precision)
	ret = timeBeginPeriod(ExactTime);
	if (ret != TIMERR_NOERROR)
		return FALSE;

	return TRUE;
}

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
// タイマーを起動する。 (Start the timer.)
// dwTimer   設定するタイマー間隔 (dwTimer   Timer interval to be set)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
BOOL StartTimer(DWORD dwTimer)
{
	MMRESULT ret = MMSYSERR_NOERROR;
	ExactTime = dwTimer;

	// タイマーを生成する (Generate timer)
	TimerID = timeSetEvent
	(
		dwTimer,                   // タイマー時間 (Timer time)
		10,                        // 許容できるタイマー精度 (Acceptable timer accuracy)
		TimerProc,                 // コールバックプロシージャ (Callback procedure)
		0,                         // ユーザーがコールバック関数のdwUserに送る情報値 (Information value sent by user to dwUser in callback function)
		TIME_PERIODIC              // タイマー時間毎にイベントを発生させる (Generate an event every timer time)
	);

	if (ret != TIMERR_NOERROR)
		return FALSE;

	bTimer = TRUE;

	return TRUE;
}

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
// タイマーのコールバック関数 (Timer callback function)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
//VOID CALLBACK TimerProc(UINT uTID,UINT uMsg,DWORD dwUser,DWORD dwParam1,DWORD dwParam2)	// The original code used the wrong types
VOID CALLBACK TimerProc(UINT uTID,UINT uMsg,DWORD_PTR dwUser,DWORD_PTR dwParam1,DWORD_PTR dwParam2)
{
	(void)uTID;
	(void)uMsg;
	(void)dwUser;
	(void)dwParam1;
	(void)dwParam2;

	DWORD dwNowTime;
	dwNowTime = timeGetTime();
	//===================================================================================
	// ここにユーザー定義のソースを書く。 (Write user-defined source here.)
	// 基本的に関数を呼び出すだけで処理は他の関数でするべきだろう。 (Basically just call a function and the process should be another function.)
	//===================================================================================
	org_data.PlayData();
}

/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
// タイマーリソースを開放する。 (Release timer resources.)
// アプリケーション終了時に一度呼び出す。 (Call once when the application ends.)
/*■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■*/
BOOL QuitMMTimer(void)
{
	MMRESULT ret;

	if (!bTimer)
		return FALSE;

	if(TimerID != TIMERR_NOERROR)
	{
		// タイマーを使用中なら終了させる (Terminate timer if in use)
		ret = timeKillEvent(TimerID);
		if (ret != TIMERR_NOERROR)
			return FALSE;
	}

	// タイマーリソースを開放する (Release timer resources)
	ret = timeEndPeriod(ExactTime);
	if (ret != TIMERR_NOERROR)
		return FALSE;

	bTimer = FALSE;

	return TRUE;
}

// Play data
long PlayPos;	// Called 'play_p' in the source code release
NOTELIST *np[MAXTRACK];
long now_leng[MAXMELODY];

int Volume = 100;
int TrackVol[MAXTRACK];
BOOL bFadeout = FALSE;

void OrgData::PlayData(void)
{
	int i;

	// Handle fading out
	if (bFadeout && Volume)
		Volume -= 2;
	if (Volume < 0)
		Volume = 0;

	// メロディの再生 (Play melody)
	for (i = 0; i < MAXMELODY; i++)
	{
		if (np[i] != NULL && PlayPos == np[i]->x)
		{
			if (!g_mute[i] && np[i]->y != KEYDUMMY)	// 音が来た。 (The sound has come.)
			{
				PlayOrganObject(np[i]->y, -1, i, info.tdata[i].freq);
				now_leng[i] = np[i]->length;
			}

			if (np[i]->pan != PANDUMMY)
				ChangeOrganPan(np[i]->y, np[i]->pan, i);
			if (np[i]->volume != VOLDUMMY)
				TrackVol[i] = np[i]->volume;

			np[i] = np[i]->to;	// 次の音符を指す (Points to the next note)
		}

		if (now_leng[i] == 0)
			PlayOrganObject(0, 2, i, info.tdata[i].freq);

		if (now_leng[i] > 0)
			now_leng[i]--;

		if (np[i])
			ChangeOrganVolume(np[i]->y, TrackVol[i] * Volume / 0x7F, i);
	}

	// ドラムの再生 (Drum playback)
	for (i = MAXMELODY; i < MAXTRACK; i++)
	{
		if (np[i] != NULL && PlayPos == np[i]->x)	// 音が来た。 (The sound has come.)
		{
			if (np[i]->y != KEYDUMMY && !g_mute[i])	// ならす (Tame)
				PlayDramObject(np[i]->y, 1, i - MAXMELODY);

			if (np[i]->pan != PANDUMMY)
				ChangeDramPan(np[i]->pan, i - MAXMELODY);
			if (np[i]->volume != VOLDUMMY)
				TrackVol[i] = np[i]->volume;

			np[i] = np[i]->to;	// 次の音符を指す (Points to the next note)
		}

		if (np[i])
			ChangeDramVolume(TrackVol[i] * Volume / 0x7F, i - MAXMELODY);
	}

	// Looping
	PlayPos++;
	if (PlayPos >= info.end_x)
	{
		PlayPos = info.repeat_x;
		SetPlayPointer(PlayPos);
	}
}

void OrgData::SetPlayPointer(long x)
{
	for (int i = 0; i < MAXTRACK; i++)
	{
		np[i] = info.tdata[i].note_list;
		while (np[i] != NULL && np[i]->x < x)
			np[i] = np[i]->to;	// 見るべき音符を設定 (Set note to watch)
	}

	PlayPos = x;
}

// Start and end organya
BOOL StartOrganya(LPDIRECTSOUND _lpDS, const char *path_wave)	// Both arguments are ignored for some reason
{
	if (lpDS == NULL)
		return FALSE;

	if (!InitWaveData100())
		return FALSE;

	org_data.InitOrgData();

	return TRUE;
}

// Load organya file
BOOL LoadOrganya(const char *name)
{
	if (lpDS == NULL)
		return FALSE;

	if (!org_data.InitMusicData(name))
		return FALSE;

	Volume = 100;
	bFadeout = 0;

#ifdef FIX_BUGS
	return TRUE;
#else
	return FALSE;	// Err... isn't this meant to be 'TRUE'?
#endif
}

void SetOrganyaPosition(unsigned int x)
{
	if (lpDS == NULL)
		return;

	org_data.SetPlayPointer(x);
	Volume = 100;
	bFadeout = FALSE;
}

unsigned int GetOrganyaPosition(void)
{
	if (lpDS == NULL)
		return 0;

	return PlayPos;
}

void PlayOrganyaMusic(void)
{
	if (lpDS == NULL)
		return;

	QuitMMTimer();
	InitMMTimer();
	StartTimer(org_data.info.wait);
}

BOOL ChangeOrganyaVolume(signed int volume)
{
	if (lpDS == NULL)
		return FALSE;

	if (volume < 0 || volume > 100)
		return FALSE;

	Volume = volume;
	return TRUE;
}

void StopOrganyaMusic(void)
{
	if (lpDS == NULL)
		return;

	// Stop timer
	QuitMMTimer();

	// Stop notes
	for (int i = 0; i < MAXMELODY; i++)
		PlayOrganObject(0, 2, i, 0);

	memset(old_key, 255, sizeof(old_key));
	memset(key_on, 0, sizeof(key_on));
	memset(key_twin, 0, sizeof(key_twin));

	// Put the main thread to sleep for 100 milliseconds... but why?
	// Really, what's the point? All this does is cause an annoying
	// stutter when a new song loads.
	// I'd guess it avoids a race-condition with the Organya thread,
	// but the earlier QuitMMTimer call already disables it.
	Sleep(100);
}

void SetOrganyaFadeout(void)
{
	bFadeout = TRUE;
}

void EndOrganya(void)
{
	if (lpDS == NULL)
		return;

	// End timer
	QuitMMTimer();

	// Release everything related to org
	org_data.ReleaseNote();

	for (int i = 0; i < MAXMELODY; i++)
	{
		PlayOrganObject(0, 0, i, 0);
		ReleaseOrganyaObject(i);
	}
}

// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "PixTone.h"

#include <math.h>
#include <string.h>

#include "WindowsWrapper.h"

signed char gWaveModelTable[6][0x100];

void MakeWaveTables(void)
{
	int i;

	int a;

	// Sine wave
	for (i = 0; i < 0x100; ++i)
	{
		gWaveModelTable[0][i] = (signed char)(sin((i * 6.283184) / 256.0) * 64.0);
		a = gWaveModelTable[0][i];	// I have no idea what this line was meant to do
	}

	// Triangle wave
	for (a = 0, i = 0; i < 0x40; ++i)
	{
		// Upwards
		gWaveModelTable[1][i] = (a * 0x40) / 0x40;
		++a;
	}
	for (a = 0; i < 0xC0; ++i)
	{
		// Downwards
		gWaveModelTable[1][i] = 0x40 - ((a * 0x40) / 0x40);
		++a;
	}
	for (a = 0; i < 0x100; ++i)
	{
		// Back up
		gWaveModelTable[1][i] = ((a * 0x40) / 0x40) - 0x40;
		++a;
	}

	// Saw up wave
	for (i = 0; i < 0x100; ++i)
		gWaveModelTable[2][i] = (i / 2) - 0x40;

	// Saw down wave
	for (i = 0; i < 0x100; ++i)
		gWaveModelTable[3][i] = 0x40 - (i / 2);

	// Square wave
	for (i = 0; i < 0x80; ++i)
		gWaveModelTable[4][i] = 0x40;
	for (; i < 0x100; ++i)
		gWaveModelTable[4][i] = -0x40;

	// White noise wave
	srand(0);
	for (i = 0; i < 0x100; ++i)
		gWaveModelTable[5][i] = (signed char)(rand() & 0xFF) / 2;
}

//BOOL wave_tables_made;

BOOL MakePixelWaveData(const PIXTONEPARAMETER *ptp, unsigned char *pData)
{
	int i;
	int a, b, c, d;

	double dPitch;
	double dMain;
	double dVolume;

	double dEnvelope;
	signed char envelopeTable[0x100];

	double d1, d2, d3;

	// The Linux port added a cute optimisation here, where MakeWaveTables is only called once during the game's execution
	//if (wave_tables_made != TRUE)
	//{
		MakeWaveTables();
	//	wave_tables_made = TRUE;
	//}

	memset(envelopeTable, 0, sizeof(envelopeTable));

	i = 0;

	dEnvelope = ptp->initial;
	while (i < ptp->pointAx)
	{
		envelopeTable[i] = (signed char)dEnvelope;
		dEnvelope = (((double)ptp->pointAy - ptp->initial) / ptp->pointAx) + dEnvelope;
		++i;
	}

	dEnvelope = ptp->pointAy;
	while (i < ptp->pointBx)
	{
		envelopeTable[i] = (signed char)dEnvelope;
		dEnvelope = (((double)ptp->pointBy - ptp->pointAy) / (double)(ptp->pointBx - ptp->pointAx)) + dEnvelope;
		++i;
	}

	dEnvelope = ptp->pointBy;
	while (i < ptp->pointCx)
	{
		envelopeTable[i] = (signed char)dEnvelope;
		dEnvelope = ((double)ptp->pointCy - ptp->pointBy) / (double)(ptp->pointCx - ptp->pointBx) + dEnvelope;
		++i;
	}

	dEnvelope = ptp->pointCy;
	while (i < 0x100)
	{
		envelopeTable[i] = (signed char)dEnvelope;
		dEnvelope = dEnvelope - (ptp->pointCy / (double)(0x100 - ptp->pointCx));
		++i;
	}

	dPitch = ptp->oPitch.offset;
	dMain = ptp->oMain.offset;
	dVolume = ptp->oVolume.offset;

	if (ptp->oMain.num == 0.0)
		d1 = 0.0;
	else
		d1 = 256.0 / (ptp->size / ptp->oMain.num);

	if (ptp->oPitch.num == 0.0)
		d2 = 0.0;
	else
		d2 = 256.0 / (ptp->size / ptp->oPitch.num);

	if (ptp->oVolume.num == 0.0)
		d3 = 0.0;
	else
		d3 = 256.0 / (ptp->size / ptp->oVolume.num);

	for (i = 0; i < ptp->size; ++i)
	{
		a = (int)dMain % 0x100;
		b = (int)dPitch % 0x100;
		c = (int)dVolume % 0x100;
		d = (int)((double)(i * 0x100) / ptp->size);
		pData[i] = gWaveModelTable[ptp->oMain.model][a]
		         * ptp->oMain.top
		         / 64
		         * (((gWaveModelTable[ptp->oVolume.model][c] * ptp->oVolume.top) / 64) + 64)
		         / 64
		         * envelopeTable[d]
		         / 64
		         + 128;

		if (gWaveModelTable[ptp->oPitch.model][b] < 0)
			dMain += d1 - d1 * 0.5 * -gWaveModelTable[ptp->oPitch.model][b] * ptp->oPitch.top / 64.0 / 64.0;
		else
			dMain += d1 + d1 * 2.0 * gWaveModelTable[ptp->oPitch.model][b] * ptp->oPitch.top / 64.0 / 64.0;

		dPitch += d2;
		dVolume += d3;
	}

	return TRUE;
}

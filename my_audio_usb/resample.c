/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2005 Miika Pekkarinen
 * Copyright (C) 2012 Michael Sevakis
 * Copyright (C) 2013 Michael Giacomelli
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#include "fracmul.h"
#include "fixedpoint.h"
#include "resample.h"
#include <string.h>
//#include "defines.h"

#define CH_LEFT		0
#define CH_RIGHT	1
/**
 * Linear interpolation resampling that introduces a one sample delay because
 * of our inability to look into the future at the end of a frame.
 */

/* Data for each resampler on each DSP */
static struct resample_data {
	unsigned int delta; /* 00h: Phase delta for each step in s15.16*/
	unsigned int phase; /* 04h: Current phase [pos16|frac16] */
	int history[2][3]; /* 08h: Last samples for interpolation (L+R)
	 0 = oldest, 2 = newest */
	/* 20h */
	unsigned int frequency; /* Virtual input samplerate */
	unsigned int frequency_out; /* Resampler output samplerate */
	//struct dsp_buffer resample_buf; /* Buffer descriptor for resampled data */
	int32_t *resample_out_p[2]; /* Actual output buffer pointers */
} resample_data[2];

/* Actual worker function. Implemented here or in target assembly code. */
int resample_hermite(struct resample_data *data, int32_t *src, int32_t *dst);

static void resample_flush_data(struct resample_data *data) {
	data->phase = 0;
	memset(&data->history, 0, sizeof(data->history));
}
void ResampleProccess(int32_t *srcL, int32_t *srcR, int32_t *dstL, int32_t *dstR) {
	resample_hermite(&resample_data[CH_LEFT], srcL, dstL);
	resample_hermite(&resample_data[CH_RIGHT], srcR, dstR);
}

void InitResample(void) {
	unsigned int frequency = 48000; /* virtual samplerate */

	resample_data[CH_LEFT].frequency_out = 768000;
	resample_data[CH_LEFT].delta = fp_div(frequency, 768000, 16);

	resample_data[CH_RIGHT].frequency_out = 768000;
	resample_data[CH_RIGHT].delta = fp_div(frequency, 768000, 16);

	resample_flush_data(&resample_data[CH_LEFT]);
	resample_flush_data(&resample_data[CH_RIGHT]);
}

int resample_hermite(struct resample_data *data, int32_t *src, int32_t *dst) {
	int ch = 1; //src->format.num_channels - 1;
	uint32_t count = MIN(48, 0x8000);
	uint32_t delta = data->delta;
	uint32_t phase, pos;
	int32_t *d;

	const int32_t *s = src;
	d = dst;

	int32_t *dmax = d + (48 * 16);

	/* Restore state */
	phase = data->phase;
	pos = phase >> 16;
	pos = MIN(pos, count);

	while (pos < count && d < dmax) {
		int x0, x1, x2, x3;

		if (pos < 3) {
			x3 = data->history[ch][pos + 0];
			x2 = pos < 2 ? data->history[ch][pos + 1] : s[pos - 2];
			x1 = pos < 1 ? data->history[ch][pos + 2] : s[pos - 1];
		} else {
			x3 = s[pos - 3];
			x2 = s[pos - 2];
			x1 = s[pos - 1];
		}

		x0 = s[pos];

		int32_t frac = (phase & 0xffff) << 15;

		/* 4-point, 3rd-order Hermite/Catmull-Rom spline (x-form):
		 * c1 = -0.5*x3 + 0.5*x1
		 *    = 0.5*(x1 - x3)                <--
		 *
		 * v = x1 - x2, -v = x2 - x1
		 * c2 = x3 - 2.5*x2 + 2*x1 - 0.5*x0
		 *    = x3 + 2*(x1 - x2) - 0.5*(x0 + x2)
		 *    = x3 + 2*v - 0.5*(x0 + x2)     <--
		 *
		 * c3 = -0.5*x3 + 1.5*x2 - 1.5*x1 + 0.5*x0
		 *    = 0.5*x0 - 0.5*x3 + 0.5*(x2 - x1) + (x2 - x1)
		 *    = 0.5*(x0 - x3 - v) - v        <--
		 *
		 * polynomial coefficients */
		int32_t c1 = (x1 - x3) >> 1;
		int32_t v = x1 - x2;
		int32_t c2 = x3 + 2 * v - ((x0 + x2) >> 1);
		int32_t c3 = ((x0 - x3 - v) >> 1) - v;

		/* Evaluate polynomial at time 'frac'; Horner's rule. */
		int32_t acc;
		acc = FRACMUL(c3, frac) + c2;
		acc = FRACMUL(acc, frac) + c1;
		acc = FRACMUL(acc, frac) + x2;

		*d++ = acc;

		phase += delta;
		pos = phase >> 16;
	}

	pos = MIN(pos, count);

	/* Save delay samples for next time. Must do this even if pos was
	 * clamped before loop in order to keep record up to date. */
	data->history[ch][0] = pos < 3 ? data->history[ch][pos + 0] : s[pos - 3];
	data->history[ch][1] = pos < 2 ? data->history[ch][pos + 1] : s[pos - 2];
	data->history[ch][2] = pos < 1 ? data->history[ch][pos + 2] : s[pos - 1];
//	} while (--ch >= 0);

	/* Wrap phase accumulator back to start of next frame. */
	data->phase = phase - (pos << 16);

	return pos;
}


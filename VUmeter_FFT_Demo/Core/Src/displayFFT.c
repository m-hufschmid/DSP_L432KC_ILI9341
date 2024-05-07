/*
 * displayFFT.c
 *
 *  Created on: 20.04.2024
 *      Author: markus.hufschmid
 */
#include "displayFFT.h"
#include "dma.h"
#include "main.h"
#include "colors.h"

void displayFFTdata(fftDisplay_t *fftDisplay, float32_t *fftData) {
	float32_t magFFT[BUFFER_SIZE >> 1];
	float32_t octValue;
	uint16_t i, iOctave, nBins, iMagFFT;

	arm_cmplx_mag_f32(fftData, magFFT, BUFFER_SIZE >> 1);
	switch (fftDisplay->style) {
	case LINEAR:
		for (i = 0; i < N_PLOT; i++) {
			int16_t y = (int16_t) (8.686 * log(2.0 * magFFT[i]) + 16.9);
			if (y > 59) {
				y = 59;
			}
			if (y < 0) {
				y = 0;
			}

			if (y > 0) {
				Displ_Line(fftDisplay->x0 + i + 4, fftDisplay->y0 + 59, fftDisplay->x0 + i + 4, fftDisplay->y0 + 60 - y, FFT_COLOR);
			}
			Displ_Line(fftDisplay->x0 + i + 4, fftDisplay->y0, fftDisplay->x0 + i + 4, fftDisplay->y0 + 59 - y, FFT_BACKGROUND);
		}
		break;
	case OCTAVE:
		iOctave = 0;
		iMagFFT = 1;
		nBins = 1;
		while (iOctave < NOCTAVES) {
			octValue = 0;
			for (i = 0; i < nBins; i++) {
				octValue += magFFT[iMagFFT] * magFFT[iMagFFT];
				iMagFFT++;
			}

			if (octValue >= fftDisplay->octValues[iOctave]) {
				fftDisplay->octValues[iOctave] = octValue;
			} else {
				fftDisplay->octValues[iOctave] = 0.8 * fftDisplay->octValues[iOctave];
			}

			iOctave++;
			nBins = nBins << 1;
		}
		for (i = 0; i < NOCTAVES; i++) {
			int16_t y = (int16_t) (-4.343 * log(fftDisplay->octValues[i]) + 42.1);
			if (y > 59) {
				y = 59;
			}
			if (y < 0) {
				y = 0;
			}
			Displ_FillArea(fftDisplay->x0 + 19 * i + 3, fftDisplay->y0 + y + 1, 15, 59 - y, FFT_COLOR);
			if (y > 0) {
				Displ_FillArea(fftDisplay->x0 + 19 * i + 3, fftDisplay->y0 + 1, 15, y, FFT_BACKGROUND);
			}
		}

		break;
	}
}

void clearFFTdisplay(fftDisplay_t *fftDisplay) {
	Displ_FillArea(fftDisplay->x0, fftDisplay->y0, FFT_WIDTH, FFT_HEIGHT, FFT_BACKGROUND);
}

void initFFTdisplay(fftDisplay_t *fftDisplay, uint16_t x0, uint16_t y0, enum displayFFTstyle_t style, float32_t *octValues){
	fftDisplay->x0 = x0;
	fftDisplay->y0 = y0;
	fftDisplay->style = style;
	fftDisplay->octValues = octValues;
	redrawFFTdisplay(fftDisplay);
}

void redrawFFTdisplay(fftDisplay_t *fftDisplay) {
	Displ_fillRoundRect(fftDisplay->x0 - 5, fftDisplay->y0 - 5, FFT_WIDTH + 10, FFT_HEIGHT + 10, 3, FRAME_COLOR);
	clearFFTdisplay(fftDisplay);
	switch(fftDisplay->style){
	case OCTAVE:
		Displ_FillArea(fftDisplay->x0, fftDisplay->y0+FFT_HEIGHT+7,fftDisplay->x0+FFT_WIDTH,fftDisplay->y0+FFT_HEIGHT+10,BACKGROUND_COLOR);
		Displ_CString(fftDisplay->x0 - 12, fftDisplay->y0 + FFT_HEIGHT + 12, fftDisplay->x0 + FFT_WIDTH + 12, Font12.Height + fftDisplay->y0 + FFT_HEIGHT + 12,"octaves", Font12, 1, TEXT_COLOR, BACKGROUND_COLOR);
		break;
	case LINEAR:
		Displ_CString(fftDisplay->x0, fftDisplay->y0 + FFT_HEIGHT + 12, fftDisplay->x0 + FFT_WIDTH, Font12.Height + fftDisplay->y0 + FFT_HEIGHT + 12, "frequency [kHz]",Font12, 1, TEXT_COLOR, BACKGROUND_COLOR);
		Displ_CString(fftDisplay->x0 - 8, fftDisplay->y0 + FFT_HEIGHT + 12, fftDisplay->x0 + 16, Font8.Height + fftDisplay->y0 + FFT_HEIGHT + 12, "0", Font8, 1,TEXT_COLOR, BACKGROUND_COLOR);
		Displ_CString(fftDisplay->x0 + FFT_WIDTH - 14, fftDisplay->y0 + FFT_HEIGHT + 12, fftDisplay->x0 + FFT_WIDTH + 6,Font8.Height + fftDisplay->y0 + FFT_HEIGHT + 12, "24", Font8, 1, TEXT_COLOR, BACKGROUND_COLOR);

		float x = 0.0;
		while (x < FFT_WIDTH){
			Displ_Line(fftDisplay->x0+(uint16_t)x+4, fftDisplay->y0 + FFT_HEIGHT + 10, fftDisplay->x0+(uint16_t)x+4, fftDisplay->y0 + FFT_HEIGHT + 7, TEXT_COLOR);
			x += 10.667;
		}
		break;
	}
}


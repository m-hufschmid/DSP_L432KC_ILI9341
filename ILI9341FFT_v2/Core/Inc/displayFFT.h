/*
 * displayFFT.h
 *
 *  Created on: 20.04.2024
 *      Author: markus.hufschmid
 */

#include <stdint.h>
#include <arm_math.h>

#ifndef INC_DISPLAYFFT_H_
#define INC_DISPLAYFFT_H_

#define NOCTAVES 7
#define N_PLOT 128

#define FFT_WIDTH 135
#define FFT_HEIGHT 60

enum displayFFTstyle_t {
	LINEAR, OCTAVE
};

typedef struct FFT_DISPLAY {
	uint16_t x0;
	uint16_t y0;
	enum displayFFTstyle_t style;
	float32_t *octValues;
}fftDisplay_t;

void initFFTdisplay(fftDisplay_t *fftDisplay, uint16_t x0, uint16_t y0, enum displayFFTstyle_t style, float32_t *octValues);
void redrawFFTdisplay(fftDisplay_t *fftDisplay);
void displayFFTdata(fftDisplay_t *fftDisplay, float32_t *fftData);


#endif /* INC_DISPLAYFFT_H_ */

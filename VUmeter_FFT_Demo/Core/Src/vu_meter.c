/*
 * vu_meter.c
 *
 *  Created on: 26.03.2024
 *      Author: markus.hufschmid
 */
#include <math.h>
#include "main.h"
#include "colors.h"

void displayVUmeter(uint16_t x0, uint16_t y0, int16_t percentage) {
	int16_t x1, x2, y1, y2;
	float angle;

	if (percentage > 100) {
		percentage = 100;
	}
	ILI9488_DrawImage(x0, y0, VU_WIDTH, VU_HEIGHT, (uint8_t*) vu_meter_135x59, VU_WIDTH * VU_HEIGHT * 2);
	angle = (100.0 - percentage) * 1.68 / 100.0 + 0.73;
	x1 = (int16_t) (x0 + 66.1 + 68 * cos(angle));
	y1 = (int16_t) (y0 + 79.5 - 68 * sin(angle));
	x2 = (int16_t) (x0 + 66.1 + 20.5 * cos(angle) / sin(angle));
	y2 = (int16_t) (y0 + 59);
	Displ_Line(x1, y1, x2, y2, BLACK);
}

void initVUmeter(uint16_t x0, uint16_t y0) {
	Displ_fillRoundRect(x0 - 5, y0 - 5, VU_WIDTH + 10, VU_HEIGHT + 10, 3, FRAME_COLOR);
	ILI9488_DrawImage(x0, y0, VU_WIDTH, VU_HEIGHT, (uint8_t*) vu_meter_135x59, VU_WIDTH * VU_HEIGHT * 2);
}

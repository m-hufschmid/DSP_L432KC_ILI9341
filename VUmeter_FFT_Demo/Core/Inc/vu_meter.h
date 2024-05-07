/*
 * vu_meter.h
 *
 *  Created on: 26.03.2024
 *      Author: markus.hufschmid
 */

#ifndef INC_VU_METER_H_
#define INC_VU_METER_H_

#define VU_WIDTH 135
#define VU_HEIGHT 59

void displayVUmeter(uint16_t x0, uint16_t y0, int16_t percentage);
void initVUmeter(uint16_t x0, uint16_t y0);

#endif /* INC_VU_METER_H_ */

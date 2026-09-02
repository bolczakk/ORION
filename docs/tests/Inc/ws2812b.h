/*
 * ws2812b.h
 *
 *  Created on: 3 cze 2026
 *      Author: Oleg
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "main.h"
#include "tim.h"

#define MAX_LED 3
#define USE_BRIGHTNESS 1

// Tutaj wpisz wartości wypełnienia PWM dla stanów "0" i "1"
// (Wylicza się je na podstawie wartości ARR Timera)
#define WS2812_0 38
#define WS2812_1 77

void WS2812_Set_LED(int led_index, uint8_t Red, uint8_t Green, uint8_t Blue);
void WS2812_Set_Brightness(int brightness);
void WS2812_Send(void);
void WS2812_Clear(void);

#endif /* INC_WS2812B_H_ */

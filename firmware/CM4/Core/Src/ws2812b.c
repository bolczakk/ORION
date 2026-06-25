/*
 * ws2812b.c
 *
 *  Created on: 3 cze 2026
 *      Author: Oleg
 */

#include "ws2812b.h"

extern TIM_HandleTypeDef htim2;

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

uint16_t pwmData[(24 * MAX_LED) + 50];
int brightness = 45;

void WS2812_Set_LED(int led_index, uint8_t Red, uint8_t Green, uint8_t Blue) {
	if (led_index < MAX_LED) {
		LED_Data[led_index][0] = led_index;
		LED_Data[led_index][1] = Green;
		LED_Data[led_index][2] = Red;
		LED_Data[led_index][3] = Blue;
	}
}

void WS2812_Set_Brightness(int b) {
	if (b > 100)
		b = 100;
	brightness = b;
}

void WS2812_Send(void) {
	uint32_t indx = 0;
	uint32_t color;

	for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
		LED_Mod[i][0] = LED_Data[i][0];
		// Skalowanie kolorów przez ułamek jasności
		LED_Mod[i][1] = (LED_Data[i][1] * brightness) / 100;
		LED_Mod[i][2] = (LED_Data[i][2] * brightness) / 100;
		LED_Mod[i][3] = (LED_Data[i][3] * brightness) / 100;
#else
        LED_Mod[i][1] = LED_Data[i][1];
        LED_Mod[i][2] = LED_Data[i][2];
        LED_Mod[i][3] = LED_Data[i][3];
#endif
	}
	for (int i = 0; i < MAX_LED; i++) {
		color =
				((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) {
				pwmData[indx] = WS2812_1;
			} else {
				pwmData[indx] = WS2812_0;
			}
			indx++;
		}
	}

	for (int i = 0; i < 50; i++) {
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) pwmData, indx);
}

void WS2812_Clear(void) {
	for (int i = 0; i < MAX_LED; i++) {
		WS2812_Set_LED(i, 0, 0, 0);
	}
	WS2812_Send();
}

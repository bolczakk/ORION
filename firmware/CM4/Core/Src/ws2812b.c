/*
 * ws2812b.c
 *
 *  Created on: 3 cze 2026
 *      Author: Oleg
 */

#include "ws2812b.h"
#include <math.h>
#include <stdbool.h>
#include "shared_data.h"

#define NUM_LEDS_PER_STRIP 30
#define TOTAL_LEDS         60

#define COLOR_OFF    0,   0,   0
#define COLOR_ORANGE 255, 100, 0
#define COLOR_RED    255, 0,   0
#define COLOR_GREEN  0,   255, 0
#define COLOR_CYAN   0,   255, 255
#define COLOR_WHITE  255, 255, 255

extern TIM_HandleTypeDef htim8;

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

		for (int j = 23; j >= 0; j--) {
			if (color & (1 << j)) {
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
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t*) pwmData, indx);
}

void WS2812_Clear(void) {
	for (int i = 0; i < MAX_LED; i++) {
		WS2812_Set_LED(i, COLOR_OFF);
	}
}

LedState_t DetermineRobotState(void) {
	if (SHARED_DATA->current_error != STATUS_OK) {
		return LED_STATE_ERROR;
	}

	float turn_threshold = 0.5f;
	float move_threshold = 0.1f;

	/* Priorytet 1: Kierunkowskazy (jeśli mocno skręca) */
	if (SHARED_DATA->m7_angular_speed > turn_threshold) {
		return LED_STATE_TURN_LEFT;
	}
	if (SHARED_DATA->m7_angular_speed < -turn_threshold) {
		return LED_STATE_TURN_RIGHT;
	}

	if (SHARED_DATA->m7_linear_speed < -move_threshold) {
		return LED_STATE_REVERSE;
	}
	if (SHARED_DATA->m7_linear_speed > move_threshold) {
		return LED_STATE_FORWARD;
	}

	return LED_STATE_IDLE;
}

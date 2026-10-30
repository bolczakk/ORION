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

extern TIM_HandleTypeDef htim8;

#define COLOR_OFF    0,   0,   0
#define COLOR_ORANGE 255, 100, 0
#define COLOR_RED    255, 0,   0
#define COLOR_GREEN  0,   255, 0
#define COLOR_CYAN   0,   255, 255
#define COLOR_WHITE  255, 255, 255

#define WS2812_RESET_SLOTS 300U

uint8_t LED_Data_CH1[NUM_LEDS_FRONT_LEFT][3];  // CH1 - Lewy Przód
uint8_t LED_Data_CH2[NUM_LEDS_FRONT_RIGHT][3]; // CH2 - Prawy Przód
uint8_t LED_Data_CH3[NUM_LEDS_REAR][3];        // CH3 - Tył

uint16_t pwmData1[24U * NUM_LEDS_FRONT_LEFT + 2U * WS2812_RESET_SLOTS];
uint16_t pwmData2[24U * NUM_LEDS_FRONT_RIGHT + 2U * WS2812_RESET_SLOTS];
uint16_t pwmData3[24U * NUM_LEDS_REAR + 2U * WS2812_RESET_SLOTS];

int brightness = 45;

void SetFrontLeftLED(int index, uint8_t Red, uint8_t Green, uint8_t Blue) {
	if (index < NUM_LEDS_FRONT_LEFT) {
		LED_Data_CH1[index][0] = Green;
		LED_Data_CH1[index][1] = Red;
		LED_Data_CH1[index][2] = Blue;
	}
}

void SetFrontRightLED(int index, uint8_t Red, uint8_t Green, uint8_t Blue) {
	if (index < NUM_LEDS_FRONT_RIGHT) {
		LED_Data_CH2[index][0] = Green;
		LED_Data_CH2[index][1] = Red;
		LED_Data_CH2[index][2] = Blue;
	}
}

void SetRearLED(int index, uint8_t Red, uint8_t Green, uint8_t Blue) {
	if (index < NUM_LEDS_REAR) {
		LED_Data_CH3[index][0] = Green;
		LED_Data_CH3[index][1] = Red;
		LED_Data_CH3[index][2] = Blue;
	}
}

//void WS2812_Set_LED(int led_index, uint8_t Red, uint8_t Green, uint8_t Blue) {
//	//if (led_index < MAX_LED) {
//	LED_Data[led_index][0] = led_index;
//	LED_Data[led_index][1] = Green;
//	LED_Data[led_index][2] = Red;
//	LED_Data[led_index][3] = Blue;
//	//}
//}

void WS2812_Set_Brightness(int b) {
	if (b > 100)
		b = 100;
	brightness = b;
}

static void PreparePWMData(uint8_t ledData[][3], uint16_t *pwmBuf, int numLeds) {
	uint32_t indx = 0;
	uint32_t color;

	for (uint32_t i = 0; i < WS2812_RESET_SLOTS; i++) {
		pwmBuf[indx++] = 0;
	}

	for (int i = 0; i < numLeds; i++) {
#if USE_BRIGHTNESS
		uint8_t g = (ledData[i][0] * brightness) / 100;
		uint8_t r = (ledData[i][1] * brightness) / 100;
		uint8_t b = (ledData[i][2] * brightness) / 100;
#else
        uint8_t g = ledData[i][0];
        uint8_t r = ledData[i][1];
        uint8_t b = ledData[i][2];
#endif
		color = (g << 16) | (r << 8) | b;

		for (int j = 23; j >= 0; j--) {
			if (color & (1 << j)) {
				pwmBuf[indx] = WS2812_1; // Bit '1'
			} else {
				pwmBuf[indx] = WS2812_0; // Bit '0'
			}
			indx++;
		}
	}

	for (uint32_t i = 0; i < WS2812_RESET_SLOTS; i++) {
		pwmBuf[indx++] = 0;
	}
}

void WS2812_Send(void) {
	PreparePWMData(LED_Data_CH1, pwmData1, NUM_LEDS_FRONT_LEFT);
	PreparePWMData(LED_Data_CH2, pwmData2, NUM_LEDS_FRONT_RIGHT);
	PreparePWMData(LED_Data_CH3, pwmData3, NUM_LEDS_REAR);

	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t*) pwmData1,
			sizeof(pwmData1) / sizeof(pwmData1[0]));

	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, (uint32_t*) pwmData2,
			sizeof(pwmData2) / sizeof(pwmData2[0]));

	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_4, (uint32_t*) pwmData3,
			sizeof(pwmData3) / sizeof(pwmData3[0]));
}

void WS2812_Clear(void) {
	for (int i = 0; i < NUM_LEDS_FRONT_LEFT; i++)
		SetFrontLeftLED(i, 0, 0, 0);
	for (int i = 0; i < NUM_LEDS_FRONT_RIGHT; i++)
		SetFrontRightLED(i, 0, 0, 0);
	for (int i = 0; i < NUM_LEDS_REAR; i++)
		SetRearLED(i, 0, 0, 0);
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

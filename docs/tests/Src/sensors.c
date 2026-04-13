/*
 * sensors.c
 *
 * Created on: Jan 23, 2026
 * Author: Oleg
 */

/**
 ******************************************************************************
 * @file           : sensors.c
 * @brief          : Implementation of DS18B20 Sensor Driver.
 * Handles the 1-Wire communication protocol, sensor configuration,
 * and raw data conversion.
 * @author         : Oleg Swerblewski, Dawid Sobieska
 * @date           : 2026-01-23
 ******************************************************************************
 */

#include "sensors.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

/** @brief DS18B20 Library Instance */
ds18b20_t ds18;

/** @brief Internal buffer for the last valid temperature reading */
static volatile float current_temperature = -10000.0f;

/* Public functions ----------------------------------------------------------*/

void Sensors_Init(TIM_HandleTypeDef *htim) {
	/* --- Hardware Abstraction Layer (HAL) Configuration --- */
	/* Define which GPIO and Timer are used for 1-Wire communication */
	ow_init_t ow_init_struct;
	ow_init_struct.tim_handle = htim; /**< Timer for timing (us) */
	ow_init_struct.gpio = GPIOA; /**< Port: GPIOA */
	ow_init_struct.pin = GPIO_PIN_5; /**< Pin: PA5 */
	ow_init_struct.tim_cb = NULL; /**< No custom callback needed */
	ow_init_struct.done_cb = NULL;

	/* Initialize Low-Level 1-Wire Driver */
	ds18b20_init(&ds18, &ow_init_struct);

	/* Read ROM ID (Address) of the sensor */
	ds18b20_update_rom_id(&ds18);

	/* --- Sensor Logic Configuration --- */
	/* Set resolution and alarm thresholds */
	ds18b20_config_t ds18_conf = { .alarm_high = 50, /**< High Alarm Threshold */
	.alarm_low = -50, /**< Low Alarm Threshold */
	.cnv_bit = DS18B20_CNV_BIT_10 /**< 12-bit Resolution (0.0625 C step) */
	};
	ds18b20_conf(&ds18, &ds18_conf);
}

void Sensors_Process(void) {
	/* 1. Start Temperature Conversion */
	if (ds18b20_cnv(&ds18) != OW_ERR_NONE) {
		// Błąd wysyłania komendy na magistralę - spróbujemy ponownie za 2 sekundy
		current_temperature = DS18B20_ERROR;
		return;
	}

	/* 2. Wait for thermal conversion inside the sensor (NOT bus busy) */
	// Tutaj sprawdzamy czas konwersji (do 750ms), osDelay(10) odciąża procesor
	while (!ds18b20_is_cnv_done(&ds18)) {
		osDelay(10);
	}

	/* 3. Request Data Read */
	if (ds18b20_req_read(&ds18) != OW_ERR_NONE) {
		// Błąd żądania odczytu
		current_temperature = DS18B20_ERROR;
		return;
	}

	/* 4. Wait for Data Transfer over 1-Wire */
	while (ds18b20_is_busy(&ds18)) {
		osDelay(1);
	}

	/* 5. Read and Store Temperature safely */
	int16_t raw_temp = ds18b20_read_c(&ds18);

	if (raw_temp != DS18B20_ERROR) {
		// Aktualizujemy globalną zmienną tylko, gdy odczyt był prawidłowy!
		current_temperature = raw_temp;
	} else {
		current_temperature = DS18B20_ERROR;
		// Tutaj możesz zapalić czerwoną diodę błędu, albo ustawić flagę,
		// żeby na OLED wyświetliło się np. "ERR" zamiast "-100"
	}
}

float Sensors_GetTemperature(void) {
	/* Return scaled temperature */
	/* Note: Division by 100.0f implies the library returns fixed-point
	 or an integer representation (e.g. 2500 for 25.00 C) */
	return current_temperature / 100.0f;
}

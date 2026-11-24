/*
 * distance_sensors.c
 *
 *  Created on: 27 paź 2026
 *      Author: Oleg
 */

#include "distance_sensors.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "main.h"
#include "shared_data.h"
#include "shared_ipc.h"
#include "vl6180x.h"
#include "FreeRTOS.h"
#include <stdbool.h>
#include <string.h>

#define VL61_INDEX                    0U
#define VL53_INDEX                    1U
#define INVALID_DISTANCE_CM           (-1.0f)
#define VL61_ADDRESS_7BIT             0x2AU
#define VL53_ADDRESS_HAL              (0x29U << 1)
#define SHUTDOWN_DELAY_MS             20U
#define BOOT_DELAY_MS                 20U
#define VL61_READY_TIMEOUT_MS         100U
#define VL61_INTERMEASUREMENT_MS      110U /* Retains the original register 0x0A. */
#define SAMPLE_TIMEOUT_MS             500U
#define RECOVERY_INTERVAL_MS          1000U
#define VL53_VALID_RANGE_STATUS       0U

typedef struct {
	bool online;
	uint32_t last_sample_ms; /* Progress includes completed invalid samples. */
	uint32_t last_recovery_ms;
} SensorState;

static VL6180X_Device vl61;
static VL53LX_Dev_t vl53_dev;
static SensorState vl61_state;
static SensorState vl53_state;

/* Kept for existing debugger watches. Sensor control uses private state. */
volatile uint8_t vl53_initialized = 0U;
volatile uint8_t vl53_init_step = 0U;
volatile VL53LX_Error vl53_init_status = VL53LX_ERROR_NONE;
volatile VL53LX_Error vl53_read_status = VL53LX_ERROR_NONE;
volatile HAL_StatusTypeDef vl61_io_status = HAL_OK;
volatile uint8_t vl61_range_status = 0U;

static void DelayMs(uint32_t milliseconds) {
	/* Extra tick covers the incomplete first tick of a relative delay. */
	(void) osDelay(pdMS_TO_TICKS(milliseconds) + 1U);
}

static bool Elapsed(uint32_t now, uint32_t since, uint32_t interval) {
	/* Unsigned subtraction tolerates HAL_GetTick() wrap, for these short intervals. */
	return (uint32_t) (now - since) >= interval;
}

static void Vl61Offline(void) {
	vl61_state.online = false;
	vl61_state.last_recovery_ms = HAL_GetTick();
	SHARED_DATA->m4_distances[VL61_INDEX] = INVALID_DISTANCE_CM;
	/* A failed/partly addressed chip must not reappear beside VL53 at 0x29. */
	HAL_GPIO_WritePin(VL61_SHUT_GPIO_Port, VL61_SHUT_Pin, GPIO_PIN_RESET);
}

static void Vl53Offline(void) {
	vl53_state.online = false;
	vl53_state.last_recovery_ms = HAL_GetTick();
	vl53_initialized = 0U;
	SHARED_DATA->m4_distances[VL53_INDEX] = INVALID_DISTANCE_CM;
	HAL_GPIO_WritePin(VL53_SHUT_GPIO_Port, VL53_SHUT_Pin, GPIO_PIN_RESET);
}

static HAL_StatusTypeDef WaitVl61Ready(void) {
	uint32_t started = HAL_GetTick();
	do {
		bool ready = false;
		HAL_StatusTypeDef result = VL6180X_IsReady(&vl61, &ready);
		if (result != HAL_OK) {
			return result;
		}
		if (ready) {
			return HAL_OK;
		}
		DelayMs(2U);
	} while (!Elapsed(HAL_GetTick(), started, VL61_READY_TIMEOUT_MS));
	return HAL_TIMEOUT;
}

static HAL_StatusTypeDef StartVl61(void) {
	HAL_StatusTypeDef result;
	HAL_GPIO_WritePin(VL61_SHUT_GPIO_Port, VL61_SHUT_Pin, GPIO_PIN_SET);
	DelayMs(BOOT_DELAY_MS);
	VL6180X_ResetContext(&vl61, &hi2c1);

	result = VL6180X_Init(&vl61);
	if (result == HAL_OK) {
		// zmieniamy adres VL6180X zeby nie byl taki sam jak VL53
		result = VL6180X_SetAddress(&vl61, VL61_ADDRESS_7BIT);
	}
	if (result == HAL_OK) {
		result = WaitVl61Ready();
	}
	if (result == HAL_OK) {
		result = VL6180X_SetScaling2x(&vl61);
	}
	if (result == HAL_OK) {
		result = VL6180X_StartContinuous(&vl61, VL61_INTERMEASUREMENT_MS);
	}
	vl61_io_status = result;
	if (result != HAL_OK) {
		Vl61Offline();
		return result;
	}
	vl61_state.online = true;
	vl61_state.last_sample_ms = HAL_GetTick();
	vl61_range_status = 0U;
	return HAL_OK;
}

/* Entry requirement: VL53 held in shutdown; VL61 is at 0x2A or held in shutdown. */
static VL53LX_Error StartVl53(void) {
	VL53LX_Error result;
	vl53_init_step = 0U;
	memset(&vl53_dev, 0, sizeof(vl53_dev));
	vl53_dev.i2c_slave_address = VL53_ADDRESS_HAL;
	HAL_GPIO_WritePin(VL53_SHUT_GPIO_Port, VL53_SHUT_Pin, GPIO_PIN_SET);
	DelayMs(BOOT_DELAY_MS);

	vl53_init_step = 1U;
	result = VL53LX_WaitDeviceBooted(&vl53_dev);
	if (result == VL53LX_ERROR_NONE) {
		vl53_init_step = 2U;
		result = VL53LX_DataInit(&vl53_dev);
	}
	if (result == VL53LX_ERROR_NONE) {
		vl53_init_step = 3U;
		result = VL53LX_StartMeasurement(&vl53_dev);
	}
	vl53_init_status = result;
	vl53_state.last_recovery_ms = HAL_GetTick();
	if (result != VL53LX_ERROR_NONE) {
		Vl53Offline();
		return result;
	}
	vl53_init_step = 4U;
	vl53_initialized = 1U;
	vl53_state.online = true;
	vl53_state.last_sample_ms = HAL_GetTick();
	vl53_read_status = VL53LX_ERROR_NONE;
	return VL53LX_ERROR_NONE;
}

static void RestartPair(void) {
	/* Both chips boot at 0x29. VL53 MUST stay off until VL61 is moved or disabled. */
	Vl53Offline();
	Vl61Offline();
	DelayMs(SHUTDOWN_DELAY_MS);
	(void) StartVl61();
	/* Failure of VL61 must not prevent operation of VL53. */
	(void) StartVl53();
	vl61_state.last_recovery_ms = HAL_GetTick();
}

void DistanceSensors_Init(void) {
	vl61_state = (SensorState ) { 0 };
	vl53_state = (SensorState ) { 0 };
	RestartPair();
}

static void PollVl61(void) {
	VL6180X_Sample sample;
	if (!vl61_state.online) {
		return;
	}
	vl61_io_status = VL6180X_ReadRange(&vl61, &sample);
	if (vl61_io_status != HAL_OK) {
		Vl61Offline();
		return;
	}
	if (sample.ready) {
		vl61_state.last_sample_ms = HAL_GetTick();
		vl61_range_status = sample.range_status;
		SHARED_DATA->m4_distances[VL61_INDEX] =
				sample.valid ? (float) sample.distance_mm / 10.0f :
				INVALID_DISTANCE_CM;
	} else if (Elapsed(HAL_GetTick(), vl61_state.last_sample_ms,
	SAMPLE_TIMEOUT_MS)) {
		vl61_io_status = HAL_TIMEOUT;
		Vl61Offline();
	}
}

static void PollVl53(void) {
	/* Large vendor structure stays out of the task stack. */
	static VL53LX_MultiRangingData_t data;
	uint8_t ready = 0U;
	VL53LX_Error result;
	if (!vl53_state.online) {
		return;
	}

	result = VL53LX_GetMeasurementDataReady(&vl53_dev, &ready);
	if (result != VL53LX_ERROR_NONE) {
		vl53_read_status = result;
		Vl53Offline();
		return;
	}
	if (ready == 0U) {
		if (Elapsed(HAL_GetTick(), vl53_state.last_sample_ms,
		SAMPLE_TIMEOUT_MS)) {
			vl53_read_status = VL53LX_ERROR_TIME_OUT;
			Vl53Offline();
		}
		return;
	}

	result = VL53LX_GetMultiRangingData(&vl53_dev, &data);
	if (result != VL53LX_ERROR_NONE) {
		vl53_read_status = result;
		Vl53Offline();
		return;
	}
	float nearest_cm = INVALID_DISTANCE_CM;
	for (uint8_t i = 0U;
			i < data.NumberOfObjectsFound && i < VL53LX_MAX_RANGE_RESULTS;
			++i) {
		if (data.RangeData[i].RangeStatus == VL53_VALID_RANGE_STATUS
				&& data.RangeData[i].RangeMilliMeter >= 0) {
			float object_cm = (float) data.RangeData[i].RangeMilliMeter / 10.0f;
			if (nearest_cm < 0.0f || object_cm < nearest_cm) {
				nearest_cm = object_cm;
			}
		}
	}
	result = VL53LX_ClearInterruptAndStartMeasurement(&vl53_dev);
	vl53_read_status = result;
	if (result != VL53LX_ERROR_NONE) {
		Vl53Offline();
		return;
	}
	vl53_state.last_sample_ms = HAL_GetTick();
	SHARED_DATA->m4_distances[VL53_INDEX] = nearest_cm;
}

void DistanceSensors_Process(void) {
	uint32_t now = HAL_GetTick();
	if (!vl61_state.online && Elapsed(now, vl61_state.last_recovery_ms,
	RECOVERY_INTERVAL_MS)) {
		RestartPair();
	}
	now = HAL_GetTick();
	if (!vl53_state.online && Elapsed(now, vl53_state.last_recovery_ms,
	RECOVERY_INTERVAL_MS)) {
		Vl53Offline();
		DelayMs(SHUTDOWN_DELAY_MS);
		(void) StartVl53();
	}
	/* Check VL61 first: a reset chip may have returned to VL53's address 0x29. */
	PollVl61();
	PollVl53();
}
